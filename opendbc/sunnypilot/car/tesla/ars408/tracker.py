import math
from dataclasses import dataclass

from opendbc.sunnypilot.car.tesla.ars408.constants import (
  ARS408_MAX_DISTANCE_M, MeasurementState, RejectionReason,
)
from opendbc.sunnypilot.car.tesla.ars408.models import AssembledObject, CycleResult, TrackedObject, TrackerResult


NEW_TRACK_MIN_PROBABILITY = 3
ESTABLISHED_TRACK_MIN_PROBABILITY = 2
TRACK_GRACE_CYCLES = 2
STATIC_OBJECT_CORRIDOR_M = 5.5
STATIC_DYNAMIC_PROPERTIES = frozenset((1, 3, 5))

DUPLICATE_LIMITS = (1.5, 0.6, 1.5, 0.8)
HANDOVER_LIMITS = (2.5, 1.0, 2.5, 1.5)
MAX_ASSOCIATION_AGE_NS = 300_000_000
TRUCK_ECHO_CONFIRM_NS = 1_000_000_000
TRUCK_ECHO_MAX_GAP_NS = 150_000_000


@dataclass(slots=True)
class _TrackState:
  raw_id: int
  track_id: int
  obj: AssembledObject
  missed_cycles: int = 0
  measured: bool = True
  ever_measured: bool = True
  observed_ns: int | None = None
  measured_ns: int | None = None


def _object_class(obj: AssembledObject) -> int:
  return obj.extended.object_class if obj.extended is not None else 7


def _same_target(first: AssembledObject, second: AssembledObject, *, handover: bool = False, elapsed_s: float = 0.0) -> bool:
  limits = list(HANDOVER_LIMITS if handover else DUPLICATE_LIMITS)
  first_class, second_class = _object_class(first), _object_class(second)
  if first_class == 7 or second_class == 7:
    limits = [limit * 0.7 for limit in limits]
  if first_class != 7 and second_class != 7 and first_class != second_class:
    return False
  first_general, second_general = first.general, second.general
  deltas = (
    abs(first_general.d_rel + first_general.v_rel * elapsed_s - second_general.d_rel),
    abs(first_general.y_rel + first_general.yv_rel * elapsed_s - second_general.y_rel),
    abs(first_general.v_rel - second_general.v_rel),
    abs(first_general.yv_rel - second_general.yv_rel),
  )
  return all(delta <= limit for delta, limit in zip(deltas, limits, strict=True))


def rejection_reason(obj: AssembledObject, *, established: bool, ever_measured: bool,
                     max_distance_m: float = ARS408_MAX_DISTANCE_M) -> RejectionReason | None:
  state = obj.quality.measurement_state
  probability = obj.quality.probability
  general = obj.general
  if state not in tuple(state.value for state in MeasurementState) or state in (
    MeasurementState.DELETED, MeasurementState.DELETED_FOR_MERGE,
  ):
    return RejectionReason.INVALID
  minimum_probability = ESTABLISHED_TRACK_MIN_PROBABILITY if established else NEW_TRACK_MIN_PROBABILITY
  if probability < minimum_probability:
    return RejectionReason.LOW_PROBABILITY
  if state == MeasurementState.PREDICTED and (not established or not ever_measured):
    return RejectionReason.INVALID
  if not all(math.isfinite(value) for value in (
    general.d_rel, general.y_rel, general.v_rel, general.yv_rel, general.rcs,
  )):
    return RejectionReason.OUT_OF_RANGE
  if not (0.0 <= general.d_rel and math.hypot(general.d_rel, general.y_rel) <= max_distance_m and
          abs(general.y_rel) <= 100.0 and -100.0 <= general.v_rel <= 100.0 and abs(general.yv_rel) <= 60.0):
    return RejectionReason.OUT_OF_RANGE
  if general.dynamic_property in STATIC_DYNAMIC_PROPERTIES and abs(general.y_rel) > STATIC_OBJECT_CORRIDOR_M:
    return RejectionReason.STATIC_OUTSIDE_CORRIDOR
  return None


class ARS408Tracker:
  def __init__(self) -> None:
    self._tracks: dict[int, _TrackState] = {}
    self._used_track_ids: set[int] = set()
    self._next_track_id = 256
    self.handover_count = 0
    self.duplicate_suppression_count = 0
    self._truck_echo_history: dict[tuple[int, int, int], tuple[int, int, float, int]] = {}
    self._truck_echo_counter: int | None = None

  def update(self, cycle: CycleResult, *, max_distance_m: float = ARS408_MAX_DISTANCE_M,
             timestamp_ns: int | None = None) -> TrackerResult:
    # Invalid protocol data cannot refresh a track or suspend its expiry.
    objects = {} if cycle.invalid else {obj.raw_id: obj for obj in cycle.objects}
    for raw_id, obj in objects.items():
      track = self._tracks.get(raw_id)
      if track is not None and obj.quality.measurement_state == MeasurementState.NEW:
        elapsed_s = self._association_elapsed(track, timestamp_ns)
        # A recycled sensor ID must not inherit the old object's identity or
        # established-track probability gate. Continuous NEW reports may repeat.
        if elapsed_s is None or not _same_target(track.obj, obj, handover=True, elapsed_s=elapsed_s):
          del self._tracks[raw_id]
    # New IDs must qualify independently before they can inherit an identity.
    handover_objects = {raw_id: obj for raw_id, obj in objects.items() if raw_id in self._tracks or
                        rejection_reason(obj, established=False, ever_measured=False, max_distance_m=max_distance_m) is None}
    self._explicit_merge_handover(handover_objects, timestamp_ns)
    self._kinematic_handover(handover_objects, timestamp_ns)

    accepted: dict[int, AssembledObject] = {}
    rejected: dict[int, RejectionReason] = {}
    for raw_id, obj in objects.items():
      track = self._tracks.get(raw_id)
      reason = rejection_reason(
        obj, established=track is not None, ever_measured=bool(track and track.ever_measured), max_distance_m=max_distance_m,
      )
      if reason is None:
        accepted[raw_id] = obj
      else:
        rejected[raw_id] = reason

    suppressed = self._suppress_duplicates(accepted)
    for raw_id in suppressed:
      accepted.pop(raw_id, None)
      rejected[raw_id] = RejectionReason.DUPLICATE

    for raw_id, obj in accepted.items():
      state = obj.quality.measurement_state
      measured = state in (MeasurementState.NEW, MeasurementState.MEASURED, MeasurementState.NEW_FROM_MERGE)
      track = self._tracks.get(raw_id)
      if track is None:
        track = _TrackState(raw_id, self._allocate_track_id(raw_id), obj, measured=measured,
                            ever_measured=measured, observed_ns=timestamp_ns, measured_ns=timestamp_ns if measured else None)
        self._tracks[raw_id] = track
      else:
        track.obj = obj
        track.missed_cycles = 0
        track.measured = measured
        track.ever_measured |= measured
        track.observed_ns = timestamp_ns
        if measured:
          track.measured_ns = timestamp_ns

    for raw_id in list(self._tracks):
      if raw_id in accepted:
        continue
      track = self._tracks[raw_id]
      track.missed_cycles += 1
      track.measured = False
      if track.missed_cycles > TRACK_GRACE_CYCLES:
        del self._tracks[raw_id]
        rejected.setdefault(raw_id, RejectionReason.TIMEOUT)

    # Keep the sensor tracks internally: a failed cluster condition must expose
    # occupancy again immediately, including the ordinary missed-cycle grace.
    truck_echoes = self._truck_echoes(cycle, timestamp_ns)
    for raw_id in truck_echoes:
      rejected[raw_id] = RejectionReason.DUPLICATE
    self.duplicate_suppression_count += len(truck_echoes)
    return self._result(tuple(sorted(set(accepted) - truck_echoes)), rejected, excluded=truck_echoes)

  def _truck_echoes(self, cycle: CycleResult, timestamp_ns: int | None) -> set[int]:
    fresh_measurement = cycle.status.measurement_counter != self._truck_echo_counter
    self._truck_echo_counter = cycle.status.measurement_counter
    if not cycle.exact or cycle.invalid or timestamp_ns is None:
      self._truck_echo_history.clear()
      return set()
    echoes = [track for track in self._tracks.values() if track.obj.extended is not None
              and _object_class(track.obj) != 2 and track.obj.general.dynamic_property == 0
              and 0 < (track.obj.extended.length_m or 0.) <= 5.0
              and 0 < (track.obj.extended.width_m or 0.) <= 2.5]
    candidates: dict[int, list[tuple[_TrackState, _TrackState, _TrackState, float]]] = {}
    for anchor in self._tracks.values():
      a = anchor.obj
      if (not anchor.measured or a.quality.measurement_state != MeasurementState.MEASURED
          or a.extended is None or _object_class(a) != 2 or a.quality.probability < 6
          or not 6.0 <= (a.extended.length_m or 0.) <= 25.0
          or not 1.5 <= (a.extended.width_m or 0.) <= 3.5
          or a.general.rcs < 15.0 or a.general.dynamic_property != 0):
        continue
      for body in self._tracks.values():
        b = body.obj
        spacing = b.general.d_rel - a.general.d_rel
        if (body is anchor or not body.measured or b.quality.measurement_state != MeasurementState.MEASURED
            or b.extended is None or _object_class(b) not in (1, 2)
            or b.quality.probability < 5 or b.general.dynamic_property != 0
            or not 3.0 <= spacing <= min(a.extended.length_m, 12.0)
            or abs(b.general.y_rel - a.general.y_rel) > min(a.extended.width_m / 2 + .3, 1.5)
            or abs(b.general.v_rel - a.general.v_rel) > .25
            or abs(b.general.yv_rel - a.general.yv_rel) > .5):
          continue
        for echo in echoes:
          e = echo.obj
          if (echo is anchor or echo is body or e.quality.measurement_state not in (MeasurementState.MEASURED, MeasurementState.PREDICTED)
              or e.general.rcs > a.general.rcs - 10.0
              or abs((e.general.d_rel - a.general.d_rel) - 2 * spacing) > .6
              or abs(e.general.y_rel - a.general.y_rel) > min(a.extended.width_m / 2 + .3, 1.5)
              or abs(e.general.v_rel - a.general.v_rel) > .25
              or abs(e.general.yv_rel - a.general.yv_rel) > .5):
            continue
          candidates.setdefault(echo.raw_id, []).append((anchor, body, echo, spacing))
    history = {}
    suppressed = set()
    for raw_id, matches in candidates.items():
      if len(matches) != 1:
        continue
      anchor, body, echo, spacing = matches[0]
      key = (anchor.track_id, body.track_id, echo.track_id)
      previous = self._truck_echo_history.get(key)
      continuous = (previous is not None and 0 < timestamp_ns - previous[3] <= TRUCK_ECHO_MAX_GAP_NS
                    and abs(spacing - previous[2]) <= .4)
      # Repeated sensor cycles retain evidence only within its original age
      # bound. They cannot create a relation, refresh it or complete confirmation.
      if not fresh_measurement and (not continuous or timestamp_ns - previous[1] > TRUCK_ECHO_MAX_GAP_NS):
        continue
      since = previous[0] if continuous else timestamp_ns
      confirmed = continuous and (previous[1] - since >= TRUCK_ECHO_CONFIRM_NS or
                                  (fresh_measurement and echo.measured and timestamp_ns - since >= TRUCK_ECHO_CONFIRM_NS))
      # A prediction may retain a proven relation, never establish one.
      if not echo.measured and not confirmed:
        continue
      history[key] = (since, timestamp_ns if fresh_measurement else previous[1],
                      previous[2] if continuous else spacing, timestamp_ns)
      # Measured echoes remain available to the existing lead fusion. Only a
      # confirmed prediction can be removed from the independent occupancy list.
      if confirmed and not echo.measured:
        suppressed.add(raw_id)
    self._truck_echo_history = history
    return suppressed

  def _allocate_track_id(self, raw_id: int) -> int:
    if raw_id not in self._used_track_ids:
      track_id = raw_id
    else:
      while self._next_track_id in self._used_track_ids:
        self._next_track_id += 1
      track_id = self._next_track_id
      self._next_track_id += 1
    self._used_track_ids.add(track_id)
    return track_id

  def _transfer(self, old_raw_id: int, new_raw_id: int) -> None:
    track = self._tracks.pop(old_raw_id)
    track.raw_id = new_raw_id
    track.missed_cycles = 0
    self._tracks[new_raw_id] = track
    self.handover_count += 1

  def _explicit_merge_handover(self, objects: dict[int, AssembledObject], timestamp_ns: int | None) -> None:
    deleted = [raw_id for raw_id, obj in objects.items()
               if obj.quality.measurement_state == MeasurementState.DELETED_FOR_MERGE and raw_id in self._tracks]
    new = [raw_id for raw_id, obj in objects.items()
           if obj.quality.measurement_state == MeasurementState.NEW_FROM_MERGE and raw_id not in self._tracks]
    candidates_by_new = {}
    for new_raw_id in new:
      candidates = [old_raw_id for old_raw_id in deleted if old_raw_id in self._tracks and
                    self._association_elapsed(self._tracks[old_raw_id], timestamp_ns) is not None and
                    _same_target(objects[old_raw_id], objects[new_raw_id], handover=True)]
      candidates_by_new[new_raw_id] = candidates
    for new_raw_id, candidates in candidates_by_new.items():
      if len(candidates) == 1 and sum(candidates[0] in ids for ids in candidates_by_new.values()) == 1:
        self._transfer(candidates[0], new_raw_id)

  @staticmethod
  def _association_elapsed(track: _TrackState, timestamp_ns: int | None) -> float | None:
    if timestamp_ns is None or track.observed_ns is None:
      return 0.0
    if track.measured_ns is not None and not 0 <= timestamp_ns - track.measured_ns <= MAX_ASSOCIATION_AGE_NS:
      return None
    elapsed_ns = timestamp_ns - track.observed_ns
    # Prediction is for association only; it never refreshes a measured point
    # or extends the existing missing-cycle grace. Reject old/reversed time.
    return elapsed_ns / 1e9 if 0 <= elapsed_ns <= MAX_ASSOCIATION_AGE_NS else None

  def _kinematic_handover(self, objects: dict[int, AssembledObject], timestamp_ns: int | None) -> None:
    active_ids = {raw_id for raw_id, obj in objects.items()
                  if obj.quality.measurement_state not in (MeasurementState.DELETED, MeasurementState.DELETED_FOR_MERGE)}
    missing = [raw_id for raw_id in self._tracks if raw_id not in active_ids]
    new = [raw_id for raw_id in active_ids if raw_id not in self._tracks]
    candidates_by_new = {}
    for new_raw_id in new:
      candidates = []
      for old_raw_id in missing:
        track = self._tracks[old_raw_id]
        elapsed_s = self._association_elapsed(track, timestamp_ns)
        if elapsed_s is not None and _same_target(track.obj, objects[new_raw_id], elapsed_s=elapsed_s):
          candidates.append(old_raw_id)
      candidates_by_new[new_raw_id] = candidates
    for new_raw_id, candidates in candidates_by_new.items():
      if len(candidates) == 1 and sum(candidates[0] in ids for ids in candidates_by_new.values()) == 1:
        self._transfer(candidates[0], new_raw_id)

  def _suppress_duplicates(self, objects: dict[int, AssembledObject]) -> set[int]:
    suppressed: set[int] = set()
    raw_ids = sorted(objects)
    for index, first_id in enumerate(raw_ids):
      if first_id in suppressed:
        continue
      for second_id in raw_ids[index + 1:]:
        if second_id in suppressed or (first_id in self._tracks and second_id in self._tracks):
          continue
        if not _same_target(objects[first_id], objects[second_id]):
          continue
        first_rank = self._rank(first_id, objects[first_id])
        second_rank = self._rank(second_id, objects[second_id])
        suppressed.add(second_id if first_rank >= second_rank else first_id)
        self.duplicate_suppression_count += 1
        if first_id in suppressed:
          break
    return suppressed

  def _rank(self, raw_id: int, obj: AssembledObject) -> tuple[bool, bool, int, int, int]:
    state_rank = {MeasurementState.MEASURED: 4, MeasurementState.NEW: 3,
                  MeasurementState.NEW_FROM_MERGE: 3, MeasurementState.PREDICTED: 1}.get(
      obj.quality.measurement_state, 0,
    )
    return obj.quality.measurement_state != MeasurementState.PREDICTED, raw_id in self._tracks, state_rank, obj.quality.probability, -raw_id

  def _result(self, accepted_raw_ids: tuple[int, ...], rejected: dict[int, RejectionReason], *,
              excluded: set[int] | None = None) -> TrackerResult:
    tracks = tuple(self._to_output(track) for track in sorted(self._tracks.values(), key=lambda item: item.track_id)
                   if excluded is None or track.raw_id not in excluded)
    return TrackerResult(
      tracks=tracks, accepted_raw_ids=accepted_raw_ids,
      rejection_reasons=tuple((raw_id, reason.value) for raw_id, reason in sorted(rejected.items())),
      handover_count=self.handover_count, duplicate_suppression_count=self.duplicate_suppression_count,
    )

  @staticmethod
  def _to_output(track: _TrackState) -> TrackedObject:
    obj = track.obj
    return TrackedObject(
      raw_id=track.raw_id, track_id=track.track_id, d_rel=obj.general.d_rel, y_rel=obj.general.y_rel,
      v_rel=obj.general.v_rel, yv_rel=obj.general.yv_rel, a_rel=obj.extended.a_rel if obj.extended is not None else 0.0,
      measured=track.measured, object_class=_object_class(obj), probability=obj.quality.probability,
      dynamic_property=obj.general.dynamic_property,
    )
