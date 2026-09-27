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


@dataclass(slots=True)
class _TrackState:
  raw_id: int
  track_id: int
  obj: AssembledObject
  missed_cycles: int = 0
  measured: bool = True
  ever_measured: bool = True


def _object_class(obj: AssembledObject) -> int:
  return obj.extended.object_class if obj.extended is not None else 7


def _same_target(first: AssembledObject, second: AssembledObject, *, handover: bool = False) -> bool:
  limits = list(HANDOVER_LIMITS if handover else DUPLICATE_LIMITS)
  first_class, second_class = _object_class(first), _object_class(second)
  if first_class == 7 or second_class == 7:
    limits = [limit * 0.7 for limit in limits]
  if first_class != 7 and second_class != 7 and first_class != second_class:
    return False
  first_general, second_general = first.general, second.general
  deltas = (
    abs(first_general.d_rel - second_general.d_rel),
    abs(first_general.y_rel - second_general.y_rel),
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

  def update(self, cycle: CycleResult, *, max_distance_m: float = ARS408_MAX_DISTANCE_M) -> TrackerResult:
    # Invalid protocol data cannot refresh a track or suspend its expiry.
    objects = {} if cycle.invalid else {obj.raw_id: obj for obj in cycle.objects}
    self._explicit_merge_handover(objects)
    self._kinematic_handover(objects)

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
        track = _TrackState(raw_id, self._allocate_track_id(raw_id), obj, measured=measured, ever_measured=measured)
        self._tracks[raw_id] = track
      else:
        track.obj = obj
        track.missed_cycles = 0
        track.measured = measured
        track.ever_measured |= measured

    for raw_id in list(self._tracks):
      if raw_id in accepted:
        continue
      track = self._tracks[raw_id]
      track.missed_cycles += 1
      track.measured = False
      if track.missed_cycles > TRACK_GRACE_CYCLES:
        del self._tracks[raw_id]
        rejected.setdefault(raw_id, RejectionReason.TIMEOUT)

    return self._result(tuple(sorted(accepted)), rejected)

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

  def _explicit_merge_handover(self, objects: dict[int, AssembledObject]) -> None:
    deleted = [raw_id for raw_id, obj in objects.items()
               if obj.quality.measurement_state == MeasurementState.DELETED_FOR_MERGE and raw_id in self._tracks]
    new = [raw_id for raw_id, obj in objects.items()
           if obj.quality.measurement_state == MeasurementState.NEW_FROM_MERGE and raw_id not in self._tracks]
    for new_raw_id in new:
      candidates = [old_raw_id for old_raw_id in deleted if old_raw_id in self._tracks and
                    _same_target(objects[old_raw_id], objects[new_raw_id], handover=True)]
      if len(candidates) == 1:
        self._transfer(candidates[0], new_raw_id)

  def _kinematic_handover(self, objects: dict[int, AssembledObject]) -> None:
    active_ids = {raw_id for raw_id, obj in objects.items()
                  if obj.quality.measurement_state not in (MeasurementState.DELETED, MeasurementState.DELETED_FOR_MERGE)}
    missing = [raw_id for raw_id in self._tracks if raw_id not in active_ids]
    new = [raw_id for raw_id in active_ids if raw_id not in self._tracks]
    for new_raw_id in new:
      candidates = [old_raw_id for old_raw_id in missing if old_raw_id in self._tracks and
                    _same_target(self._tracks[old_raw_id].obj, objects[new_raw_id])]
      if len(candidates) == 1:
        self._transfer(candidates[0], new_raw_id)
        missing.remove(candidates[0])

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

  def _rank(self, raw_id: int, obj: AssembledObject) -> tuple[bool, int, int, int]:
    state_rank = {MeasurementState.MEASURED: 4, MeasurementState.NEW: 3,
                  MeasurementState.NEW_FROM_MERGE: 3, MeasurementState.PREDICTED: 1}.get(
      obj.quality.measurement_state, 0,
    )
    return raw_id in self._tracks, state_rank, obj.quality.probability, -raw_id

  def _result(self, accepted_raw_ids: tuple[int, ...], rejected: dict[int, RejectionReason]) -> TrackerResult:
    tracks = tuple(self._to_output(track) for track in sorted(self._tracks.values(), key=lambda item: item.track_id))
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
