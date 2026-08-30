from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field, replace
from enum import IntFlag, StrEnum

from opendbc.sunnypilot.car.tesla.ars408.constants import (
  DynamicProperty, LATERAL_KINEMATICS_VERIFIED, MeasurementState, MotionRxState, ObjectClass, RMS_INVALID_CODE,
)
from opendbc.sunnypilot.car.tesla.ars408.models import AssembledObject, CycleResult, ObjectExtended


NORMAL_COAST_NS = 180_000_000
MATURE_VRU_COAST_NS = 250_000_000
TENTATIVE_TIMEOUT_NS = 400_000_000
HANDOVER_MAX_NS = 180_000_000
EXTENDED_STALE_NS = 300_000_000
HISTORY_LENGTH = 8
COUNTER_MODULUS = 1 << 16
COUNTER_HALF_RANGE = COUNTER_MODULUS >> 1
COUNTER_RESTART_LOW_WATERMARK = 64
MAX_FORWARD_GAP_CYCLES = 8
CADENCE_LOCK_DELTAS = 5
# Diagnostic inference only. These timing bounds are an uncalibrated deployment
# assumption, not an ARS408 protocol guarantee and never make cadence valid.
CADENCE_INTERVAL_MIN_NS = 50_000_000
CADENCE_INTERVAL_MAX_NS = 110_000_000
VALID_COUNTER_STEPS = frozenset((1, 2))

MEASURED_STATES = frozenset((MeasurementState.NEW, MeasurementState.MEASURED, MeasurementState.NEW_FROM_MERGE))
DELETED_STATES = frozenset((MeasurementState.DELETED, MeasurementState.DELETED_FOR_MERGE))
STATIC_PROPERTIES = frozenset((DynamicProperty.STATIONARY, DynamicProperty.STATIONARY_CANDIDATE,
                               DynamicProperty.CROSSING_STATIONARY, DynamicProperty.STOPPED))
MOVING_PROPERTIES = frozenset((DynamicProperty.MOVING, DynamicProperty.ONCOMING, DynamicProperty.CROSSING_MOVING))
VRU_CLASSES = frozenset((ObjectClass.PEDESTRIAN, ObjectClass.MOTORCYCLE, ObjectClass.BICYCLE))
VEHICLE_CLASSES = frozenset((ObjectClass.CAR, ObjectClass.TRUCK))


class CycleStatus(StrEnum):
  EXACT = "exact"
  PARTIAL = "partial"
  DUPLICATE = "duplicate"
  INVALID = "invalid"


class CadenceStepSource(StrEnum):
  UNKNOWN = "unknown"
  EXTERNAL = "external"
  DEVICE_CAPTURE = "deviceCapture"
  INFERRED = "inferred"


class Lifecycle(StrEnum):
  TENTATIVE = "tentative"
  CONFIRMED_FRESH = "confirmedFresh"
  COASTING = "coasting"
  CLUTTER_SUSPECT = "clutterSuspect"
  EXPIRED = "expired"


class SemanticGroup(StrEnum):
  VEHICLE = "vehicle"
  PEDESTRIAN = "pedestrian"
  RIDER = "rider"
  UNKNOWN_OBSTACLE = "unknownObstacle"


class ReasonBits(IntFlag):
  NONE = 0
  LOW_PROBABILITY = 1 << 0
  NOT_MEASURED = 1 << 1
  PARTIAL_CYCLE = 1 << 2
  MISSING_EXTENDED = 1 << 3
  MOTION_INPUT_INVALID = 1 << 4
  ID_HANDOVER = 1 << 5
  CLASS_CONFLICT = 1 << 6
  RMS_INVALID = 1 << 7
  KINEMATIC_JUMP = 1 << 8
  STRUCTURAL_CLUTTER = 1 << 9
  COASTING = 1 << 10
  EXPLICIT_DELETE = 1 << 11
  LATERAL_KINEMATICS_UNVERIFIED = 1 << 12
  STANDARD_DUPLICATE_REJECTED = 1 << 13


@dataclass(frozen=True, slots=True)
class PerceptionTarget:
  logical_id: int
  raw_id: int
  lifecycle: Lifecycle
  semantic_group: SemanticGroup
  fresh_measured: bool
  possible_vru: bool
  extended_fresh: bool
  hit_count: int
  miss_count: int
  age_cycles: int
  last_measured_age_ms: float
  raw_d_rel: float
  raw_y_rel: float
  raw_v_rel: float
  raw_yv_rel: float
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  a_rel_long: float
  a_rel_lat: float
  orientation: float
  length: float
  width: float
  rcs: float
  existence_probability_code: int
  measurement_state: int
  dynamic_property: int
  raw_class: int
  stable_class: int
  quality_score: float  # uncalibrated diagnostic heuristic; never a probability/control threshold
  clutter_score: float  # uncalibrated diagnostic flag; never evidence that a region is clear
  rms_codes: tuple[int, ...]
  rms_upper_bounds: tuple[float | None, ...]
  reason_bits: int


@dataclass(frozen=True, slots=True)
class PerceptionSnapshot:
  sequence: int
  source_mono_time: int
  last_component_mono_time: int
  assembly_close_mono_time: int
  measurement_counter: int
  cycle_status: CycleStatus
  motion_input_valid: bool
  general_complete: bool
  quality_complete: bool
  extended_complete: bool
  raw_count: int
  accepted_count: int
  counter_gap: int  # raw uint16 modulo delta for the current anomaly; zero otherwise
  dropped_cycle_count: int
  counter_anomaly_count: int
  cadence_valid: bool
  cadence_verified: bool
  cadence_inferred: bool
  expected_counter_step: int
  step_source: CadenceStepSource
  stable_targets: tuple[PerceptionTarget, ...]
  uncertain_targets: tuple[PerceptionTarget, ...]
  clutter_suspects: tuple[PerceptionTarget, ...]
  expired_events: tuple[PerceptionTarget, ...]


@dataclass(slots=True)
class _Track:
  logical_id: int
  raw_id: int
  obj: AssembledObject
  created_ns: int
  last_seen_ns: int
  last_measured_ns: int
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  extended: ObjectExtended | None
  last_extended_ns: int | None
  hit_history: deque[bool] = field(default_factory=lambda: deque(maxlen=HISTORY_LENGTH))
  probability_history: deque[int] = field(default_factory=lambda: deque(maxlen=HISTORY_LENGTH))
  class_history: deque[tuple[int, int]] = field(default_factory=lambda: deque(maxlen=7))
  hit_count: int = 0
  miss_count: int = 0
  age_cycles: int = 0
  confirmed: bool = False
  stable_class: int = int(ObjectClass.RESERVED)
  possible_vru_hold: int = 0
  fresh_measured: bool = False
  extended_fresh: bool = False
  handed_over: bool = False
  class_conflict: bool = False

  @property
  def possible_vru(self) -> bool:
    return self.possible_vru_hold > 0


class ARS408PerceptionTracker:
  """Auditable shadow tracker. It never mutates the standard RadarData output."""

  def __init__(self, *, expected_counter_step: int | None = None, cadence_verified: bool = False,
               lateral_transform_verified: bool = LATERAL_KINEMATICS_VERIFIED) -> None:
    if expected_counter_step is not None and expected_counter_step not in VALID_COUNTER_STEPS:
      raise ValueError("ARS408 expected counter step must be 1 or 2")
    if cadence_verified and expected_counter_step is None:
      raise ValueError("verified ARS408 cadence requires an expected counter step")
    self._configured_counter_step = expected_counter_step or 0
    self._configured_cadence_verified = cadence_verified and expected_counter_step is not None
    self._configured_step_source = (
      CadenceStepSource.DEVICE_CAPTURE if self._configured_cadence_verified else
      CadenceStepSource.EXTERNAL if expected_counter_step is not None else CadenceStepSource.UNKNOWN
    )
    self._lateral_transform_verified = lateral_transform_verified
    self._tracks: dict[int, _Track] = {}
    # One bounded bit per raw ID remembers first use; active IDs are bounded by the
    # ARS408 protocol maximum. This avoids an unbounded lifetime set.
    self._seen_raw_ids_mask = 0
    self._next_logical_id = 256
    self._last_counter: int | None = None
    self._last_counter_time_ns: int | None = None
    self._cadence_verified = self._configured_cadence_verified
    self._cadence_valid = False
    self._expected_counter_step = self._configured_counter_step
    self._step_source = self._configured_step_source
    self._learning_step = 0
    self._learning_delta_count = 0
    self._restart_last_counter: int | None = None
    self._restart_last_counter_time_ns: int | None = None
    self._restart_previous_step = 0
    self._restart_previous_step_source = CadenceStepSource.UNKNOWN
    self._restart_step = 0
    self._restart_delta_count = 0
    self._counter_gap = 0
    self._dropped_cycle_count = 0
    self._counter_anomaly_count = 0
    self._accepted_count = 0
    self._last_now_ns: int | None = None
    self._sequence = 0
    self.snapshot: PerceptionSnapshot | None = None

  def reset_after_fault(self) -> None:
    """Drop all mutable perception state while preserving reviewed device configuration."""
    self._tracks.clear()
    # Keep the bounded seen-ID mask and logical-ID allocator across faults so a
    # half-built pre-fault target cannot reappear with the same logical identity.
    self._last_counter = None
    self._last_counter_time_ns = None
    self._cadence_verified = self._configured_cadence_verified
    self._cadence_valid = False
    self._expected_counter_step = self._configured_counter_step
    self._step_source = self._configured_step_source
    self._reset_learning()
    self._clear_restart_candidate()
    self._counter_gap = 0
    self._dropped_cycle_count = 0
    self._counter_anomaly_count = 0
    self._accepted_count = 0
    self._last_now_ns = None
    self.snapshot = None

  def update(self, cycle: CycleResult, now_ns: int, *, motion_rx_state: int | None,
             accepted_raw_ids: tuple[int, ...] | None = None,
             duplicate_rejected_raw_ids: tuple[int, ...] = (),
             input_trusted: bool = True) -> PerceptionSnapshot:
    if now_ns < 0 or (self._last_now_ns is not None and now_ns < self._last_now_ns):
      raise ValueError("ARS408 perception time must be monotonic")
    self._last_now_ns = now_ns
    self._sequence += 1
    if not input_trusted:
      return self._reject_untrusted_cycle(cycle, now_ns, motion_rx_state)

    status = self._cycle_status(cycle, now_ns)
    self._accepted_count = 0
    motion_input_valid = motion_rx_state == MotionRxState.INPUT_OK
    expired = self._remove_standard_duplicate_rejections(
      cycle, now_ns, motion_input_valid, status, duplicate_rejected_raw_ids,
    )
    # Expire against the last real measurement before allowing a reused raw ID
    # to update the track. Otherwise a long-dead target can be revived in place.
    expired.extend(self._expire_tracks(now_ns, motion_input_valid, status))

    if status not in (CycleStatus.INVALID, CycleStatus.DUPLICATE):
      expired.extend(self._process_cycle(
        cycle, now_ns, status, motion_input_valid, accepted_raw_ids, duplicate_rejected_raw_ids,
      ))
    else:
      for track in self._tracks.values():
        track.fresh_measured = False
        track.extended_fresh = False

    clutter_ids = self._structural_clutter_ids(motion_input_valid)
    stable: list[PerceptionTarget] = []
    uncertain: list[PerceptionTarget] = []
    clutter: list[PerceptionTarget] = []
    for track in sorted(self._tracks.values(), key=lambda item: item.logical_id):
      target = self._to_target(track, now_ns, motion_input_valid, status, track.raw_id in clutter_ids)
      if target.lifecycle == Lifecycle.CLUTTER_SUSPECT:
        clutter.append(target)
      elif track.confirmed:
        stable.append(target)
      else:
        uncertain.append(target)

    expected = cycle.status.object_count
    no_duplicates = not cycle.duplicate_ids
    self.snapshot = PerceptionSnapshot(
      sequence=self._sequence, source_mono_time=now_ns,
      last_component_mono_time=int(getattr(cycle, "last_component_mono_time_ns", 0)),
      assembly_close_mono_time=int(getattr(cycle, "assembly_close_mono_time_ns", 0)),
      measurement_counter=cycle.status.measurement_counter,
      cycle_status=status, motion_input_valid=motion_input_valid,
      general_complete=status == CycleStatus.EXACT and cycle.status.protocol_valid and no_duplicates and cycle.general_count == expected,
      quality_complete=status == CycleStatus.EXACT and cycle.status.protocol_valid and no_duplicates and cycle.quality_count == expected,
      extended_complete=status == CycleStatus.EXACT and cycle.status.protocol_valid and no_duplicates and cycle.extended_count == expected,
      raw_count=expected, accepted_count=self._accepted_count,
      counter_gap=self._counter_gap, dropped_cycle_count=self._dropped_cycle_count,
      counter_anomaly_count=self._counter_anomaly_count,
      cadence_valid=self._cadence_valid, cadence_verified=self._cadence_verified,
      cadence_inferred=self._step_source == CadenceStepSource.INFERRED,
      expected_counter_step=self._expected_counter_step, step_source=self._step_source,
      stable_targets=tuple(stable), uncertain_targets=tuple(uncertain),
      clutter_suspects=tuple(clutter), expired_events=tuple(expired),
    )
    return self.snapshot

  def _reject_untrusted_cycle(self, cycle: CycleResult, now_ns: int,
                              motion_rx_state: int | None) -> PerceptionSnapshot:
    # A blocked health/config/source frame cannot seed counter continuity or
    # object confirmation. Clear all perception runtime state and publish an
    # explicit empty INVALID snapshot; recovery starts from a fresh baseline.
    self.reset_after_fault()
    self._last_now_ns = now_ns
    self.snapshot = PerceptionSnapshot(
      sequence=self._sequence, source_mono_time=now_ns,
      last_component_mono_time=int(getattr(cycle, "last_component_mono_time_ns", 0)),
      assembly_close_mono_time=int(getattr(cycle, "assembly_close_mono_time_ns", 0)),
      measurement_counter=cycle.status.measurement_counter,
      cycle_status=CycleStatus.INVALID,
      motion_input_valid=motion_rx_state == MotionRxState.INPUT_OK,
      general_complete=False, quality_complete=False, extended_complete=False,
      raw_count=cycle.status.object_count, accepted_count=0,
      counter_gap=0, dropped_cycle_count=0, counter_anomaly_count=0,
      cadence_valid=False, cadence_verified=self._cadence_verified,
      cadence_inferred=False, expected_counter_step=self._expected_counter_step,
      step_source=self._step_source,
      stable_targets=(), uncertain_targets=(), clutter_suspects=(), expired_events=(),
    )
    return self.snapshot

  def _cycle_status(self, cycle: CycleResult, now_ns: int) -> CycleStatus:
    self._counter_gap = 0
    if cycle.invalid or not cycle.timestamp_valid:
      self._cadence_valid = False
      return CycleStatus.INVALID
    counter = cycle.status.measurement_counter
    base_status = CycleStatus.EXACT if cycle.exact else CycleStatus.PARTIAL
    if self._last_counter is None:
      self._last_counter = counter
      self._last_counter_time_ns = now_ns
      self._cadence_valid = False
      return CycleStatus.INVALID

    delta = (counter - self._last_counter) & 0xFFFF
    if self._restart_last_counter is not None:
      restart_status = self._continue_restart_candidate(counter, delta, base_status, now_ns)
      if restart_status is not None:
        return restart_status

    if delta == 0:
      self._cadence_valid = False
      return CycleStatus.DUPLICATE

    if not self._cadence_verified:
      if self._step_source == CadenceStepSource.EXTERNAL:
        # An explicit but unverified value is diagnostic configuration only.
        # It never permits a consumable cycle or dropped-cycle inference.
        self._last_counter = counter
        self._last_counter_time_ns = now_ns
        if delta >= COUNTER_HALF_RANGE or delta != self._expected_counter_step:
          self._record_counter_anomaly(delta)
        return CycleStatus.INVALID
      if delta >= COUNTER_HALF_RANGE:
        # Without a lock, fail safe by rebasing and relearning. Do not infer a
        # dropped-cycle count from a backward modulo delta.
        self._last_counter = counter
        self._last_counter_time_ns = now_ns
        self._expected_counter_step = 0
        self._step_source = CadenceStepSource.UNKNOWN
        self._reset_learning()
        self._record_counter_anomaly(delta)
        return CycleStatus.INVALID
      interval_ns = now_ns - int(self._last_counter_time_ns)
      self._last_counter = counter
      self._last_counter_time_ns = now_ns
      if self._step_source == CadenceStepSource.INFERRED:
        if delta == self._expected_counter_step and self._cadence_interval_valid(interval_ns):
          return CycleStatus.INVALID
        self._expected_counter_step = 0
        self._step_source = CadenceStepSource.UNKNOWN
        self._reset_learning()
        self._record_counter_anomaly(delta)
        return CycleStatus.INVALID
      if delta not in VALID_COUNTER_STEPS or not self._cadence_interval_valid(interval_ns):
        self._reset_learning()
        self._record_counter_anomaly(delta)
        return CycleStatus.INVALID
      if delta == self._learning_step:
        self._learning_delta_count += 1
      else:
        self._learning_step = delta
        self._learning_delta_count = 1
      if self._learning_delta_count < CADENCE_LOCK_DELTAS:
        return CycleStatus.INVALID
      self._expected_counter_step = delta
      self._step_source = CadenceStepSource.INFERRED
      self._reset_learning()
      return CycleStatus.INVALID

    if delta == self._expected_counter_step:
      self._last_counter = counter
      self._last_counter_time_ns = now_ns
      self._cadence_valid = True
      return base_status

    self._cadence_valid = False
    interval_ns = now_ns - int(self._last_counter_time_ns)
    restart_or_ambiguous = self._counter_jump_requires_resync(counter, delta, interval_ns)
    self._record_counter_anomaly(
      delta, expected_step=0 if restart_or_ambiguous else self._expected_counter_step,
    )
    if restart_or_ambiguous:
      # Backward, reset-to-low, and implausibly large forward jumps are
      # ambiguous. Keep the device-capture identity, but never guess dropped
      # cycles; a normal old-stream counter proves an isolated out-of-order
      # frame, otherwise five timed deltas are required to resynchronize.
      self._begin_restart_candidate(counter, now_ns)
      return CycleStatus.INVALID

    # A forward discontinuity is consumed so that the next expected delta can
    # recover, but this cycle never refreshes perception state.
    self._last_counter = counter
    self._last_counter_time_ns = now_ns
    return CycleStatus.INVALID

  def _continue_restart_candidate(self, counter: int, accepted_delta: int,
                                  base_status: CycleStatus, now_ns: int) -> CycleStatus | None:
    if accepted_delta == self._restart_previous_step:
      # The low frame was out-of-order, not a restart. Restore the old lock.
      self._expected_counter_step = self._restart_previous_step
      self._cadence_valid = self._cadence_verified
      self._step_source = self._restart_previous_step_source
      self._last_counter = counter
      self._last_counter_time_ns = now_ns
      self._clear_restart_candidate()
      return base_status
    if accepted_delta == 0:
      self._expected_counter_step = self._restart_previous_step
      self._cadence_valid = False
      self._step_source = self._restart_previous_step_source
      self._clear_restart_candidate()
      return CycleStatus.DUPLICATE
    if counter == self._restart_last_counter:
      self._cadence_valid = False
      return CycleStatus.DUPLICATE

    restart_delta = (counter - self._restart_last_counter) & 0xFFFF
    restart_interval_ns = now_ns - int(self._restart_last_counter_time_ns)
    self._restart_last_counter = counter
    self._restart_last_counter_time_ns = now_ns
    restart_step_allowed = restart_delta == self._restart_previous_step if self._cadence_verified else \
      restart_delta in VALID_COUNTER_STEPS
    if restart_step_allowed and self._cadence_interval_valid(restart_interval_ns):
      if restart_delta == self._restart_step:
        self._restart_delta_count += 1
      else:
        self._restart_step = restart_delta
        self._restart_delta_count = 1
      if self._restart_delta_count >= CADENCE_LOCK_DELTAS:
        self._expected_counter_step = restart_delta
        self._cadence_valid = self._cadence_verified
        self._step_source = (
          self._restart_previous_step_source if self._cadence_verified else CadenceStepSource.INFERRED
        )
        self._last_counter = counter
        self._last_counter_time_ns = now_ns
        self._counter_gap = 0
        self._clear_restart_candidate()
        return base_status
    else:
      self._record_counter_anomaly(restart_delta)
      self._restart_step = 0
      self._restart_delta_count = 0
    return CycleStatus.INVALID

  def _begin_restart_candidate(self, counter: int, now_ns: int) -> None:
    self._restart_last_counter = counter
    self._restart_last_counter_time_ns = now_ns
    self._restart_previous_step = self._expected_counter_step
    self._restart_previous_step_source = self._step_source
    self._cadence_valid = False
    self._restart_step = 0
    self._restart_delta_count = 0

  def _clear_restart_candidate(self) -> None:
    self._restart_last_counter = None
    self._restart_last_counter_time_ns = None
    self._restart_previous_step = 0
    self._restart_previous_step_source = CadenceStepSource.UNKNOWN
    self._restart_step = 0
    self._restart_delta_count = 0

  def _reset_learning(self) -> None:
    self._learning_step = 0
    self._learning_delta_count = 0

  @staticmethod
  def _cadence_interval_valid(interval_ns: int) -> bool:
    return CADENCE_INTERVAL_MIN_NS <= interval_ns <= CADENCE_INTERVAL_MAX_NS

  def _counter_jump_requires_resync(self, counter: int, delta: int, interval_ns: int) -> bool:
    if delta >= COUNTER_HALF_RANGE:
      return True
    # An exact verified delta was accepted before this helper. Any other jump
    # from the high half into the low reset region is therefore not treated as
    # a lossy modulo wrap.
    if self._last_counter is not None and self._last_counter >= COUNTER_HALF_RANGE and \
        counter <= COUNTER_RESTART_LOW_WATERMARK:
      return True
    # Small integral gaps retain the existing diagnostic dropped-count
    # behavior. Beyond this conservative, non-protocol bound, use arrival time
    # only to widen the allowance after a long silence and require resync.
    elapsed_slots = max(1, math.ceil(max(0, interval_ns) / CADENCE_INTERVAL_MIN_NS))
    plausible_cycles = max(MAX_FORWARD_GAP_CYCLES, elapsed_slots)
    return delta > self._expected_counter_step * plausible_cycles

  def _record_counter_anomaly(self, delta: int, *, expected_step: int = 0) -> None:
    # counter_gap is deliberately the raw uint16 modulo delta. Only a locked,
    # clearly forward, integral cadence gap supports estimating missed cycles.
    self._counter_gap = delta
    self._counter_anomaly_count += 1
    if expected_step and delta < COUNTER_HALF_RANGE and delta % expected_step == 0:
      self._dropped_cycle_count += max(0, delta // expected_step - 1)

  def _process_cycle(self, cycle: CycleResult, now_ns: int, status: CycleStatus, motion_input_valid: bool,
                     accepted_raw_ids: tuple[int, ...] | None,
                     duplicate_rejected_raw_ids: tuple[int, ...]) -> list[PerceptionTarget]:
    expired: list[PerceptionTarget] = []
    for track in self._tracks.values():
      track.handed_over = False
      track.fresh_measured = False
      track.extended_fresh = False
    # A partial cycle is diagnostic-only. It cannot promote, refresh, delete,
    # create, or move a target used by the stable channel.
    if status != CycleStatus.EXACT:
      return expired

    objects = {obj.raw_id: obj for obj in cycle.objects}
    duplicate_rejected = set(duplicate_rejected_raw_ids)
    for raw_id, obj in objects.items():
      if obj.quality.measurement_state == MeasurementState.DELETED and raw_id in self._tracks:
        track = self._tracks[raw_id]
        track.obj = obj
        track.last_seen_ns = now_ns
        expired.append(self._remove_track(raw_id, now_ns, ReasonBits.EXPLICIT_DELETE, motion_input_valid, status))
    if accepted_raw_ids is not None:
      accepted = set(accepted_raw_ids)
    else:
      accepted = set()
      for raw_id, obj in objects.items():
        if obj.quality.measurement_state in DELETED_STATES:
          continue
        established = raw_id in self._tracks
        minimum_probability = 2 if established else 3
        state = obj.quality.measurement_state
        if obj.quality.probability >= minimum_probability and (
          state in MEASURED_STATES or (state == MeasurementState.PREDICTED and established)
        ):
          accepted.add(raw_id)
    accepted.difference_update(duplicate_rejected)
    active = {raw_id: obj for raw_id, obj in objects.items()
              if raw_id in accepted and obj.quality.measurement_state not in DELETED_STATES}
    self._accepted_count = len(active)
    observed_ids: set[int] = set()
    rejected_observed_ids: set[int] = set()

    # Rejected measurements never refresh kinematics or last-measured time, but
    # repeated low-existence evidence is allowed to demote a confirmed target.
    for raw_id, obj in objects.items():
      if raw_id in accepted or raw_id not in self._tracks or not self._is_fresh_measurement(obj):
        continue
      track = self._tracks[raw_id]
      track.obj = obj
      track.last_seen_ns = now_ns
      track.hit_history.append(False)
      track.probability_history.append(obj.quality.probability)
      track.miss_count += 1
      track.age_cycles += 1
      if track.confirmed and len(track.probability_history) >= 2 and \
         all(code <= 1 for code in list(track.probability_history)[-2:]):
        track.confirmed = False
      rejected_observed_ids.add(raw_id)

    new_objects: list[AssembledObject] = []
    for raw_id, obj in active.items():
      track = self._tracks.get(raw_id)
      if track is None:
        new_objects.append(obj)
        continue
      if self._is_fresh_measurement(obj) and not self._kinematic_gate(track, obj, now_ns):
        expired.append(self._remove_track(raw_id, now_ns, ReasonBits.KINEMATIC_JUMP, motion_input_valid, status))
        new_objects.append(obj)
        continue
      self._update_track(track, obj, now_ns, status)
      observed_ids.add(raw_id)

    handovers = self._handover_matches(new_objects, active, now_ns)
    for obj in new_objects:
      if not self._is_fresh_measurement(obj):
        continue
      track = handovers.get(obj.raw_id)
      if track is None:
        track = self._new_track(obj, now_ns)
      else:
        del self._tracks[track.raw_id]
        track.raw_id = obj.raw_id
        track.handed_over = True
        self._tracks[obj.raw_id] = track
        self._update_track(track, obj, now_ns, status)
      observed_ids.add(obj.raw_id)

    for raw_id, track in self._tracks.items():
      if raw_id not in observed_ids and raw_id not in rejected_observed_ids:
        track.hit_history.append(False)
        track.miss_count += 1
        track.age_cycles += 1
    return expired

  def _remove_standard_duplicate_rejections(
    self, cycle: CycleResult, now_ns: int, motion_input_valid: bool, status: CycleStatus,
    duplicate_rejected_raw_ids: tuple[int, ...],
  ) -> list[PerceptionTarget]:
    expired: list[PerceptionTarget] = []
    objects = {obj.raw_id: obj for obj in cycle.objects}
    rejected = set(duplicate_rejected_raw_ids)
    for raw_id in sorted(rejected & self._tracks.keys()):
      track = self._tracks[raw_id]
      if raw_id in objects:
        # Preserve the current rejected measurement state in the one-cycle
        # expiry diagnostic without treating it as a fresh observation.
        track.obj = objects[raw_id]
        track.last_seen_ns = now_ns
      expired.append(self._remove_track(
        raw_id, now_ns, ReasonBits.STANDARD_DUPLICATE_REJECTED, motion_input_valid, status,
      ))
    return expired

  def _new_track(self, obj: AssembledObject, now_ns: int) -> _Track:
    logical_id = self._allocate_logical_id(obj.raw_id)
    measured = self._is_fresh_measurement(obj)
    track = _Track(
      logical_id=logical_id, raw_id=obj.raw_id, obj=obj, created_ns=now_ns, last_seen_ns=now_ns,
      last_measured_ns=now_ns if measured else now_ns, d_rel=obj.general.d_rel, y_rel=obj.general.y_rel,
      v_rel=obj.general.v_rel, yv_rel=obj.general.yv_rel, extended=obj.extended,
      last_extended_ns=now_ns if obj.extended is not None else None,
    )
    self._tracks[obj.raw_id] = track
    self._update_history(track, obj)
    track.age_cycles = 1
    track.fresh_measured = measured
    track.extended_fresh = obj.extended is not None
    track.confirmed = self._confirmation_ready(track)
    return track

  def _update_track(self, track: _Track, obj: AssembledObject, now_ns: int, status: CycleStatus) -> None:
    measured = self._is_fresh_measurement(obj)
    track.obj = obj
    track.last_seen_ns = now_ns
    track.age_cycles += 1
    track.fresh_measured = measured
    track.extended_fresh = measured and obj.extended is not None
    if measured:
      if obj.extended is not None:
        track.extended = obj.extended
        track.last_extended_ns = now_ns
      elif track.last_extended_ns is not None and now_ns - track.last_extended_ns > EXTENDED_STALE_NS:
        # Stale class/shape data is never kept as a fresh semantic claim. The
        # target remains an obstacle, but its semantic group becomes unknown.
        track.extended = None
        track.class_history.clear()
        track.stable_class = int(ObjectClass.RESERVED)
        track.possible_vru_hold = 0
        track.class_conflict = False
      track.last_measured_ns = now_ns
      track.miss_count = 0
      self._filter_measurement(track, obj)
      self._update_history(track, obj)
    else:
      track.miss_count += 1
    if status == CycleStatus.EXACT and not track.confirmed:
      track.confirmed = self._confirmation_ready(track)
    elif status == CycleStatus.EXACT and measured and track.confirmed and \
         len(track.probability_history) >= 2 and all(code <= 1 for code in list(track.probability_history)[-2:]):
      track.confirmed = False

  @staticmethod
  def _is_fresh_measurement(obj: AssembledObject) -> bool:
    return obj.quality.measurement_state in MEASURED_STATES

  def _update_history(self, track: _Track, obj: AssembledObject) -> None:
    track.hit_history.append(True)
    track.probability_history.append(obj.quality.probability)
    track.hit_count += 1
    if obj.extended is not None:
      weight = max(1, obj.quality.probability)
      object_class = obj.extended.object_class
      track.class_history.append((object_class, weight))
      if object_class in VRU_CLASSES:
        track.possible_vru_hold = 7
      elif track.possible_vru_hold > 0:
        track.possible_vru_hold -= 1
    elif track.possible_vru_hold > 0:
      track.possible_vru_hold -= 1
    self._update_stable_class(track)

  @staticmethod
  def _update_stable_class(track: _Track) -> None:
    weights = {int(object_class): 0 for object_class in ObjectClass}
    for object_class, weight in track.class_history:
      weights[object_class] += weight
    vehicle_weight = sum(weights[int(item)] for item in VEHICLE_CLASSES)
    vru_weight = sum(weights[int(item)] for item in VRU_CLASSES)
    track.class_conflict = vehicle_weight > 0 and vru_weight > 0 and max(vehicle_weight, vru_weight) <= 2 * min(vehicle_weight, vru_weight)
    if track.class_conflict or not track.class_history:
      track.stable_class = int(ObjectClass.RESERVED)
      return
    track.stable_class = max(weights, key=weights.get)

  @staticmethod
  def _semantic_group(track: _Track) -> SemanticGroup:
    if track.class_conflict:
      return SemanticGroup.UNKNOWN_OBSTACLE
    if track.stable_class in VEHICLE_CLASSES:
      return SemanticGroup.VEHICLE
    if track.stable_class == ObjectClass.PEDESTRIAN:
      return SemanticGroup.PEDESTRIAN
    if track.stable_class in (ObjectClass.MOTORCYCLE, ObjectClass.BICYCLE):
      return SemanticGroup.RIDER
    return SemanticGroup.UNKNOWN_OBSTACLE

  def _confirmation_ready(self, track: _Track) -> bool:
    group = self._semantic_group(track)
    hits = list(track.hit_history)
    probabilities = list(track.probability_history)
    if not probabilities or probabilities[-1] <= 1:
      return False
    if group == SemanticGroup.VEHICLE:
      return sum(hits[-4:]) >= 3 and sum(code >= 3 for code in probabilities[-4:]) >= 2
    if group in (SemanticGroup.PEDESTRIAN, SemanticGroup.RIDER) or track.possible_vru:
      if probabilities[-1] >= 3:
        return sum(hits[-3:]) >= 2
      return probabilities[-1] == 2 and sum(hits[-5:]) >= 4
    return sum(hits[-5:]) >= 3

  @staticmethod
  def _filter_measurement(track: _Track, obj: AssembledObject) -> None:
    upper_bounds = ARS408PerceptionTracker._kinematic_upper_bounds(obj)

    def alpha(sigma: float | None) -> float:
      if sigma is None:
        return 0.45
      return min(0.80, max(0.25, 0.75 / (1.0 + sigma)))

    values = (obj.general.d_rel, obj.general.y_rel, obj.general.v_rel)
    previous = (track.d_rel, track.y_rel, track.v_rel)
    filtered = tuple(old + alpha(bound) * (new - old)
                     for old, new, bound in zip(previous, values, upper_bounds, strict=True))
    track.d_rel, track.y_rel, track.v_rel = filtered
    # Lateral kinematics are retained as unverified diagnostics only. They do
    # not participate in filtering/association until installation evidence
    # establishes their sign and timing.
    track.yv_rel = obj.general.yv_rel

  @staticmethod
  def _kinematic_gate(track: _Track, obj: AssembledObject, now_ns: int) -> bool:
    dt = max(0.0, min((now_ns - track.last_measured_ns) / 1e9, 0.5))
    predicted = (track.d_rel + track.v_rel * dt, track.y_rel, track.v_rel)
    observed = (obj.general.d_rel, obj.general.y_rel, obj.general.v_rel)
    defaults = (2.5, 1.0, 2.5)
    caps = (6.0, 2.5, 6.0)
    upper_bounds = ARS408PerceptionTracker._kinematic_upper_bounds(obj)
    limits = tuple(min(cap, max(default, 2.0 * bound if bound is not None else default))
                   for default, cap, bound in zip(defaults, caps, upper_bounds, strict=True))
    return all(abs(value - estimate) <= limit for value, estimate, limit in zip(observed, predicted, limits, strict=True))

  @staticmethod
  def _kinematic_upper_bounds(obj: AssembledObject) -> tuple[float | None, ...]:
    # Protocol values are quantization-bin upper bounds, not measured sigma.
    bounds = getattr(obj.quality, "rms_upper_bounds", obj.quality.rms_sigmas)
    return bounds[0], bounds[2], bounds[1]

  def _handover_matches(self, new_objects: list[AssembledObject], active: dict[int, AssembledObject],
                        now_ns: int) -> dict[int, _Track]:
    by_new: dict[int, list[_Track]] = {}
    reverse_count: dict[int, int] = {}
    for obj in new_objects:
      candidates = [track for track in self._tracks.values() if (
        track.raw_id not in active and now_ns - track.last_measured_ns <= HANDOVER_MAX_NS and
        self._class_compatible(track, obj) and self._kinematic_gate(track, obj, now_ns)
      )]
      by_new[obj.raw_id] = candidates
      for track in candidates:
        reverse_count[track.logical_id] = reverse_count.get(track.logical_id, 0) + 1
    return {
      raw_id: candidates[0] for raw_id, candidates in by_new.items()
      if len(candidates) == 1 and reverse_count[candidates[0].logical_id] == 1
    }

  @staticmethod
  def _class_compatible(track: _Track, obj: AssembledObject) -> bool:
    if obj.extended is None or track.stable_class in (ObjectClass.POINT, ObjectClass.RESERVED):
      return True
    new_class = obj.extended.object_class
    if new_class in (ObjectClass.POINT, ObjectClass.RESERVED):
      return True
    old_group = ARS408PerceptionTracker._semantic_group(track)
    new_group = (SemanticGroup.VEHICLE if new_class in VEHICLE_CLASSES else
                 SemanticGroup.PEDESTRIAN if new_class == ObjectClass.PEDESTRIAN else
                 SemanticGroup.RIDER if new_class in (ObjectClass.MOTORCYCLE, ObjectClass.BICYCLE) else
                 SemanticGroup.UNKNOWN_OBSTACLE)
    return old_group == SemanticGroup.UNKNOWN_OBSTACLE or old_group == new_group

  def _expire_tracks(self, now_ns: int, motion_input_valid: bool, status: CycleStatus) -> list[PerceptionTarget]:
    expired: list[PerceptionTarget] = []
    for raw_id, track in list(self._tracks.items()):
      if track.confirmed:
        mature_vru = track.possible_vru and track.hit_count >= 5
        timeout_ns = MATURE_VRU_COAST_NS if mature_vru else NORMAL_COAST_NS
        stale = now_ns - track.last_measured_ns > timeout_ns
      else:
        stale = now_ns - track.last_measured_ns > TENTATIVE_TIMEOUT_NS
      if stale:
        expired.append(self._remove_track(raw_id, now_ns, ReasonBits.COASTING, motion_input_valid, status))
    return expired

  def _remove_track(self, raw_id: int, now_ns: int, reason: ReasonBits,
                    motion_input_valid: bool = False, status: CycleStatus = CycleStatus.EXACT) -> PerceptionTarget:
    track = self._tracks.pop(raw_id)
    track.fresh_measured = False
    track.extended_fresh = False
    track.handed_over = False
    target = self._to_target(track, now_ns, motion_input_valid, status, False, extra_reasons=reason)
    return replace(target, lifecycle=Lifecycle.EXPIRED, fresh_measured=False)

  def _allocate_logical_id(self, raw_id: int) -> int:
    raw_bit = 1 << raw_id
    if not self._seen_raw_ids_mask & raw_bit:
      logical_id = raw_id
      self._seen_raw_ids_mask |= raw_bit
    else:
      active_ids = {track.logical_id for track in self._tracks.values()}
      for _ in range(257):
        logical_id = self._next_logical_id
        self._next_logical_id = 256 if logical_id >= 0xFFFFFFFF else logical_id + 1
        if logical_id not in active_ids:
          break
      else:
        raise RuntimeError("no bounded ARS408 logical ID available")
    return logical_id

  def _structural_clutter_ids(self, motion_input_valid: bool) -> set[int]:
    if not motion_input_valid:
      return set()
    candidates = [track for track in self._tracks.values() if (
      track.confirmed and track.fresh_measured and not track.possible_vru and
      track.stable_class in (ObjectClass.POINT, ObjectClass.WIDE, ObjectClass.RESERVED) and
      track.obj.general.dynamic_property in STATIC_PROPERTIES and abs(track.y_rel) > 3.5
    )]
    clutter: set[int] = set()
    for track in candidates:
      band = [other for other in candidates if math.copysign(1.0, other.y_rel) == math.copysign(1.0, track.y_rel) and
              abs(other.y_rel - track.y_rel) <= 1.0]
      if len(band) >= 3 and max(item.y_rel for item in band) - min(item.y_rel for item in band) <= 1.0 and \
         max(item.d_rel for item in band) - min(item.d_rel for item in band) >= 15.0:
        clutter.update(item.raw_id for item in band)
    return clutter

  def _to_target(self, track: _Track, now_ns: int, motion_input_valid: bool, status: CycleStatus,
                 structural_clutter: bool, *, extra_reasons: ReasonBits = ReasonBits.NONE) -> PerceptionTarget:
    reasons = extra_reasons
    quality = track.obj.quality
    if quality.probability <= 1:
      reasons |= ReasonBits.LOW_PROBABILITY
    if not track.fresh_measured:
      reasons |= ReasonBits.NOT_MEASURED | ReasonBits.COASTING
    if status == CycleStatus.PARTIAL:
      reasons |= ReasonBits.PARTIAL_CYCLE
    if not track.extended_fresh:
      reasons |= ReasonBits.MISSING_EXTENDED
    if not motion_input_valid:
      reasons |= ReasonBits.MOTION_INPUT_INVALID
    if track.handed_over:
      reasons |= ReasonBits.ID_HANDOVER
    if track.class_conflict:
      reasons |= ReasonBits.CLASS_CONFLICT
    if any(code == RMS_INVALID_CODE for code in quality.rms_codes):
      reasons |= ReasonBits.RMS_INVALID
    if structural_clutter:
      reasons |= ReasonBits.STRUCTURAL_CLUTTER
    if not self._lateral_transform_verified:
      reasons |= ReasonBits.LATERAL_KINEMATICS_UNVERIFIED

    age_ms = max(0.0, (now_ns - track.last_measured_ns) / 1e6)
    lifecycle = (Lifecycle.CLUTTER_SUSPECT if structural_clutter else
                 Lifecycle.CONFIRMED_FRESH if track.confirmed and track.fresh_measured else
                 Lifecycle.COASTING if track.confirmed else Lifecycle.TENTATIVE)
    extended = track.extended
    raw_class = int(extended.object_class) if extended is not None else int(ObjectClass.RESERVED)
    measured_score = 1.0 if track.fresh_measured else 0.25 if quality.measurement_state == MeasurementState.PREDICTED else 0.0
    valid_rms = sum(code != RMS_INVALID_CODE for code in quality.rms_codes[:4]) / 4.0
    # This bounded number is only for offline ranking/debugging. It is not a
    # calibrated probability and must not be consumed by vehicle control.
    quality_score = min(1.0, max(0.0, 0.55 * quality.probability / 7.0 + 0.30 * measured_score + 0.15 * valid_rms))
    return PerceptionTarget(
      logical_id=track.logical_id, raw_id=track.raw_id, lifecycle=lifecycle,
      semantic_group=self._semantic_group(track), fresh_measured=track.fresh_measured,
      possible_vru=track.possible_vru, extended_fresh=track.extended_fresh,
      hit_count=track.hit_count, miss_count=track.miss_count, age_cycles=track.age_cycles,
      last_measured_age_ms=age_ms, raw_d_rel=track.obj.general.d_rel, raw_y_rel=track.obj.general.y_rel,
      raw_v_rel=track.obj.general.v_rel, raw_yv_rel=track.obj.general.yv_rel,
      d_rel=track.d_rel, y_rel=track.y_rel, v_rel=track.v_rel, yv_rel=track.yv_rel,
      a_rel_long=extended.a_rel if extended is not None else 0.0,
      a_rel_lat=extended.a_rel_lat if extended is not None else 0.0,
      orientation=extended.orientation_angle if extended is not None else 0.0,
      length=extended.length if extended is not None else 0.0, width=extended.width if extended is not None else 0.0,
      rcs=track.obj.general.rcs, existence_probability_code=quality.probability,
      measurement_state=quality.measurement_state, dynamic_property=track.obj.general.dynamic_property,
      raw_class=raw_class, stable_class=track.stable_class, quality_score=quality_score,
      clutter_score=1.0 if structural_clutter else 0.0, rms_codes=quality.rms_codes,
      rms_upper_bounds=quality.rms_upper_bounds, reason_bits=int(reasons),
    )
