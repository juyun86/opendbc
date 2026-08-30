import secrets
import time
from collections.abc import Callable
from dataclasses import dataclass

from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.tesla.ars408.constants import (
  ARS408_BUS, ARS408_EXTENDED, ARS408_GENERAL, ARS408_MAX_DISTANCE_M, ARS408_QUALITY,
  ARS408_RADAR_STATE, ARS408_SENSOR_ID, ARS408_STATUS, LATERAL_KINEMATICS_VERIFIED,
)
from opendbc.sunnypilot.car.tesla.ars408.cycle import ARS408CycleAssembler
from opendbc.sunnypilot.car.tesla.ars408.diagnostics import ARS408Diagnostics, RADAR_STATE_TIMEOUT_NS
from opendbc.sunnypilot.car.tesla.ars408.models import (
  DiagnosticsSnapshot, FilterStateHeader, FilterStateRecord, ObjectExtended, ObjectGeneral, ObjectQuality,
  ObjectStatus, RadarStateSnapshot,
)
from opendbc.sunnypilot.car.tesla.ars408.parser import ARS408Parser, MESSAGE_NAMES
from opendbc.sunnypilot.car.tesla.ars408.stabilizer import ARS408PerceptionTracker, CycleStatus, PerceptionSnapshot
from opendbc.sunnypilot.car.tesla.ars408.tracker import ARS408Tracker


SHADOW_TRANSIENT_SOURCE_FAILED = 1 << 0
SHADOW_TRANSIENT_RADAR_FAILED = 1 << 1
SHADOW_TRANSIENT_CONFIG_FAILED = 1 << 2
SHADOW_TRANSIENT_INTERFERENCE_FAILED = 1 << 3
SHADOW_TRANSIENT_MOTION_FAILED = 1 << 4
SHADOW_TRANSIENT_PARSER_FAILED = 1 << 5
SHADOW_TRANSIENT_PRODUCER_FAILED = 1 << 6
SHADOW_TRANSIENT_COUNTER_FAILED = 1 << 7
LIVE_SOURCE_LAG_NS = 250_000_000
PARSER_REQUIRED_FRESH_NS = 500_000_000
REQUIRED_FRAME_ADDRESSES = frozenset((
  ARS408_RADAR_STATE, ARS408_STATUS, ARS408_GENERAL, ARS408_QUALITY, ARS408_EXTENDED,
))


@dataclass(frozen=True, slots=True)
class ARS408PerceptionState:
  producer_epoch: int
  snapshot: PerceptionSnapshot | None
  diagnostics: DiagnosticsSnapshot
  parser_valid: bool
  motion_input_valid: bool
  radar_state_ready: bool
  radar_state_fresh: bool
  shadow_now_source_ns: int
  lateral_transform_valid: bool
  interference_active: bool
  config_observed: bool
  config_valid: bool
  transient_failure_bits: int
  producer_fault: bool
  producer_fault_count: int


def _nonzero_epoch() -> int:
  while True:
    epoch = secrets.randbits(64)
    if epoch != 0:
      return epoch


class ARS408ShadowPipeline:
  """Raw-CAN-only shadow pipeline with a sticky per-drain fail-closed boundary."""

  def __init__(self, *, clock: Callable[[], int] = time.monotonic_ns,
               epoch_factory: Callable[[], int] = _nonzero_epoch,
               expected_counter_step: int | None = None, cadence_verified: bool = False,
               source_clock_mode: str) -> None:
    if source_clock_mode not in ("live", "source"):
      raise ValueError("ARS408 source clock mode must be live or source")
    self._clock = clock
    self._epoch_factory = epoch_factory
    self._expected_counter_step = expected_counter_step
    self._cadence_verified = cadence_verified
    self._source_clock_mode = source_clock_mode
    self.producer_fault_count = 0
    self.producer_epoch = 0
    self._reset_runtime(self._clock(), new_epoch=True)

  def _reset_runtime(self, host_now_ns: int, *, new_epoch: bool) -> None:
    if new_epoch:
      epoch = int(self._epoch_factory())
      if not 0 < epoch < 1 << 64:
        raise ValueError("ARS408 producer epoch must be a non-zero UInt64")
      self.producer_epoch = epoch
    self.parser = ARS408Parser()
    self.assembler = ARS408CycleAssembler()
    self.standard_tracker = ARS408Tracker()
    self.perception_tracker = ARS408PerceptionTracker(
      expected_counter_step=self._expected_counter_step,
      cadence_verified=self._cadence_verified,
      lateral_transform_verified=LATERAL_KINEMATICS_VERIFIED,
    )
    self.diagnostics = ARS408Diagnostics(0 if self._source_clock_mode == "source" else host_now_ns)
    self.snapshot: PerceptionSnapshot | None = None
    self._source_anchor_ns: int | None = None
    self._source_anchor_host_ns: int | None = None
    self._source_clock_valid = False
    self._parser_ready = False
    self._required_frame_seen = False
    self._last_required_frame_source_ns: int | None = None
    self._last_status_source_ns: int | None = None
    self._last_status_counter: int | None = None
    self._trusted_baseline_established = False
    self._recovery_needs_baseline = True
    self._producer_fault = False
    self._latched_failure_bits = 0

  def hard_reset(self, failure_bits: int, *, host_now_ns: int | None = None) -> ARS408PerceptionState:
    now_ns = self._clock() if host_now_ns is None else int(host_now_ns)
    previous_failure_bits = int(getattr(self, "_latched_failure_bits", 0))
    self.producer_fault_count += 1
    self._reset_runtime(now_ns, new_epoch=True)
    self._producer_fault = True
    self._latched_failure_bits = previous_failure_bits | int(failure_bits) | SHADOW_TRANSIENT_PRODUCER_FAILED
    return self.state(now_ns)

  def process_batch(self, can_packets: list[tuple[int, list[CanData]]], *, host_now_ns: int | None = None) -> ARS408PerceptionState:
    now_ns = self._clock() if host_now_ns is None else int(host_now_ns)
    failure_bits = 0
    completed_cycles: list[tuple[object, object]] = []
    baseline_cycle_collected = False
    tail_baseline_cycle = None
    parser_ready_at_start = self._parser_ready

    source_times = [int(timestamp) for timestamp, _ in can_packets]
    if (any(timestamp <= 0 for timestamp in source_times) or
        any(current <= previous for previous, current in zip(source_times, source_times[1:], strict=False))):
      return self.hard_reset(SHADOW_TRANSIENT_SOURCE_FAILED, host_now_ns=now_ns)
    if self._trusted_baseline_established and source_times:
      pre_batch_failure_bits = self._pre_batch_gap_failure_bits(source_times[0], now_ns)
      if pre_batch_failure_bits:
        return self.hard_reset(pre_batch_failure_bits, host_now_ns=now_ns)

    try:
      for timestamp, frames in can_packets:
        source_time_ns = int(timestamp)
        # Re-check old anchors before this packet can refresh either one. This
        # prevents a fresh required frame or RadarState later in the same drain
        # from laundering an already-expired trusted epoch.
        if self._trusted_baseline_established:
          packet_gap_bits = self._pre_batch_gap_failure_bits(source_time_ns, now_ns)
          if packet_gap_bits:
            failure_bits |= packet_gap_bits
            break
        if not self._update_source_anchor(source_time_ns, now_ns):
          failure_bits |= SHADOW_TRANSIENT_SOURCE_FAILED
          break
        for frame in frames:
          address, _data, bus = frame
          relevant = bus == ARS408_BUS and address in REQUIRED_FRAME_ADDRESSES
          if not (bus == ARS408_BUS and address in MESSAGE_NAMES):
            continue
          parsed = self.parser.parse(source_time_ns, frame)
          if relevant and parsed is None:
            failure_bits |= SHADOW_TRANSIENT_PARSER_FAILED
            break
          if parsed is None:
            continue
          if relevant:
            self._required_frame_seen = True
            self._last_required_frame_source_ns = source_time_ns
          if isinstance(parsed, RadarStateSnapshot):
            self.diagnostics.observe_radar_state(
              parsed, self._diagnostic_now_ns(source_time_ns, now_ns), source_time_ns=source_time_ns,
            )
            # Every drain is a health transaction, including cold warmup and
            # the tail after a recovery baseline. A later good state cannot
            # erase an earlier bad state from the same drain.
            failure_bits |= self._radar_failure_bits(parsed)
            if failure_bits:
              break
            continue
          if isinstance(parsed, FilterStateHeader):
            self.diagnostics.observe_filter_header(parsed)
            continue
          if isinstance(parsed, FilterStateRecord):
            self.diagnostics.observe_filter_record(parsed)
            continue
          if isinstance(parsed, ObjectStatus):
            status_failure = self._status_failure_bits(parsed)
            if status_failure:
              failure_bits |= status_failure
              break
            self._last_status_source_ns = source_time_ns
            self.diagnostics.observe_status(parsed, self._diagnostic_now_ns(source_time_ns, now_ns))
          if not isinstance(parsed, (ObjectStatus, ObjectGeneral, ObjectQuality, ObjectExtended)):
            continue
          if baseline_cycle_collected:
            # Status closes one cycle and simultaneously opens the next. Keep
            # assembling a partial tail across drains, but never let a cycle
            # completed in this baseline drain pre-warm acceptance or object
            # identity history. Each complete tail cycle becomes only a fresh
            # empty counter baseline.
            tail_cycle = self.assembler.push(parsed, source_time_ns)
            if tail_cycle is not None:
              if not self._raw_cycle_complete(tail_cycle):
                failure_bits |= SHADOW_TRANSIENT_COUNTER_FAILED
                break
              tail_baseline_cycle = tail_cycle
            continue
          cycle = self.assembler.push(parsed, source_time_ns)
          if cycle is None:
            continue
          result = self.standard_tracker.update(cycle, max_distance_m=self._max_distance_m())
          completed_cycles.append((cycle, result))
          if self._trusted_baseline_established and self._recovery_needs_baseline:
            baseline_cycle_collected = True
        if failure_bits:
          break

      if not failure_bits:
        shadow_now_source_ns = self._shadow_now_source_ns(now_ns)
        current_failure_bits = self._current_failure_bits(shadow_now_source_ns)
        if self._trusted_baseline_established:
          failure_bits |= current_failure_bits
      if failure_bits:
        return self.hard_reset(failure_bits, host_now_ns=now_ns)

      self._parser_ready = self._parser_valid(self._shadow_now_source_ns(now_ns))
      shadow_ready = self._parser_ready and self._current_failure_bits(self._shadow_now_source_ns(now_ns)) == 0
      if not shadow_ready:
        # Parser and diagnostics may warm up, but no perception history is
        # allowed before the parser has become valid.
        self.snapshot = None
        self.perception_tracker.reset_after_fault()
        self.standard_tracker = ARS408Tracker()
        return self.state(now_ns)
      if not self._trusted_baseline_established or not parser_ready_at_start:
        # First fully trusted drain is discarded. Preserve the validated parser
        # and RadarState, but start a new target identity epoch and require the
        # following strict cycle to establish only a counter baseline.
        self._trusted_baseline_established = True
        epoch = int(self._epoch_factory())
        if not 0 < epoch < 1 << 64:
          raise ValueError("ARS408 producer epoch must be a non-zero UInt64")
        self.producer_epoch = epoch
        self.snapshot = None
        self.standard_tracker = ARS408Tracker()
        self.assembler = ARS408CycleAssembler()
        self._last_status_counter = None
        self.perception_tracker = self._new_perception_tracker()
        self._recovery_needs_baseline = True
        # A good configuration/source drain only warms the producer. Keep the
        # latched fault evidence and unavailable state through the barrier;
        # the next complete counter baseline is the sole recovery point.
        return self.state(now_ns)

      for cycle, result in completed_cycles:
        trusted_state = self.diagnostics.trusted_radar_state
        motion_rx_state = trusted_state.motion_rx_state if trusted_state is not None else None
        duplicate_rejected = tuple(raw_id for raw_id, reason in result.rejection_reasons if reason == "duplicate")
        baseline_only = self._recovery_needs_baseline
        self.snapshot = self.perception_tracker.update(
          cycle, int(cycle.measurement_mono_time_ns), motion_rx_state=motion_rx_state,
          accepted_raw_ids=result.accepted_raw_ids, duplicate_rejected_raw_ids=duplicate_rejected,
          input_trusted=True,
        )
        self.diagnostics.observe_cycle(cycle, result)
        components_complete = bool(
          self.snapshot.cycle_status == CycleStatus.EXACT and self.snapshot.general_complete and
          self.snapshot.quality_complete and self._extended_complete(cycle)
        )
        if baseline_only:
          baseline_complete = bool(
            cycle.exact and cycle.status.protocol_valid and cycle.general_count == cycle.status.object_count and
            cycle.quality_count == cycle.status.object_count and self._extended_complete(cycle)
          )
          if (not baseline_complete or self.snapshot.cycle_status != CycleStatus.INVALID or
              self.snapshot.stable_targets or self.snapshot.uncertain_targets or
              self.snapshot.clutter_suspects):
            return self.hard_reset(SHADOW_TRANSIENT_PRODUCER_FAILED, host_now_ns=now_ns)
          self._recovery_needs_baseline = False
          # A baseline never confirms or pre-warms targets. Its raw tail was
          # parsed only for safety and assembly continuity; clear the standard
          # tracker so acceptance history cannot leak into the next drain. The
          # closing Status remains the assembler's open status for the next cycle.
          self.standard_tracker = ARS408Tracker()
          if tail_baseline_cycle is not None:
            self.perception_tracker = self._new_perception_tracker()
            self.snapshot = self.perception_tracker.update(
              tail_baseline_cycle, int(tail_baseline_cycle.measurement_mono_time_ns),
              motion_rx_state=motion_rx_state, accepted_raw_ids=(), duplicate_rejected_raw_ids=(), input_trusted=True,
            )
            if (self.snapshot.cycle_status != CycleStatus.INVALID or self.snapshot.stable_targets or
                self.snapshot.uncertain_targets or self.snapshot.clutter_suspects):
              return self.hard_reset(SHADOW_TRANSIENT_PRODUCER_FAILED, host_now_ns=now_ns)
          break
        elif self.snapshot.counter_gap or not components_complete:
          return self.hard_reset(SHADOW_TRANSIENT_COUNTER_FAILED, host_now_ns=now_ns)

      if not self._recovery_needs_baseline:
        self._producer_fault = False
        self._latched_failure_bits = 0
      return self.state(now_ns)
    except Exception:
      return self.hard_reset(SHADOW_TRANSIENT_PRODUCER_FAILED, host_now_ns=now_ns)

  def state(self, host_now_ns: int | None = None) -> ARS408PerceptionState:
    now_ns = self._clock() if host_now_ns is None else int(host_now_ns)
    shadow_now_source_ns = self._shadow_now_source_ns(now_ns)
    parser_valid = self._parser_valid(shadow_now_source_ns)
    diagnostics_snapshot = self.diagnostics.snapshot(
      self._diagnostic_now_ns(shadow_now_source_ns, now_ns), parser_valid=parser_valid,
    )
    trusted_state = self.diagnostics.trusted_radar_state
    trusted_source_ns = self.diagnostics.trusted_radar_state_source_ns
    trusted_ready = bool(diagnostics_snapshot.trusted_radar_state_ready)
    radar_state_fresh = bool(
      trusted_ready and trusted_source_ns is not None and shadow_now_source_ns >= trusted_source_ns and
      shadow_now_source_ns - trusted_source_ns <= RADAR_STATE_TIMEOUT_NS
    )
    return ARS408PerceptionState(
      producer_epoch=self.producer_epoch,
      snapshot=self.snapshot,
      diagnostics=diagnostics_snapshot,
      parser_valid=parser_valid,
      motion_input_valid=bool(trusted_state is not None and trusted_state.motion_rx_state == 0),
      radar_state_ready=trusted_ready,
      radar_state_fresh=radar_state_fresh,
      shadow_now_source_ns=shadow_now_source_ns,
      lateral_transform_valid=LATERAL_KINEMATICS_VERIFIED,
      interference_active=bool(trusted_state and trusted_state.interference),
      config_observed=trusted_state is not None,
      config_valid=self._config_valid(trusted_state),
      transient_failure_bits=self._latched_failure_bits,
      producer_fault=self._producer_fault,
      producer_fault_count=self.producer_fault_count,
    )

  def _update_source_anchor(self, source_time_ns: int, host_now_ns: int) -> bool:
    if source_time_ns <= 0:
      self._source_clock_valid = False
      return False
    if self._source_clock_mode == "live":
      if source_time_ns > host_now_ns or host_now_ns - source_time_ns > LIVE_SOURCE_LAG_NS:
        self._source_clock_valid = False
        return False
    if self._source_anchor_ns is not None and source_time_ns <= self._source_anchor_ns:
      self._source_clock_valid = False
      return False
    self._source_anchor_ns = source_time_ns
    self._source_anchor_host_ns = host_now_ns
    self._source_clock_valid = True
    return True

  def _shadow_now_source_ns(self, host_now_ns: int) -> int:
    if not self._source_clock_valid or self._source_anchor_ns is None or self._source_anchor_host_ns is None:
      return 0
    if self._source_clock_mode == "source":
      return self._source_anchor_ns
    if host_now_ns < self._source_anchor_host_ns:
      self._source_clock_valid = False
      return 0
    # Live Event.logMonoTime and host_now_ns are both CLOCK_BOOTTIME. Returning
    # host time preserves real queue age instead of projecting an old source
    # anchor forward and laundering transport backlog.
    return host_now_ns

  def _current_failure_bits(self, shadow_now_source_ns: int) -> int:
    state = self.diagnostics.trusted_radar_state
    source_ns = self.diagnostics.trusted_radar_state_source_ns
    source_fresh = bool(
      self.diagnostics.trusted_radar_state_timestamp_valid and source_ns is not None and
      shadow_now_source_ns >= source_ns and shadow_now_source_ns - source_ns <= RADAR_STATE_TIMEOUT_NS
    )
    failures = 0
    if not source_fresh:
      failures |= SHADOW_TRANSIENT_SOURCE_FAILED
    if state is None:
      failures |= SHADOW_TRANSIENT_RADAR_FAILED | SHADOW_TRANSIENT_CONFIG_FAILED | SHADOW_TRANSIENT_MOTION_FAILED
    else:
      failures |= self._radar_failure_bits(state)
    if not self._parser_valid(shadow_now_source_ns):
      failures |= SHADOW_TRANSIENT_PARSER_FAILED
    return failures

  def _parser_valid(self, shadow_now_source_ns: int) -> bool:
    source_ns = self._last_required_frame_source_ns
    status_source_ns = self._last_status_source_ns
    return bool(
      self._required_frame_seen and source_ns is not None and shadow_now_source_ns >= source_ns and
      shadow_now_source_ns - source_ns <= PARSER_REQUIRED_FRESH_NS and status_source_ns is not None and
      shadow_now_source_ns >= status_source_ns and shadow_now_source_ns - status_source_ns <= PARSER_REQUIRED_FRESH_NS
    )

  def _diagnostic_now_ns(self, source_now_ns: int, host_now_ns: int) -> int:
    return source_now_ns if self._source_clock_mode == "source" else host_now_ns

  def _pre_batch_gap_failure_bits(self, first_source_ns: int, host_now_ns: int) -> int:
    if self._source_clock_mode == "source":
      boundary_source_ns = first_source_ns
    else:
      projected_source_ns = self._shadow_now_source_ns(host_now_ns)
      if projected_source_ns <= 0:
        return SHADOW_TRANSIENT_SOURCE_FAILED
      boundary_source_ns = max(first_source_ns, projected_source_ns)

    failures = 0
    required_source_ns = self._last_required_frame_source_ns
    if (required_source_ns is None or boundary_source_ns < required_source_ns or
        boundary_source_ns - required_source_ns > PARSER_REQUIRED_FRESH_NS):
      failures |= SHADOW_TRANSIENT_PARSER_FAILED

    status_source_ns = self._last_status_source_ns
    if (status_source_ns is None or boundary_source_ns < status_source_ns or
        boundary_source_ns - status_source_ns > PARSER_REQUIRED_FRESH_NS):
      failures |= SHADOW_TRANSIENT_PARSER_FAILED

    radar_source_ns = self.diagnostics.trusted_radar_state_source_ns
    if (not self.diagnostics.trusted_radar_state_timestamp_valid or radar_source_ns is None or
        boundary_source_ns < radar_source_ns or boundary_source_ns - radar_source_ns > RADAR_STATE_TIMEOUT_NS):
      failures |= SHADOW_TRANSIENT_SOURCE_FAILED
    return failures

  def _new_perception_tracker(self) -> ARS408PerceptionTracker:
    return ARS408PerceptionTracker(
      expected_counter_step=self._expected_counter_step,
      cadence_verified=self._cadence_verified,
      lateral_transform_verified=LATERAL_KINEMATICS_VERIFIED,
    )

  @staticmethod
  def _raw_cycle_complete(cycle) -> bool:
    expected = cycle.status.object_count
    return bool(
      cycle.exact and cycle.status.protocol_valid and cycle.general_count == expected and
      cycle.quality_count == expected and ARS408ShadowPipeline._extended_complete(cycle)
    )

  @staticmethod
  def _extended_complete(cycle) -> bool:
    expected = cycle.status.object_count
    return cycle.extended_count == expected and all(obj.extended is not None for obj in cycle.objects)

  def _status_failure_bits(self, status: ObjectStatus) -> int:
    if not status.protocol_valid:
      return SHADOW_TRANSIENT_PARSER_FAILED
    if not self._cadence_verified or self._expected_counter_step is None:
      return 0
    if self._last_status_counter is None:
      self._last_status_counter = status.measurement_counter
      return 0
    delta = (status.measurement_counter - self._last_status_counter) & 0xFFFF
    if delta != self._expected_counter_step:
      return SHADOW_TRANSIENT_COUNTER_FAILED
    self._last_status_counter = status.measurement_counter
    return 0

  @classmethod
  def _radar_failure_bits(cls, state: RadarStateSnapshot) -> int:
    failures = 0
    if state.voltage_error or state.temporary_error or state.temperature_error or state.persistent_error:
      failures |= SHADOW_TRANSIENT_RADAR_FAILED
    if not cls._config_valid(state):
      failures |= SHADOW_TRANSIENT_CONFIG_FAILED
    if state.interference:
      failures |= SHADOW_TRANSIENT_INTERFERENCE_FAILED
    if state.motion_rx_state != 0:
      failures |= SHADOW_TRANSIENT_MOTION_FAILED
    return failures

  @staticmethod
  def _config_valid(state: RadarStateSnapshot | None) -> bool:
    return bool(state is not None and state.sensor_id == ARS408_SENSOR_ID and state.output_type == 1 and
                state.quality_enabled and state.extended_enabled and
                state.max_distance_m == int(ARS408_MAX_DISTANCE_M))

  def _max_distance_m(self) -> float:
    state = self.diagnostics.radar_state
    if state is not None and 0 < state.max_distance_m <= ARS408_MAX_DISTANCE_M:
      return float(state.max_distance_m)
    return ARS408_MAX_DISTANCE_M
