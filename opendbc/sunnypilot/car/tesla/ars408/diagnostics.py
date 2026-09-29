from collections.abc import Iterable

from opendbc.car.carlog import carlog
from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_MAX_DISTANCE_M, ARS408_SENSOR_ID
from opendbc.sunnypilot.car.tesla.ars408.models import (
  CycleResult, DiagnosticErrors, DiagnosticsSnapshot, FilterStateHeader, FilterStateRecord, ObjectStatus,
  RadarStateSnapshot, SoftwareVersion, TrackerResult,
)


STARTUP_GRACE_NS = 10_000_000_000
STATUS_TIMEOUT_NS = 500_000_000
RADAR_STATE_TIMEOUT_NS = 3_000_000_000
CONFIG_GRACE_REPORTS = 10
INTERFERENCE_CONFIRM_REPORTS = 10
PROTOCOL_INVALID_CONFIRM_CYCLES = 2


class ARS408Diagnostics:
  def __init__(self, start_time_ns: int) -> None:
    if start_time_ns < 0:
      raise ValueError("start_time_ns must be non-negative")
    self.start_time_ns = start_time_ns
    self.last_status_ns: int | None = None
    self.last_radar_state_ns: int | None = None
    self.radar_state: RadarStateSnapshot | None = None
    self.software_version: SoftwareVersion | None = None
    self.radar_state_count = 0
    self.interference_count = 0
    self.exact_cycles = 0
    self.partial_cycles = 0
    self.invalid_cycles = 0
    self.consecutive_invalid_cycles = 0
    self.protocol_invalid = False
    self.raw_object_count = 0
    self.class_counts: dict[int, int] = {}
    self.probability_counts: dict[int, int] = {}
    self.grace_held_tracks = 0
    self.filter_header: FilterStateHeader | None = None
    self.filter_records: dict[int, FilterStateRecord] = {}
    self.rejection_counts: dict[str, int] = {}
    self.handover_count = 0
    self.duplicate_suppression_count = 0
    self._last_state_signature: tuple[object, ...] | None = None
    self._cycle_log_counter = 0

  def observe_status(self, status: ObjectStatus, now_ns: int) -> None:
    self._validate_time(now_ns)
    self.last_status_ns = now_ns
    self.raw_object_count = status.object_count

  def observe_radar_state(self, state: RadarStateSnapshot, now_ns: int) -> None:
    self._validate_time(now_ns)
    self.radar_state = state
    self.last_radar_state_ns = now_ns
    self.radar_state_count += 1
    self.interference_count = self.interference_count + 1 if state.interference else 0
    signature = (
      state.interference, state.voltage_error, state.temporary_error, state.temperature_error,
      state.persistent_error, state.sensor_id, state.output_type, state.quality_enabled,
      state.extended_enabled, state.motion_rx_state, state.max_distance_m,
    )
    if signature != self._last_state_signature:
      self._safe_log("warning", {"event": "ars408RadarState", "state": signature})
      self._last_state_signature = signature

  def observe_filter_header(self, header: FilterStateHeader) -> None:
    self.filter_header = header

  def observe_software_version(self, version: SoftwareVersion) -> None:
    if version != self.software_version:
      self._safe_log("debug", {"event": "ars408SoftwareVersion", "version": version})
    self.software_version = version

  def observe_filter_record(self, record: FilterStateRecord) -> None:
    previous = self.filter_records.get(record.index)
    self.filter_records[record.index] = record
    if record != previous:
      self._safe_log("debug", {"event": "ars408FilterState", "record": record})

  def observe_cycle(self, cycle: CycleResult, tracker: TrackerResult) -> None:
    self.protocol_invalid = cycle.invalid
    self.consecutive_invalid_cycles = self.consecutive_invalid_cycles + 1 if cycle.invalid else 0
    if cycle.invalid:
      self.invalid_cycles += 1
    elif cycle.exact:
      self.exact_cycles += 1
    else:
      self.partial_cycles += 1
    for _, reason in tracker.rejection_reasons:
      self.rejection_counts[reason] = self.rejection_counts.get(reason, 0) + 1
    self.handover_count = tracker.handover_count
    self.duplicate_suppression_count = tracker.duplicate_suppression_count
    self.class_counts = self._counts(track.object_class for track in tracker.tracks)
    self.probability_counts = self._counts(track.probability for track in tracker.tracks)
    self.grace_held_tracks = sum(track.raw_id not in tracker.accepted_raw_ids for track in tracker.tracks)
    if self._cycle_log_counter % 70 == 0:
      self._safe_log("debug", {
        "event": "ars408CycleSummary", "rawObjectCount": self.raw_object_count,
        "classes": self.class_counts, "probabilities": self.probability_counts,
        "rejections": self.rejection_counts, "graceHeld": self.grace_held_tracks,
        "handovers": self.handover_count, "duplicates": self.duplicate_suppression_count,
      })
    self._cycle_log_counter += 1

  def snapshot(self, now_ns: int, *, parser_valid: bool) -> DiagnosticsSnapshot:
    self._validate_time(now_ns)
    after_startup_grace = now_ns - self.start_time_ns >= STARTUP_GRACE_NS
    state = self.radar_state
    object_output_enabled = state is None or state.output_type == 1
    status_stale = object_output_enabled and self._stale(self.last_status_ns, now_ns, STATUS_TIMEOUT_NS)
    state_stale = self._stale(self.last_radar_state_ns, now_ns, RADAR_STATE_TIMEOUT_NS)
    can_error = after_startup_grace and (not parser_valid or status_stale or state_stale)

    hard_fault = bool(state and (state.voltage_error or state.persistent_error))
    temporary_fault = bool(state and (
      state.temperature_error or state.temporary_error or state.motion_rx_state != 0 or
      self.interference_count >= INTERFERENCE_CONFIRM_REPORTS
    ))
    wrong_config = (self.protocol_invalid and self.consecutive_invalid_cycles >= PROTOCOL_INVALID_CONFIRM_CYCLES) or bool(
      state and self.radar_state_count >= CONFIG_GRACE_REPORTS and (
      state.sensor_id != ARS408_SENSOR_ID or state.output_type != 1 or not state.quality_enabled or
      state.max_distance_m != int(ARS408_MAX_DISTANCE_M)
    ))
    return DiagnosticsSnapshot(
      errors=DiagnosticErrors(can_error, hard_fault, temporary_fault, wrong_config),
      radar_state_ready=state is not None,
      radar_state_count=self.radar_state_count,
      interference_count=self.interference_count,
      exact_cycles=self.exact_cycles,
      partial_cycles=self.partial_cycles,
      invalid_cycles=self.invalid_cycles,
      raw_object_count=self.raw_object_count,
      class_counts=tuple(sorted(self.class_counts.items())),
      probability_counts=tuple(sorted(self.probability_counts.items())),
      grace_held_tracks=self.grace_held_tracks,
      filter_header=self.filter_header,
      filter_records=tuple(self.filter_records[index] for index in sorted(self.filter_records)),
      software_version=self.software_version,
    )

  @staticmethod
  def _stale(last_ns: int | None, now_ns: int, timeout_ns: int) -> bool:
    return last_ns is None or now_ns - last_ns > timeout_ns

  def _validate_time(self, now_ns: int) -> None:
    if now_ns < self.start_time_ns:
      raise ValueError("diagnostic timestamp precedes initialization")

  @staticmethod
  def _safe_log(level: str, message: dict[str, object]) -> None:
    try:
      getattr(carlog, level)(message)
    except Exception:
      pass

  @staticmethod
  def _counts(values: Iterable[int]) -> dict[int, int]:
    counts: dict[int, int] = {}
    for value in values:
      counts[value] = counts.get(value, 0) + 1
    return counts
