from opendbc.sunnypilot.car.tesla.ars408.diagnostics import (
  ARS408Diagnostics, CONFIG_GRACE_REPORTS, INTERFERENCE_CONFIRM_REPORTS, RADAR_STATE_TIMEOUT_NS,
  PROTOCOL_INVALID_CONFIRM_CYCLES, STARTUP_GRACE_NS, STATUS_TIMEOUT_NS,
)
from opendbc.sunnypilot.car.tesla.ars408.models import (
  CycleResult, FilterStateHeader, FilterStateRecord, ObjectStatus, RadarStateSnapshot, TrackerResult,
  TrackedObject,
)


def radar_state(**overrides: object) -> RadarStateSnapshot:
  values: dict[str, object] = {
    "interference": False, "voltage_error": False, "temporary_error": False, "temperature_error": False,
    "persistent_error": False, "sensor_id": 0, "output_type": 1, "quality_enabled": True,
    "extended_enabled": True, "motion_rx_state": 0, "max_distance_m": 250, "nvm_read_status": 0,
    "nvm_write_status": 0, "sort_index": 1, "ctrl_relay_enabled": False, "rcs_threshold": 0,
  }
  values.update(overrides)
  return RadarStateSnapshot(**values)  # type: ignore[arg-type]


def empty_cycle(*, exact: bool = True, invalid: bool = False) -> CycleResult:
  return CycleResult(ObjectStatus(0, 1, 1), (), exact, invalid, 0, 0, 0)


def tracker_result(*reasons: tuple[int, str], tracks: tuple[TrackedObject, ...] = (),
                   accepted: tuple[int, ...] = ()) -> TrackerResult:
  return TrackerResult(tracks, accepted, reasons, 2, 3)


def test_timeouts_use_monotonic_time_and_recover() -> None:
  diagnostics = ARS408Diagnostics(100)
  before_grace = diagnostics.snapshot(100 + STARTUP_GRACE_NS - 1, parser_valid=False)
  assert not before_grace.errors.can_error

  now = 100 + STARTUP_GRACE_NS
  assert diagnostics.snapshot(now, parser_valid=True).errors.can_error
  diagnostics.observe_status(ObjectStatus(0, 1, 1), now)
  diagnostics.observe_radar_state(radar_state(), now)
  assert not diagnostics.snapshot(now, parser_valid=True).errors.can_error
  assert diagnostics.snapshot(now + STATUS_TIMEOUT_NS + 1, parser_valid=True).errors.can_error

  later = now + STATUS_TIMEOUT_NS + 2
  diagnostics.observe_status(ObjectStatus(0, 2, 1), later)
  assert not diagnostics.snapshot(later, parser_valid=True).errors.can_error
  assert diagnostics.snapshot(now + RADAR_STATE_TIMEOUT_NS + 1, parser_valid=True).errors.can_error


def test_fault_mapping_interference_confirmation_and_recovery() -> None:
  diagnostics = ARS408Diagnostics(0)
  diagnostics.observe_radar_state(radar_state(voltage_error=True, temperature_error=True), 1)
  errors = diagnostics.snapshot(1, parser_valid=True).errors
  assert errors.radar_fault and errors.radar_unavailable_temporary

  for now in range(2, INTERFERENCE_CONFIRM_REPORTS + 2):
    diagnostics.observe_radar_state(radar_state(interference=True), now)
  assert diagnostics.snapshot(INTERFERENCE_CONFIRM_REPORTS + 1, parser_valid=True).errors.radar_unavailable_temporary
  diagnostics.observe_radar_state(radar_state(), INTERFERENCE_CONFIRM_REPORTS + 2)
  recovered = diagnostics.snapshot(INTERFERENCE_CONFIRM_REPORTS + 2, parser_valid=True).errors
  assert not recovered.radar_fault and not recovered.radar_unavailable_temporary


def test_wrong_config_waits_for_state_report_grace() -> None:
  diagnostics = ARS408Diagnostics(0)
  bad = radar_state(sensor_id=5, output_type=2, quality_enabled=False, max_distance_m=200)
  for now in range(CONFIG_GRACE_REPORTS - 1):
    diagnostics.observe_radar_state(bad, now)
  assert not diagnostics.snapshot(CONFIG_GRACE_REPORTS - 2, parser_valid=True).errors.wrong_config
  diagnostics.observe_radar_state(bad, CONFIG_GRACE_REPORTS)
  assert diagnostics.snapshot(CONFIG_GRACE_REPORTS, parser_valid=True).errors.wrong_config
  diagnostics.observe_radar_state(radar_state(extended_enabled=False, motion_rx_state=3), CONFIG_GRACE_REPORTS + 1)
  assert not diagnostics.snapshot(CONFIG_GRACE_REPORTS + 1, parser_valid=True).errors.wrong_config


def test_filter_and_cycle_state_is_internal_and_bounded() -> None:
  diagnostics = ARS408Diagnostics(0)
  diagnostics.observe_filter_header(FilterStateHeader(0, 2))
  diagnostics.observe_filter_record(FilterStateRecord(8, True, 3.0, 7.0))
  diagnostics.observe_filter_record(FilterStateRecord(8, False, 2.0, 6.0))
  tracks = (
    TrackedObject(1, 1, 10.0, 0.0, 0.0, 0.0, 0.0, True, 2, 4, 0),
    TrackedObject(2, 2, 20.0, 0.0, 0.0, 0.0, 0.0, False, 2, 3, 0),
  )
  diagnostics.observe_cycle(empty_cycle(), tracker_result((1, "duplicate"), tracks=tracks, accepted=(1,)))
  metadata = diagnostics.snapshot(0, parser_valid=True)
  assert metadata.class_counts == ((2, 2),)
  assert metadata.probability_counts == ((3, 1), (4, 1))
  assert metadata.grace_held_tracks == 1
  diagnostics.observe_cycle(empty_cycle(exact=False), tracker_result())
  diagnostics.observe_cycle(empty_cycle(invalid=True), tracker_result())
  result = diagnostics.snapshot(0, parser_valid=True)
  assert result.filter_header == FilterStateHeader(0, 2)
  assert result.filter_records == (FilterStateRecord(8, False, 2.0, 6.0),)
  assert (result.exact_cycles, result.partial_cycles, result.invalid_cycles) == (1, 1, 1)
  assert diagnostics.rejection_counts == {"duplicate": 1}
  assert (diagnostics.handover_count, diagnostics.duplicate_suppression_count) == (2, 3)


def test_invalid_protocol_requires_two_cycles_and_recovers() -> None:
  diagnostics = ARS408Diagnostics(0)
  diagnostics.observe_radar_state(radar_state(), 0)
  for cycle in range(PROTOCOL_INVALID_CONFIRM_CYCLES - 1):
    diagnostics.observe_status(ObjectStatus(0, cycle, 2), cycle)
    diagnostics.observe_cycle(empty_cycle(invalid=True), tracker_result())
    assert not diagnostics.snapshot(cycle, parser_valid=True).errors.wrong_config
  diagnostics.observe_status(ObjectStatus(0, PROTOCOL_INVALID_CONFIRM_CYCLES, 2), PROTOCOL_INVALID_CONFIRM_CYCLES)
  diagnostics.observe_cycle(empty_cycle(invalid=True), tracker_result())
  assert diagnostics.snapshot(PROTOCOL_INVALID_CONFIRM_CYCLES, parser_valid=True).errors.wrong_config
  diagnostics.observe_cycle(empty_cycle(), tracker_result())
  assert not diagnostics.snapshot(PROTOCOL_INVALID_CONFIRM_CYCLES, parser_valid=True).errors.wrong_config


def test_missing_motion_feedback_is_temporary_and_recovers() -> None:
  diagnostics = ARS408Diagnostics(0)
  for state in (1, 2, 3):
    diagnostics.observe_radar_state(radar_state(motion_rx_state=state), state)
    errors = diagnostics.snapshot(state, parser_valid=True).errors
    assert errors.radar_unavailable_temporary
    assert not errors.radar_fault and not errors.wrong_config
  diagnostics.observe_radar_state(radar_state(), 4)
  assert not diagnostics.snapshot(4, parser_valid=True).errors.radar_unavailable_temporary
