from collections import deque
from dataclasses import replace
from pathlib import Path

import pytest

from opendbc.can import CANPacker
from opendbc.sunnypilot.car.tesla.ars408.constants import (
  ARS408_BUS, ARS408_GENERAL, ARS408_QUALITY,
)
from opendbc.sunnypilot.car.tesla.ars408.models import (
  ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus, ParsedFrame, RadarStateSnapshot,
)
from opendbc.sunnypilot.car.tesla.ars408.shadow import (
  ARS408ShadowPipeline, SHADOW_TRANSIENT_CONFIG_FAILED, SHADOW_TRANSIENT_COUNTER_FAILED,
  SHADOW_TRANSIENT_INTERFERENCE_FAILED, SHADOW_TRANSIENT_MOTION_FAILED, SHADOW_TRANSIENT_PARSER_FAILED,
  SHADOW_TRANSIENT_PRODUCER_FAILED, SHADOW_TRANSIENT_RADAR_FAILED, SHADOW_TRANSIENT_SOURCE_FAILED,
  PARSER_REQUIRED_FRESH_NS,
)


def radar_state() -> RadarStateSnapshot:
  return RadarStateSnapshot(False, False, False, False, False, 0, 1, True, True, 0, 250, 0, 0, 1, False, 0)


class ScriptedParser:
  def __init__(self, frames: list[ParsedFrame]) -> None:
    self.frames = deque(frames)
    self.can_valid = True

  def parse(self, _timestamp: int, _frame: tuple[int, bytes, int]) -> ParsedFrame | None:
    return self.frames.popleft()


def packets(count: int, start_ns: int) -> list[tuple[int, list[tuple[int, bytes, int]]]]:
  return [(start_ns + index, [(0x60A, b"", 1)]) for index in range(count)]


def cycle(counter: int, raw_id: int = 7) -> list[ParsedFrame]:
  return [
    ObjectStatus(1, counter, 1), ObjectGeneral(raw_id, 30.0, 0.2, -1.0, 0.0, 4.0, 0),
    ObjectQuality(raw_id, 6, 2), ObjectExtended(raw_id, 0.0, 1), ObjectStatus(1, counter + 1, 1),
  ]


def continued_cycle(close_counter: int, raw_id: int = 7) -> list[ParsedFrame]:
  return [
    ObjectGeneral(raw_id, 30.0, 0.2, -1.0, 0.0, 4.0, 0), ObjectQuality(raw_id, 6, 2),
    ObjectExtended(raw_id, 0.0, 1), ObjectStatus(1, close_counter, 1),
  ]


def probability_cycle(counter: int, probability: int, raw_id: int = 9) -> list[ParsedFrame]:
  return [
    ObjectStatus(1, counter, 1), ObjectGeneral(raw_id, 30.0, 0.2, -1.0, 0.0, 4.0, 0),
    ObjectQuality(raw_id, probability, 2), ObjectExtended(raw_id, 0.0, 1), ObjectStatus(1, counter + 1, 1),
  ]


def continued_probability_cycle(close_counter: int, probability: int, raw_id: int = 9) -> list[ParsedFrame]:
  return [
    ObjectGeneral(raw_id, 30.0, 0.2, -1.0, 0.0, 4.0, 0), ObjectQuality(raw_id, probability, 2),
    ObjectExtended(raw_id, 0.0, 1), ObjectStatus(1, close_counter, 1),
  ]


def pipeline(*, epochs=(11, 12, 13, 14, 15)) -> ARS408ShadowPipeline:
  values = iter(epochs)
  return ARS408ShadowPipeline(clock=lambda: 10_000, epoch_factory=lambda: next(values),
                              expected_counter_step=1, cadence_verified=True, source_clock_mode="source")


def run(current: ARS408ShadowPipeline, frames: list[ParsedFrame], start_ns: int):
  current.parser = ScriptedParser(frames)  # type: ignore[assignment]
  current._parser_was_valid = True
  current._parser_ready = True
  return current.process_batch(packets(len(frames), start_ns), host_now_ns=10_000)


def raw_run(current: ARS408ShadowPipeline, frames: list[ParsedFrame], start_ns: int):
  current.parser = ScriptedParser(frames)  # type: ignore[assignment]
  return current.process_batch(packets(len(frames), start_ns), host_now_ns=10_000)


def packed_radar_state(packer: CANPacker):
  return packer.make_can_msg("RadarState", ARS408_BUS, {
    "RadarState_MaxDistanceCfg": 250, "RadarState_SensorID": 0, "RadarState_OutputTypeCfg": 1,
    "RadarState_SendQualityCfg": 1, "RadarState_SendExtInfoCfg": 1, "RadarState_MotionRxState": 0,
  })


def packed_status(packer: CANPacker, count: int, counter: int):
  return packer.make_can_msg("Obj_0_Status", ARS408_BUS, {
    "Obj_NofObjects": count, "Obj_MeasCounter": counter, "Obj_InterfaceVersion": 1,
  })


def packed_general(packer: CANPacker, raw_id: int):
  return packer.make_can_msg("Obj_1_General", ARS408_BUS, {
    "Obj_ID": raw_id, "Obj_DistLong": 30, "Obj_DistLat": 0.2, "Obj_VrelLong": -1,
    "Obj_VrelLat": 0, "Obj_DynProp": 0, "Obj_RCS": 4,
  })


def packed_quality(packer: CANPacker, raw_id: int):
  return packer.make_can_msg("Obj_2_Quality", ARS408_BUS, {
    "Obj_ID": raw_id, "Obj_ProbOfExist": 6, "Obj_MeasState": 2,
  })


def packed_extended(packer: CANPacker, raw_id: int):
  return packer.make_can_msg("Obj_3_Extended", ARS408_BUS, {
    "Obj_ID": raw_id, "Obj_ArelLong": 0, "Obj_ArelLat": 0, "Obj_Class": 1,
    "Obj_OrientationAngle": 0, "Obj_Length": 4, "Obj_Width": 2,
  })


def packed_cycle(packer: CANPacker, counter: int, *, raw_id: int = 7, extended_id: int | None = None,
                 close_count: int = 1):
  return [
    packed_status(packer, 1, counter), packed_general(packer, raw_id), packed_quality(packer, raw_id),
    packed_extended(packer, raw_id if extended_id is None else extended_id),
    packed_status(packer, close_count, counter + 1),
  ]


def packed_continued_cycle(packer: CANPacker, close_counter: int, *, raw_id: int = 7,
                           extended_id: int | None = None, close_count: int = 1):
  return [
    packed_general(packer, raw_id), packed_quality(packer, raw_id),
    packed_extended(packer, raw_id if extended_id is None else extended_id),
    packed_status(packer, close_count, close_counter),
  ]


def raw_packets(frames, start_ns: int, step_ns: int = 1):
  return [(start_ns + index * step_ns, [frame]) for index, frame in enumerate(frames)]


def real_warm_to_hit(current: ARS408ShadowPipeline, packer: CANPacker, start_ns: int) -> int:
  first = [packed_radar_state(packer), *packed_cycle(packer, 1)]
  current.process_batch(raw_packets(first, start_ns), host_now_ns=start_ns + len(first))
  baseline_start = start_ns + 100
  current.process_batch(raw_packets(packed_cycle(packer, 2), baseline_start),
                        host_now_ns=baseline_start + 5)
  hit_start = start_ns + 200
  hit = current.process_batch(raw_packets(packed_continued_cycle(packer, 4), hit_start),
                              host_now_ns=hit_start + 4)
  assert hit.snapshot is not None and hit.snapshot.uncertain_targets[0].hit_count == 1
  return hit_start + 3


def real_pipeline(mode: str, epochs=(41, 42, 43, 44, 45)) -> ARS408ShadowPipeline:
  values = iter(epochs)
  return ARS408ShadowPipeline(
    clock=lambda: 1_000_000_000, epoch_factory=lambda: next(values), expected_counter_step=1,
    cadence_verified=True, source_clock_mode=mode,
  )


def warm_to_baseline(current: ARS408ShadowPipeline) -> None:
  assert run(current, [radar_state(), *cycle(1)], 100).snapshot is None
  baseline = run(current, cycle(2), 200)
  assert baseline.snapshot is not None and baseline.snapshot.cycle_status.value == "invalid"


def test_startup_epoch_is_nonzero_and_first_state_is_empty_barrier() -> None:
  state = pipeline().state(10_000)
  assert state.producer_epoch == 11
  assert state.snapshot is None and not state.producer_fault


def test_readme_documents_independent_process_and_ttl_without_old_card_claim() -> None:
  readme = (Path(__file__).resolve().parents[1] / "README.md").read_text(encoding="utf-8")
  assert "separate observational `ars408shadowd` subscribes to non-conflated raw `can`" in readme
  assert "There is no synchronous shadow IPC, serialization, wait, or publisher call" in readme
  assert "after processing it applies a 25 ms wall-time check" in readme
  assert "events over 256 KiB, and batches over 2 MiB" in readme
  assert "Future source timestamps and transport age over 250 ms fail closed" in readme
  assert "live shadow-now is the actual host `CLOCK_BOOTTIME`" in readme
  assert "standard acceptance/dedup tracker for every completed cycle" in readme
  assert "duplicate suppression is already worst-case `O(n^2)`" in readme
  assert "perception association, handover, and structural-clutter work add further" in readme
  assert "Shadow parser health does not use `CANParser.can_valid`" in readme
  assert "required-frame source age of at most 500 ms" in readme
  assert "Counter continuity is checked at Status arrival only when" in readme
  assert "service loss or local TTL expiry as `UNKNOWN`" in readme
  assert "REVIEW_BLOCKED / NO DEPLOYMENT AUTHORIZATION" in readme
  assert "still runs inside `RadarInterface.update()`" not in readme
  assert "10 ms card-loop budget" not in readme


def test_bad_then_good_same_drain_is_sticky_and_changes_epoch() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  state = run(current, [replace(radar_state(), sensor_id=5), radar_state()], 200)
  assert state.producer_epoch == 13 and state.snapshot is None and state.producer_fault
  assert state.transient_failure_bits & SHADOW_TRANSIENT_CONFIG_FAILED


def test_extended_disabled_then_good_same_drain_is_sticky_config_failure() -> None:
  current = pipeline()
  state = run(current, [replace(radar_state(), extended_enabled=False), radar_state()], 100)
  assert state.producer_epoch == 12 and state.snapshot is None and state.producer_fault
  assert not state.config_valid
  assert state.transient_failure_bits & SHADOW_TRANSIENT_CONFIG_FAILED


def test_source_regression_discards_whole_drain_and_clears_runtime() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  state = current.process_batch([(200, [(0, b"", 1)]), (199, [(0, b"", 1)])], host_now_ns=10_000)
  assert state.producer_epoch == 13 and state.snapshot is None and state.producer_fault
  assert state.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED


def test_first_healthy_cycle_after_barrier_is_baseline_only() -> None:
  current = pipeline()
  barrier = run(current, [radar_state(), *cycle(1)], 100)
  assert barrier.producer_epoch == 12 and barrier.snapshot is None
  first = run(current, cycle(2), 200)
  assert first.snapshot is not None
  assert not first.snapshot.stable_targets and not first.snapshot.uncertain_targets
  second = run(current, continued_cycle(4), 300)
  assert second.producer_epoch == 12 and second.snapshot is not None


def test_counter_gap_with_healthy_tail_cannot_recover_in_same_drain() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  run(current, cycle(2), 200)
  state = run(current, [ObjectStatus(1, 10, 1), *continued_cycle(11), *continued_cycle(12)], 300)
  assert state.producer_epoch == 13 and state.snapshot is None and state.producer_fault


def test_source_clock_mode_is_explicit_and_route_epoch_is_not_compared_to_host() -> None:
  with pytest.raises(TypeError):
    ARS408ShadowPipeline()  # type: ignore[call-arg]
  with pytest.raises(ValueError):
    ARS408ShadowPipeline(source_clock_mode="guess")
  route = ARS408ShadowPipeline(clock=lambda: 5, epoch_factory=lambda: 1, source_clock_mode="source")
  assert raw_run(route, [radar_state()], 9_000_000_000_000).radar_state_fresh
  live = ARS408ShadowPipeline(clock=lambda: 5, epoch_factory=iter((1, 2)).__next__, source_clock_mode="live")
  state = raw_run(live, [radar_state()], 9_000_000_000_000)
  assert state.producer_fault and state.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED


def test_live_future_source_timestamp_fails_but_source_mode_does_not_compare_host_epoch() -> None:
  live_epochs = iter((1, 2, 3))
  live = ARS408ShadowPipeline(
    clock=lambda: 100, epoch_factory=live_epochs.__next__, source_clock_mode="live",
    expected_counter_step=1, cadence_verified=True,
  )
  live.parser = ScriptedParser([radar_state()])  # type: ignore[assignment]
  future = live.process_batch(packets(1, 101), host_now_ns=100)
  assert future.producer_epoch == 2 and future.snapshot is None and future.producer_fault
  assert future.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED
  assert future.transient_failure_bits & SHADOW_TRANSIENT_PRODUCER_FAILED

  source = ARS408ShadowPipeline(
    clock=lambda: 100, epoch_factory=lambda: 7, source_clock_mode="source",
    expected_counter_step=1, cadence_verified=True,
  )
  source.parser = ScriptedParser([radar_state()])  # type: ignore[assignment]
  accepted = source.process_batch(packets(1, 101), host_now_ns=100)
  assert not (accepted.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED)
  assert accepted.radar_state_fresh


def test_live_stable_transport_backlog_cannot_warm_or_build_target_history() -> None:
  epochs = iter((1, 2, 3, 4, 5))
  current = ARS408ShadowPipeline(
    clock=lambda: 1_000_000_000, epoch_factory=epochs.__next__, source_clock_mode="live",
    expected_counter_step=1, cadence_verified=True,
  )
  for index, frames in enumerate(([radar_state(), *cycle(1)], cycle(2), continued_cycle(4))):
    host_now_ns = 1_000_000_000 + index * 100_000_000
    source_start_ns = host_now_ns - 900_000_000
    current.parser = ScriptedParser(list(frames))  # type: ignore[assignment]
    state = current.process_batch(packets(len(frames), source_start_ns), host_now_ns=host_now_ns)
    assert state.snapshot is None and state.producer_fault
    assert state.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED
    assert not current.standard_tracker._tracks and not current.perception_tracker._tracks


def test_live_transport_within_250ms_boundary_can_warm_and_reach_baseline() -> None:
  epochs = iter((11, 12, 13, 14, 15))
  current = ARS408ShadowPipeline(
    clock=lambda: 1_000_000_000, epoch_factory=epochs.__next__, source_clock_mode="live",
    expected_counter_step=1, cadence_verified=True,
  )

  def live_run(frames, host_now_ns: int):
    current.parser = ScriptedParser(list(frames))  # type: ignore[assignment]
    return current.process_batch(
      packets(len(frames), host_now_ns - 250_000_000), host_now_ns=host_now_ns,
    )

  warm = live_run([radar_state(), *cycle(1)], 1_000_000_000)
  assert not (warm.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED)
  baseline = live_run(cycle(2), 1_100_000_000)
  assert not (baseline.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED)
  assert baseline.snapshot is not None and baseline.snapshot.cycle_status.value == "invalid"


def test_cold_radar_state_and_object_components_across_drains_reach_baseline_then_hit_one() -> None:
  current = pipeline()
  trusted = raw_run(current, [radar_state()], 100)
  assert trusted.producer_epoch == 11 and trusted.snapshot is None and not trusted.parser_valid
  status_ready = raw_run(current, [ObjectStatus(1, 2, 1)], 200)
  assert status_ready.producer_epoch == 12 and status_ready.snapshot is None and status_ready.parser_valid
  raw_run(current, [ObjectStatus(1, 3, 1)], 205)
  raw_run(current, [ObjectGeneral(7, 30.0, 0.2, -1.0, 0.0, 4.0, 0)], 210)
  raw_run(current, [ObjectQuality(7, 6, 2)], 220)
  raw_run(current, [ObjectExtended(7, 0.0, 1)], 230)
  baseline = raw_run(current, [ObjectStatus(1, 4, 1)], 240)
  assert baseline.producer_epoch == 12 and not baseline.producer_fault
  assert baseline.snapshot is not None and baseline.snapshot.cycle_status.value == "invalid"
  assert not baseline.snapshot.stable_targets and not baseline.snapshot.uncertain_targets
  first_hit = run(current, continued_cycle(5), 300)
  assert first_hit.snapshot is not None and first_hit.snapshot.uncertain_targets[0].hit_count == 1


def test_baseline_drain_with_multiple_complete_cycles_drops_healthy_tail() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  state = run(current, [*cycle(2), *continued_cycle(4)], 200)
  assert not state.producer_fault and state.snapshot is not None
  assert state.snapshot.cycle_status.value == "invalid"
  assert not state.snapshot.stable_targets and not state.snapshot.uncertain_targets
  first_hit = run(current, continued_cycle(5), 400)
  assert first_hit.snapshot is not None and first_hit.snapshot.uncertain_targets[0].hit_count == 1


def test_baseline_tail_partial_cycle_keeps_status_anchor_without_prewarming_tracks() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  baseline = run(current, [*cycle(2), ObjectGeneral(7, 29.8, 0.2, -1.0, 0.0, 4.0, 0)], 200)
  assert not baseline.producer_fault and baseline.snapshot is not None
  assert baseline.snapshot.cycle_status.value == "invalid"
  assert current.standard_tracker._tracks == {} and current.perception_tracker._tracks == {}
  completed = run(current, [ObjectQuality(7, 6, 2), ObjectExtended(7, 0.0, 1), ObjectStatus(1, 4, 1)], 300)
  assert not completed.producer_fault and completed.snapshot is not None
  assert completed.snapshot.counter_gap == 0 and completed.snapshot.uncertain_targets[0].hit_count == 1


def test_baseline_tail_cannot_prewarm_standard_acceptance_or_logical_identity() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  tail = [
    *cycle(2),
    ObjectGeneral(9, 30.0, 0.2, -1.0, 0.0, 4.0, 0), ObjectQuality(9, 1, 2),
    ObjectExtended(9, 0.0, 1), ObjectStatus(1, 4, 1),
  ]
  baseline = run(current, tail, 200)
  assert baseline.snapshot is not None and baseline.snapshot.accepted_count == 0
  assert current.standard_tracker._tracks == {}
  next_state = run(current, continued_probability_cycle(5, 1), 400)
  assert next_state.snapshot is not None and next_state.snapshot.accepted_count == 0
  assert not next_state.snapshot.stable_targets and not next_state.snapshot.uncertain_targets
  assert current.standard_tracker._tracks == {}
  assert current.perception_tracker._tracks == {}


def test_baseline_tail_bad_radar_state_invalidates_whole_drain() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  state = run(current, [*cycle(2), replace(radar_state(), interference=True)], 200)
  assert state.producer_fault and state.snapshot is None and state.producer_epoch == 13
  assert state.transient_failure_bits & SHADOW_TRANSIENT_INTERFERENCE_FAILED


def test_baseline_tail_invalid_protocol_status_faults_immediately_and_cannot_recover_next_batch() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  state = run(current, [*cycle(2), ObjectStatus(1, 99, 0)], 200)
  assert state.producer_fault and state.snapshot is None and state.producer_epoch == 13
  assert state.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED
  recovery = run(current, [radar_state(), *cycle(100)], 400)
  assert recovery.producer_fault and recovery.snapshot is None and recovery.producer_epoch == 14


def test_baseline_tail_verified_counter_gap_faults_immediately() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  state = run(current, [*cycle(2), ObjectStatus(1, 99, 1)], 200)
  assert state.producer_fault and state.snapshot is None and state.producer_epoch == 13
  assert state.transient_failure_bits & SHADOW_TRANSIENT_COUNTER_FAILED


def test_unverified_cadence_does_not_claim_status_gap_detection() -> None:
  current = ARS408ShadowPipeline(clock=lambda: 10_000, epoch_factory=lambda: 1, source_clock_mode="source")
  assert current._status_failure_bits(ObjectStatus(0, 10, 1)) == 0
  assert current._status_failure_bits(ObjectStatus(0, 99, 1)) == 0


def test_baseline_tail_parser_failure_invalidates_whole_drain() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)

  class TailRejectingParser(ScriptedParser):
    def parse(self, timestamp, frame):
      parsed = super().parse(timestamp, frame)
      if len(self.frames) == 0:
        return None
      return parsed

  frames = [*cycle(2), radar_state()]
  current.parser = TailRejectingParser(frames)
  state = current.process_batch(packets(len(frames), 200), host_now_ns=10_000)
  assert state.producer_fault and state.snapshot is None and state.producer_epoch == 13
  assert state.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED


def test_baseline_drain_source_regression_is_sticky() -> None:
  current = pipeline()
  run(current, [radar_state(), *cycle(1)], 100)
  frames = cycle(2)
  current.parser = ScriptedParser(frames)
  timestamps = [200, 201, 202, 203, 199]
  state = current.process_batch([(timestamp, [(0x60A, b"", 1)]) for timestamp in timestamps], host_now_ns=10_000)
  assert state.producer_fault and state.snapshot is None and state.producer_epoch == 13
  assert state.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED


def test_cold_bad_then_good_radar_state_cannot_seed_trusted_warmup() -> None:
  current = pipeline()
  state = raw_run(current, [replace(radar_state(), sensor_id=5), radar_state()], 100)
  assert state.producer_fault and state.snapshot is None and state.producer_epoch == 12
  assert state.transient_failure_bits & SHADOW_TRANSIENT_CONFIG_FAILED
  recovery = raw_run(current, [radar_state()], 200)
  assert recovery.producer_fault and recovery.snapshot is None and recovery.producer_epoch == 12
  assert recovery.transient_failure_bits & SHADOW_TRANSIENT_PRODUCER_FAILED
  warm = run(current, cycle(2), 300)
  assert warm.producer_fault and warm.snapshot is None and warm.producer_epoch == 13
  baseline = run(current, cycle(3), 400)
  assert not baseline.producer_fault and baseline.snapshot is not None
  assert baseline.snapshot.cycle_status.value == "invalid"


def test_fault_evidence_remains_latched_through_partial_recovery_without_baseline() -> None:
  current = pipeline()
  faulted = current.hard_reset(SHADOW_TRANSIENT_CONFIG_FAILED, host_now_ns=10_000)
  assert faulted.producer_fault and faulted.transient_failure_bits & SHADOW_TRANSIENT_CONFIG_FAILED
  warm = raw_run(current, [radar_state()], 100)
  assert warm.producer_fault and warm.snapshot is None
  partial = raw_run(current, [ObjectGeneral(7, 30.0, 0.2, -1.0, 0.0, 4.0, 0)], 200)
  assert partial.producer_fault and partial.snapshot is None
  assert partial.transient_failure_bits & SHADOW_TRANSIENT_PRODUCER_FAILED


def test_serial_config_then_bad_dlc_faults_preserve_both_causes_until_baseline() -> None:
  current = pipeline()
  config_fault = raw_run(current, [replace(radar_state(), sensor_id=5)], 100)
  assert config_fault.transient_failure_bits & SHADOW_TRANSIENT_CONFIG_FAILED

  # hard_reset installs a fresh real parser. A required ARS408 frame with a
  # short DLC must add parser failure without erasing the earlier config cause.
  bad_dlc = current.process_batch([(200, [(ARS408_GENERAL, b"\x00" * 7, ARS408_BUS)])], host_now_ns=10_000)
  assert bad_dlc.producer_fault and bad_dlc.snapshot is None
  assert bad_dlc.transient_failure_bits & SHADOW_TRANSIENT_CONFIG_FAILED
  assert bad_dlc.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED
  assert bad_dlc.transient_failure_bits & SHADOW_TRANSIENT_PRODUCER_FAILED

  partial = raw_run(current, [radar_state()], 300)
  assert partial.producer_fault and partial.snapshot is None
  assert partial.transient_failure_bits & SHADOW_TRANSIENT_CONFIG_FAILED
  assert partial.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED

  run(current, cycle(2), 400)
  baseline = run(current, cycle(3), 500)
  assert not baseline.producer_fault
  assert baseline.transient_failure_bits == 0


def test_source_mode_semantics_do_not_depend_on_host_replay_speed() -> None:
  frames_by_drain = ([radar_state(), *cycle(1)], cycle(2), continued_cycle(4))
  states = []
  for host_times in ((10, 20, 30), (9_000_000_000_000, 9_500_000_000_000, 20_000_000_000_000)):
    current = pipeline(epochs=(61, 62, 63, 64, 65))
    route_states = []
    for index, (frames, host_now) in enumerate(zip(frames_by_drain, host_times, strict=True)):
      current.parser = ScriptedParser(list(frames))  # type: ignore[assignment]
      state = current.process_batch(packets(len(frames), 100 + index * 100), host_now_ns=host_now)
      route_states.append((
        state.producer_epoch, state.parser_valid, state.radar_state_fresh, state.producer_fault,
        state.transient_failure_bits, state.snapshot,
      ))
    states.append(route_states)
  assert states[0] == states[1]


@pytest.mark.parametrize(("bad_state", "reason"), [
  (replace(radar_state(), sensor_id=5), SHADOW_TRANSIENT_CONFIG_FAILED),
  (replace(radar_state(), interference=True), SHADOW_TRANSIENT_INTERFERENCE_FAILED),
  (replace(radar_state(), motion_rx_state=1), SHADOW_TRANSIENT_MOTION_FAILED),
  (replace(radar_state(), persistent_error=True), SHADOW_TRANSIENT_RADAR_FAILED),
])
def test_trusted_health_fault_at_head_discards_healthy_multicycle_tail(bad_state, reason: int) -> None:
  current = pipeline()
  warm_to_baseline(current)
  old_epoch = current.producer_epoch
  state = run(current, [bad_state, radar_state(), *continued_cycle(4), *continued_cycle(5)], 300)
  assert state.producer_epoch != old_epoch and state.snapshot is None and state.producer_fault
  assert state.transient_failure_bits & reason


def test_parser_fault_at_head_discards_healthy_tail() -> None:
  current = pipeline()
  warm_to_baseline(current)

  class InvalidatingParser(ScriptedParser):
    def parse(self, timestamp, frame):
      super().parse(timestamp, frame)
      return None

  current.parser = InvalidatingParser([radar_state(), *continued_cycle(4), *continued_cycle(5)])
  state = current.process_batch(packets(9, 300), host_now_ns=10_000)
  assert state.snapshot is None and state.producer_fault
  assert state.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED


def test_producer_exception_at_head_discards_healthy_tail() -> None:
  current = pipeline()
  warm_to_baseline(current)
  current.perception_tracker.update = lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("fault"))  # type: ignore[method-assign]
  state = run(current, [*continued_cycle(4), *continued_cycle(5)], 300)
  assert state.snapshot is None and state.producer_fault
  assert state.transient_failure_bits & SHADOW_TRANSIENT_PRODUCER_FAILED


def test_trusted_missing_extended_component_fails_closed_before_healthy_tail() -> None:
  current = pipeline()
  warm_to_baseline(current)
  incomplete = [
    ObjectGeneral(7, 29.0, 0.2, -1.0, 0.0, 4.0, 0), ObjectQuality(7, 6, 2),
    ObjectStatus(1, 4, 1), *continued_cycle(5),
  ]
  state = run(current, incomplete, 300)
  assert state.snapshot is None and state.producer_fault
  assert state.transient_failure_bits & SHADOW_TRANSIENT_COUNTER_FAILED


def test_real_parser_required_frame_seen_reject_and_freshness_contract() -> None:
  packer = CANPacker("ARS408")
  radar = packer.make_can_msg("RadarState", ARS408_BUS, {
    "RadarState_MaxDistanceCfg": 250, "RadarState_SensorID": 0, "RadarState_OutputTypeCfg": 1,
    "RadarState_SendQualityCfg": 1, "RadarState_SendExtInfoCfg": 1, "RadarState_MotionRxState": 0,
  })
  now = [1_000_000_000]
  epochs = iter((21, 22, 23, 24))
  current = ARS408ShadowPipeline(
    clock=lambda: now[0], epoch_factory=lambda: next(epochs), source_clock_mode="live",
  )
  assert not current.state(now[0]).parser_valid
  unrelated = current.process_batch([(now[0], [(0x123, b"\x00" * 8, ARS408_BUS)])], host_now_ns=now[0])
  assert not unrelated.parser_valid and not unrelated.producer_fault
  now[0] += 1
  valid = current.process_batch(
    [(now[0], [radar]), (now[0] + 1, [packed_status(packer, 0, 1)])], host_now_ns=now[0] + 1,
  )
  now[0] += 1
  assert valid.parser_valid

  now[0] += PARSER_REQUIRED_FRESH_NS + 1
  stale = current.process_batch([], host_now_ns=now[0])
  assert not stale.parser_valid and stale.producer_fault
  assert stale.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED


def test_real_parser_extended_id_mismatch_faults_recovery_baseline() -> None:
  packer = CANPacker("ARS408")
  current = real_pipeline("source")
  first = [packed_radar_state(packer), *packed_cycle(packer, 1)]
  current.process_batch(raw_packets(first, 1_000_000_000), host_now_ns=1_000_000_010)
  old_epoch = current.producer_epoch
  mismatch = current.process_batch(
    raw_packets(packed_cycle(packer, 2, extended_id=9), 1_000_000_100), host_now_ns=1_000_000_110,
  )
  assert mismatch.producer_epoch != old_epoch and mismatch.snapshot is None and mismatch.producer_fault
  assert mismatch.transient_failure_bits & SHADOW_TRANSIENT_PRODUCER_FAILED


def test_real_parser_extended_id_mismatch_and_orphan_zero_fault_active_epoch() -> None:
  packer = CANPacker("ARS408")
  mismatch_pipeline = real_pipeline("source")
  last_ns = real_warm_to_hit(mismatch_pipeline, packer, 2_000_000_000)
  old_epoch = mismatch_pipeline.producer_epoch
  mismatch = mismatch_pipeline.process_batch(
    raw_packets(packed_continued_cycle(packer, 5, extended_id=9), last_ns + 100),
    host_now_ns=last_ns + 110,
  )
  assert mismatch.producer_epoch != old_epoch and mismatch.snapshot is None and mismatch.producer_fault

  orphan_pipeline = real_pipeline("source", epochs=(51, 52, 53, 54, 55))
  first = [packed_radar_state(packer), *packed_cycle(packer, 1)]
  orphan_pipeline.process_batch(raw_packets(first, 3_000_000_000), host_now_ns=3_000_000_010)
  orphan_pipeline.process_batch(raw_packets(packed_cycle(packer, 2), 3_000_000_100), host_now_ns=3_000_000_110)
  hit = orphan_pipeline.process_batch(
    raw_packets(packed_continued_cycle(packer, 4, close_count=0), 3_000_000_200), host_now_ns=3_000_000_210,
  )
  assert hit.snapshot is not None
  old_epoch = orphan_pipeline.producer_epoch
  orphan = orphan_pipeline.process_batch(
    raw_packets([packed_extended(packer, 9), packed_status(packer, 0, 5)], 3_000_000_300),
    host_now_ns=3_000_000_310,
  )
  assert orphan.producer_epoch != old_epoch and orphan.snapshot is None and orphan.producer_fault


@pytest.mark.parametrize("mode", ("source", "live"))
def test_real_parser_mid_drain_required_ttl_cannot_be_laundered_by_fresh_head(mode: str) -> None:
  packer = CANPacker("ARS408")
  current = real_pipeline(mode)
  last_ns = real_warm_to_hit(current, packer, 4_000_000_000)
  old_epoch = current.producer_epoch
  packets_in_drain = [
    (last_ns + 400_000_000, [(0x123, b"\x00" * 8, ARS408_BUS)]),
    (last_ns + 600_000_000, [packed_radar_state(packer)]),
  ]
  assert packets_in_drain[-1][0] - packets_in_drain[0][0] <= 250_000_000
  state = current.process_batch(packets_in_drain, host_now_ns=packets_in_drain[-1][0])
  assert state.producer_epoch != old_epoch and state.snapshot is None and state.producer_fault
  assert state.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED


@pytest.mark.parametrize("mode", ("source", "live"))
def test_real_parser_mid_drain_radar_ttl_cannot_be_laundered_by_fresh_tail(mode: str) -> None:
  packer = CANPacker("ARS408")
  current = real_pipeline(mode)
  last_ns = real_warm_to_hit(current, packer, 5_000_000_000)
  old_epoch = current.producer_epoch
  for index, offset in enumerate(range(400_000_000, 2_900_000_000, 400_000_000)):
    packet = raw_packets(packed_continued_cycle(packer, 5 + index), last_ns + offset)
    state = current.process_batch(packet, host_now_ns=packet[-1][0])
    assert state.producer_epoch == old_epoch and not state.producer_fault
  final_drain = raw_packets(packed_continued_cycle(packer, 12), last_ns + 2_900_000_000)
  final_drain.append((last_ns + 3_100_000_000, [packed_radar_state(packer)]))
  assert final_drain[-1][0] - final_drain[0][0] <= 250_000_000
  state = current.process_batch(final_drain, host_now_ns=final_drain[-1][0])
  assert state.producer_epoch != old_epoch and state.snapshot is None and state.producer_fault
  assert state.transient_failure_bits & SHADOW_TRANSIENT_SOURCE_FAILED


@pytest.mark.parametrize("mode", ("source", "live"))
def test_real_parser_status_ttl_cannot_be_refreshed_by_other_required_frames(mode: str) -> None:
  packer = CANPacker("ARS408")
  current = real_pipeline(mode, epochs=(71, 72, 73, 74, 75, 76, 77))
  last_status_ns = real_warm_to_hit(current, packer, 6_000_000_000)
  old_epoch = current.producer_epoch
  first = [
    (last_status_ns + 400_000_000, [packed_radar_state(packer)]),
    (last_status_ns + 400_000_001, [packed_general(packer, 7)]),
  ]
  still_fresh = current.process_batch(first, host_now_ns=first[-1][0])
  assert still_fresh.producer_epoch == old_epoch and still_fresh.parser_valid and not still_fresh.producer_fault

  second = [
    (last_status_ns + 800_000_000, [packed_radar_state(packer)]),
    (last_status_ns + 800_000_001, [packed_general(packer, 7)]),
  ]
  stale = current.process_batch(second, host_now_ns=second[-1][0])
  assert stale.producer_epoch != old_epoch and stale.snapshot is None and stale.producer_fault
  assert not stale.parser_valid and stale.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED

  fault_epoch = stale.producer_epoch
  recovered_last_ns = real_warm_to_hit(current, packer, last_status_ns + 1_000_000_000)
  recovered = current.state(recovered_last_ns)
  assert recovered.producer_epoch != fault_epoch and recovered.snapshot is not None
  assert recovered.snapshot.uncertain_targets[0].hit_count == 1


@pytest.mark.parametrize("frame", [
  (ARS408_GENERAL, b"\x00" * 7, ARS408_BUS),
  (ARS408_QUALITY, bytes.fromhex("01 00 00 00 00 00 98"), ARS408_BUS),
])
def test_real_parser_required_bad_dlc_or_reserved_state_faults_new_epoch(frame) -> None:
  epochs = iter((31, 32))
  current = ARS408ShadowPipeline(
    clock=lambda: 1_000, epoch_factory=lambda: next(epochs), source_clock_mode="source",
  )
  state = current.process_batch([(100, [frame])], host_now_ns=1_000)
  assert state.producer_epoch == 32 and state.producer_fault and not state.parser_valid
  assert state.transient_failure_bits & SHADOW_TRANSIENT_PARSER_FAILED
