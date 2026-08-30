from dataclasses import replace

import pytest

from opendbc.sunnypilot.car.tesla.ars408.models import (
  AssembledObject, CycleResult, ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus,
)
from opendbc.sunnypilot.car.tesla.ars408.stabilizer import (
  ARS408PerceptionTracker, CadenceStepSource, CycleStatus, Lifecycle, ReasonBits, SemanticGroup,
)


CYCLE_NS = 75_000_000


def obj(raw_id: int, *, probability: int = 4, state: int = 2, d_rel: float = 30.0, y_rel: float = 0.0,
        v_rel: float = 0.0, yv_rel: float = 0.0, dynamic_property: int = 0,
        object_class: int | None = 1, rms: int = 5) -> AssembledObject:
  extended = None if object_class is None else ObjectExtended(raw_id, 0.1, object_class, 0.0, 0.0, 4.0, 1.8)
  return AssembledObject(
    ObjectGeneral(raw_id, d_rel, y_rel, v_rel, yv_rel, 5.0, dynamic_property),
    ObjectQuality(raw_id, probability, state, rms, rms, rms, rms, rms, rms, rms), extended,
  )


def cycle(counter: int, *objects: AssembledObject, exact: bool = True, invalid: bool = False,
          expected: int | None = None, extended_count: int | None = None) -> CycleResult:
  count = len(objects) if expected is None else expected
  return CycleResult(
    ObjectStatus(count, counter, 1), tuple(objects), exact, invalid, len(objects), len(objects),
    len(objects) if extended_count is None else extended_count, timestamp_valid=True,
  )


def update(tracker: ARS408PerceptionTracker, counter: int, *objects: AssembledObject,
           time_ns: int | None = None, motion_rx_state: int = 0,
           accepted_raw_ids: tuple[int, ...] | None = None, input_trusted: bool = True,
           duplicate_rejected_raw_ids: tuple[int, ...] = (),
           **cycle_kwargs: object):
  now_ns = counter * CYCLE_NS if time_ns is None else time_ns
  return tracker.update(
    cycle(counter, *objects, **cycle_kwargs), now_ns, motion_rx_state=motion_rx_state,
    accepted_raw_ids=accepted_raw_ids, duplicate_rejected_raw_ids=duplicate_rejected_raw_ids,
    input_trusted=input_trusted,
  )


def verified_tracker(step: int = 1, *, initial_counter: int = 0, initial_time_ns: int = 0,
                     lateral_transform_verified: bool = False) -> ARS408PerceptionTracker:
  tracker = ARS408PerceptionTracker(
    expected_counter_step=step, cadence_verified=True,
    lateral_transform_verified=lateral_transform_verified,
  )
  startup = tracker.update(cycle(initial_counter), initial_time_ns, motion_rx_state=0)
  assert startup.cycle_status == CycleStatus.INVALID
  assert not startup.cadence_valid and startup.cadence_verified
  return tracker


def test_single_frame_never_enters_stable_channel_and_expires_by_wall_time() -> None:
  tracker = verified_tracker()
  first = update(tracker, 1, obj(1))
  assert first.stable_targets == ()
  assert len(first.uncertain_targets) == 1

  held_for_diagnostics = update(tracker, 2, invalid=True, time_ns=CYCLE_NS + 180_000_001)
  assert held_for_diagnostics.stable_targets == ()
  assert len(held_for_diagnostics.uncertain_targets) == 1

  expired = update(tracker, 3, invalid=True, time_ns=CYCLE_NS + 400_000_001)
  assert expired.stable_targets == () and expired.uncertain_targets == ()
  assert len(expired.expired_events) == 1
  assert expired.expired_events[0].lifecycle == Lifecycle.EXPIRED


def test_vehicle_requires_three_of_four_new_measurements() -> None:
  tracker = verified_tracker()
  assert update(tracker, 1, obj(1)).stable_targets == ()
  assert update(tracker, 2, obj(1, d_rel=30.1)).stable_targets == ()
  confirmed = update(tracker, 3, obj(1, d_rel=30.2))
  assert len(confirmed.stable_targets) == 1
  assert confirmed.stable_targets[0].semantic_group == SemanticGroup.VEHICLE


def test_vru_requires_accepted_probability_and_confirms_after_two_measurements() -> None:
  tracker = verified_tracker()
  assert update(tracker, 1, obj(3, probability=1, object_class=3)).uncertain_targets == ()
  assert update(tracker, 2, obj(3, probability=3, object_class=3)).stable_targets == ()
  confirmed = update(tracker, 3, obj(3, probability=3, object_class=3))
  assert len(confirmed.stable_targets) == 1
  target = confirmed.stable_targets[0]
  assert target.semantic_group == SemanticGroup.PEDESTRIAN and target.possible_vru


def test_duplicate_counter_does_not_add_hits_or_refresh_measurement() -> None:
  tracker = verified_tracker()
  first = update(tracker, 1, obj(1))
  duplicate = tracker.update(cycle(1, obj(1)), 2 * CYCLE_NS, motion_rx_state=0)
  assert duplicate.cycle_status == CycleStatus.DUPLICATE
  assert not duplicate.cadence_valid and duplicate.cadence_verified
  target = duplicate.uncertain_targets[0]
  assert target.hit_count == first.uncertain_targets[0].hit_count == 1
  assert not target.fresh_measured


def test_partial_cycle_is_diagnostic_only_and_cannot_promote_or_refresh() -> None:
  tracker = verified_tracker()
  update(tracker, 1, obj(1))
  before = update(tracker, 2, obj(1, d_rel=30.1)).uncertain_targets[0]
  partial = update(tracker, 3, obj(1, d_rel=35.0), exact=False, expected=2)
  assert partial.stable_targets == ()
  target = partial.uncertain_targets[0]
  assert not target.fresh_measured
  assert target.hit_count == before.hit_count
  assert target.d_rel == before.d_rel
  assert target.reason_bits & ReasonBits.PARTIAL_CYCLE
  confirmed = update(tracker, 4, obj(1, d_rel=30.2))
  assert len(confirmed.stable_targets) == 1


def test_missing_extended_is_explicit_and_last_value_is_not_presented_as_fresh() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    snapshot = update(tracker, counter, obj(1))
  assert snapshot.stable_targets[0].length == 4.0
  missing = update(tracker, 4, obj(1, object_class=None), extended_count=0)
  target = missing.stable_targets[0]
  assert target.length == 4.0
  assert not target.extended_fresh
  assert target.reason_bits & ReasonBits.MISSING_EXTENDED


def test_unique_id_handover_preserves_logical_id() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    update(tracker, counter, obj(10, d_rel=40.0 + counter * 0.1))
  handover = update(tracker, 4, obj(20, state=1, d_rel=40.4))
  assert handover.stable_targets[0].logical_id == 10
  assert handover.stable_targets[0].raw_id == 20
  assert handover.stable_targets[0].reason_bits & ReasonBits.ID_HANDOVER


def test_vehicle_vru_class_conflict_becomes_unknown_obstacle_not_no_target() -> None:
  tracker = verified_tracker()
  update(tracker, 1, obj(1, object_class=1))
  update(tracker, 2, obj(1, object_class=3))
  result = update(tracker, 3, obj(1, object_class=1))
  target = (result.stable_targets or result.uncertain_targets)[0]
  assert target.semantic_group == SemanticGroup.UNKNOWN_OBSTACLE
  assert target.possible_vru
  assert target.reason_bits & ReasonBits.CLASS_CONFLICT


def test_structural_clutter_requires_valid_motion_input_and_multiple_tracks() -> None:
  tracker = verified_tracker()
  objects = (obj(1, d_rel=20.0, y_rel=5.0, dynamic_property=1, object_class=0),
             obj(2, d_rel=40.0, y_rel=5.2, dynamic_property=1, object_class=0),
             obj(3, d_rel=60.0, y_rel=4.9, dynamic_property=1, object_class=0))
  for counter in range(1, 4):
    result = update(tracker, counter, *objects)
  assert len(result.clutter_suspects) == 3
  assert result.stable_targets == ()  # output categories are mutually exclusive
  assert all(target.reason_bits & ReasonBits.STRUCTURAL_CLUTTER for target in result.clutter_suspects)

  no_motion = verified_tracker()
  for counter in range(1, 4):
    result = update(no_motion, counter, *objects, motion_rx_state=3)
  assert result.clutter_suspects == ()
  assert len(result.stable_targets) == 3


def test_explicit_deleted_state_does_not_coast_as_a_ghost() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    result = update(tracker, counter, obj(4))
  assert len(result.stable_targets) == 1

  deleted = update(tracker, 4, obj(4, state=0))
  assert deleted.stable_targets == () and deleted.uncertain_targets == ()
  event = deleted.expired_events[0]
  assert event.reason_bits & ReasonBits.EXPLICIT_DELETE
  assert not event.fresh_measured
  assert event.measurement_state == 0
  assert not event.reason_bits & ReasonBits.MOTION_INPUT_INVALID


def test_impossible_same_raw_id_jump_starts_new_logical_generation() -> None:
  tracker = verified_tracker()
  update(tracker, 1, obj(7, d_rel=20.0))
  jumped = update(tracker, 2, obj(7, d_rel=80.0))
  assert jumped.expired_events[0].reason_bits & ReasonBits.KINEMATIC_JUMP
  assert jumped.uncertain_targets[0].logical_id == 256


def test_rms_channels_are_mapped_to_the_matching_kinematic_axis() -> None:
  tracker = verified_tracker()
  update(tracker, 1, obj(1, y_rel=0.0, v_rel=0.0, rms=0))
  quality = ObjectQuality(1, 4, 2, 0, 30, 0, 0, 0, 0, 0)
  measured = replace(obj(1, y_rel=0.5, v_rel=0.5, rms=0), quality=quality)
  result = update(tracker, 2, measured)
  target = result.uncertain_targets[0]
  assert target.y_rel > target.v_rel  # low lateral-position RMS reacts faster than high longitudinal-speed RMS


def test_two_fresh_low_existence_frames_leave_stable_channel() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    result = update(tracker, counter, obj(1))
  assert len(result.stable_targets) == 1
  assert len(update(tracker, 4, obj(1, probability=1)).stable_targets) == 1
  demoted = update(tracker, 5, obj(1, probability=1))
  assert demoted.stable_targets == ()
  assert demoted.uncertain_targets[0].reason_bits & ReasonBits.LOW_PROBABILITY


def test_predicted_frame_uses_last_measurement_epoch_for_reacquisition() -> None:
  tracker = verified_tracker()
  first = update(tracker, 1, obj(7, d_rel=100.0, v_rel=-40.0), time_ns=0)
  logical_id = first.uncertain_targets[0].logical_id
  update(tracker, 2, obj(7, d_rel=97.0, v_rel=-40.0), time_ns=75_000_000)
  confirmed = update(tracker, 3, obj(7, d_rel=94.0, v_rel=-40.0), time_ns=150_000_000)
  assert confirmed.stable_targets[0].logical_id == logical_id
  filtered_before = confirmed.stable_targets[0].d_rel

  predicted = update(tracker, 4, obj(7, state=3, d_rel=91.0, v_rel=-40.0), time_ns=225_000_000)
  coast = predicted.stable_targets[0]
  assert coast.lifecycle == Lifecycle.COASTING and not coast.fresh_measured
  assert coast.hit_count == 3 and coast.last_measured_age_ms == 75.0
  assert coast.d_rel == filtered_before

  restored = update(tracker, 5, obj(7, d_rel=88.0, v_rel=-40.0), time_ns=300_000_000)
  target = restored.stable_targets[0]
  assert restored.expired_events == ()
  assert target.logical_id == logical_id and target.raw_d_rel == 88.0
  assert target.lifecycle == Lifecycle.CONFIRMED_FRESH and target.hit_count == 4 and target.miss_count == 0


def test_multiple_predicted_frames_do_not_move_filter_epoch_or_semantics() -> None:
  tracker = verified_tracker()
  update(tracker, 1, obj(8, d_rel=100.0, v_rel=-40.0), time_ns=0)
  update(tracker, 2, obj(8, d_rel=98.0, v_rel=-40.0), time_ns=50_000_000)
  confirmed = update(tracker, 3, obj(8, d_rel=96.0, v_rel=-40.0), time_ns=100_000_000)
  logical_id = confirmed.stable_targets[0].logical_id
  filtered_before = confirmed.stable_targets[0].d_rel
  update(tracker, 4, obj(8, state=3, d_rel=94.0, v_rel=-40.0, object_class=3), time_ns=150_000_000)
  second = update(tracker, 5, obj(8, state=3, d_rel=92.0, v_rel=-40.0, object_class=3), time_ns=200_000_000)
  coast = second.stable_targets[0]
  assert coast.d_rel == filtered_before and coast.hit_count == 3
  assert coast.semantic_group == SemanticGroup.VEHICLE and not coast.possible_vru
  restored = update(tracker, 6, obj(8, d_rel=90.0, v_rel=-40.0), time_ns=250_000_000)
  assert restored.stable_targets[0].logical_id == logical_id


def test_predicted_only_does_not_keep_tentative_or_confirmed_track_alive() -> None:
  tentative = verified_tracker()
  update(tentative, 1, obj(1), time_ns=0)
  expired = update(tentative, 2, obj(1, state=3), time_ns=400_000_001)
  assert expired.uncertain_targets == () and len(expired.expired_events) == 1

  confirmed = verified_tracker()
  for counter, now_ns in enumerate((0, 50_000_000, 100_000_000), 1):
    update(confirmed, counter, obj(2), time_ns=now_ns)
  expired = update(confirmed, 4, obj(2, state=3), time_ns=300_000_001)
  assert expired.stable_targets == () and len(expired.expired_events) == 1


def test_unverified_lateral_velocity_never_causes_identity_jump() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    result = update(tracker, counter, obj(5, y_rel=1.0, yv_rel=2.0), motion_rx_state=3)
  logical_id = result.stable_targets[0].logical_id
  changed = update(tracker, 4, obj(5, y_rel=1.1, yv_rel=-2.0), motion_rx_state=3)
  target = changed.stable_targets[0]
  assert target.logical_id == logical_id and changed.expired_events == ()
  assert target.reason_bits & ReasonBits.MOTION_INPUT_INVALID
  assert target.reason_bits & ReasonBits.LATERAL_KINEMATICS_UNVERIFIED


@pytest.mark.parametrize("lateral_transform_verified", [False, True])
def test_lateral_reason_matches_transform_verification(lateral_transform_verified: bool) -> None:
  tracker = verified_tracker(lateral_transform_verified=lateral_transform_verified)
  for counter in range(1, 4):
    result = update(tracker, counter, obj(6, y_rel=0.5, yv_rel=0.1))
  target = result.stable_targets[0]
  has_unverified_reason = bool(target.reason_bits & ReasonBits.LATERAL_KINEMATICS_UNVERIFIED)
  assert has_unverified_reason is not lateral_transform_verified


def test_handover_requires_mutual_unique_match() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    update(tracker, counter, obj(10, d_rel=40.0))
  ambiguous = update(tracker, 4, obj(20, d_rel=39.9), obj(21, d_rel=40.1))
  new_targets = [target for target in ambiguous.uncertain_targets if target.raw_id in (20, 21)]
  assert len(new_targets) == 2
  assert all(target.logical_id != 10 for target in new_targets)


def test_high_rms_upper_bound_cannot_handover_across_thirty_metres() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    update(tracker, counter, obj(10, d_rel=40.0, rms=30))
  far = update(tracker, 4, obj(20, d_rel=70.0, rms=30))
  target = next(target for target in far.uncertain_targets if target.raw_id == 20)
  assert target.logical_id == 20


def test_timeout_precedes_same_raw_id_reuse() -> None:
  tracker = verified_tracker()
  for counter, now_ns in enumerate((0, 75_000_000, 150_000_000), 1):
    result = update(tracker, counter, obj(7), time_ns=now_ns)
  old_id = result.stable_targets[0].logical_id
  reused = update(tracker, 4, obj(7), time_ns=400_000_001)
  assert reused.expired_events[0].logical_id == old_id
  assert reused.uncertain_targets[0].logical_id != old_id


def test_standard_tracker_accepted_ids_prevent_double_counting() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    result = update(
      tracker, counter, obj(1, d_rel=30.0), obj(2, d_rel=30.5), accepted_raw_ids=(1,),
    )
  assert [(target.logical_id, target.raw_id) for target in result.stable_targets] == [(1, 1)]
  assert result.uncertain_targets == ()
  assert result.raw_count == 2 and result.accepted_count == 1


def test_standard_duplicate_rejection_immediately_expires_existing_shadow_track() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    confirmed = update(
      tracker, counter, obj(1, d_rel=30.0), obj(2, d_rel=40.0),
      accepted_raw_ids=(1, 2),
    )
  assert {target.raw_id for target in confirmed.stable_targets} == {1, 2}

  duplicate = update(
    tracker, 4, obj(1, d_rel=30.1), obj(2, d_rel=40.1),
    accepted_raw_ids=(1,), duplicate_rejected_raw_ids=(2,),
  )
  assert [target.raw_id for target in duplicate.stable_targets] == [1]
  assert duplicate.uncertain_targets == duplicate.clutter_suspects == ()
  assert len(duplicate.expired_events) == 1
  event = duplicate.expired_events[0]
  assert event.raw_id == 2 and event.lifecycle == Lifecycle.EXPIRED
  assert not event.fresh_measured and event.measurement_state == 2
  assert event.reason_bits & ReasonBits.STANDARD_DUPLICATE_REJECTED
  assert not event.reason_bits & ReasonBits.MOTION_INPUT_INVALID
  assert 2 not in tracker._tracks


@pytest.mark.parametrize(("rejected_counter", "rejected_time_ns", "expected_status", "next_counter"), [
  (5, 4 * CYCLE_NS, CycleStatus.INVALID, 6),
  (3, 4 * CYCLE_NS, CycleStatus.DUPLICATE, 4),
])
def test_standard_duplicate_rejection_removes_track_even_when_cycle_counter_is_not_consumable(
  rejected_counter: int, rejected_time_ns: int, expected_status: CycleStatus, next_counter: int,
) -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    confirmed = update(
      tracker, counter, obj(1, d_rel=30.0), obj(2, d_rel=40.0),
      accepted_raw_ids=(1, 2),
    )
  assert {target.raw_id for target in confirmed.stable_targets} == {1, 2}

  rejected = update(
    tracker, rejected_counter, obj(1, d_rel=30.1), obj(2, probability=5, state=1, d_rel=40.1),
    time_ns=rejected_time_ns, accepted_raw_ids=(1,), duplicate_rejected_raw_ids=(2,),
  )
  assert rejected.cycle_status == expected_status
  assert 2 not in {target.raw_id for target in (
    *rejected.stable_targets, *rejected.uncertain_targets, *rejected.clutter_suspects,
  )}
  assert len(rejected.expired_events) == 1
  event = rejected.expired_events[0]
  assert event.raw_id == 2 and event.lifecycle == Lifecycle.EXPIRED
  assert not event.fresh_measured and event.measurement_state == 1
  assert event.reason_bits & ReasonBits.STANDARD_DUPLICATE_REJECTED
  assert not event.reason_bits & ReasonBits.MOTION_INPUT_INVALID
  assert 2 not in tracker._tracks

  recovered = update(
    tracker, next_counter, obj(1, d_rel=30.2), obj(2, d_rel=40.2),
    time_ns=rejected_time_ns + CYCLE_NS, accepted_raw_ids=(1,),
  )
  assert recovered.cycle_status == CycleStatus.EXACT
  assert 2 not in {target.raw_id for target in (
    *recovered.stable_targets, *recovered.uncertain_targets, *recovered.clutter_suspects,
  )}
  assert recovered.expired_events == () and 2 not in tracker._tracks


@pytest.mark.parametrize("rejected", [obj(2, probability=1, d_rel=40.1), obj(2, d_rel=260.0)])
def test_nonduplicate_standard_rejection_keeps_existing_track_coasting(rejected: AssembledObject) -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    update(
      tracker, counter, obj(1, d_rel=30.0), obj(2, d_rel=40.0),
      accepted_raw_ids=(1, 2),
    )

  result = update(
    tracker, 4, obj(1, d_rel=30.1), rejected,
    accepted_raw_ids=(1,), duplicate_rejected_raw_ids=(),
  )
  target = next(target for target in result.stable_targets if target.raw_id == 2)
  assert target.lifecycle == Lifecycle.COASTING and not target.fresh_measured
  assert result.expired_events == () and 2 in tracker._tracks


def test_rejected_negative_and_over_range_raw_objects_never_enter_consumable_lists() -> None:
  tracker = verified_tracker()
  result = update(
    tracker, 1, obj(1, d_rel=-1.0), obj(2, d_rel=260.0), accepted_raw_ids=(),
  )
  assert result.raw_count == 2 and result.accepted_count == 0
  assert result.stable_targets == result.uncertain_targets == result.clutter_suspects == ()


def test_out_of_order_counter_is_invalid_and_does_not_refresh() -> None:
  tracker = verified_tracker(initial_counter=9)
  first = update(tracker, 10, obj(1), time_ns=0).uncertain_targets[0]
  old = update(tracker, 8, obj(1), time_ns=75_000_000)
  target = old.uncertain_targets[0]
  assert old.cycle_status == CycleStatus.INVALID
  assert not old.cadence_valid and old.cadence_verified and old.expected_counter_step == 1
  assert old.counter_gap == ((8 - 10) & 0xFFFF)
  assert old.counter_anomaly_count == 1 and old.dropped_cycle_count == 0
  assert target.hit_count == first.hit_count and not target.fresh_measured
  recovered = update(tracker, 11, obj(1), time_ns=150_000_000)
  assert recovered.cycle_status == CycleStatus.EXACT
  assert recovered.cadence_valid and recovered.expected_counter_step == 1


def test_device_capture_verified_startup_requires_contiguous_second_counter() -> None:
  tracker = ARS408PerceptionTracker(expected_counter_step=1, cadence_verified=True)
  first = update(tracker, 10, time_ns=0)
  assert first.cycle_status == CycleStatus.INVALID
  assert not first.cadence_valid and first.cadence_verified
  assert first.step_source == CadenceStepSource.DEVICE_CAPTURE

  second = update(tracker, 11, time_ns=CYCLE_NS)
  assert second.cycle_status == CycleStatus.EXACT
  assert second.cadence_valid and second.cadence_verified
  assert second.step_source == CadenceStepSource.DEVICE_CAPTURE


def test_counter_gap_is_invalid_for_one_cycle_and_wrap_recovers() -> None:
  tracker = verified_tracker(initial_counter=9)
  assert update(tracker, 10, obj(1), time_ns=0).cycle_status == CycleStatus.EXACT
  normal = update(tracker, 11, obj(1), time_ns=75_000_000)
  assert normal.cycle_status == CycleStatus.EXACT and normal.counter_gap == 0
  gap = update(tracker, 15, obj(1), time_ns=150_000_000)
  assert gap.cycle_status == CycleStatus.INVALID
  assert gap.counter_gap == 4 and gap.dropped_cycle_count == 3
  assert gap.counter_anomaly_count == 1
  assert not gap.cadence_valid and gap.cadence_verified and gap.expected_counter_step == 1
  recovered = update(tracker, 16, obj(1), time_ns=225_000_000)
  assert recovered.cycle_status == CycleStatus.EXACT and recovered.counter_gap == 0

  wrapped = verified_tracker(initial_counter=65534)
  assert update(wrapped, 65535, obj(2), time_ns=0).cycle_status == CycleStatus.EXACT
  assert update(wrapped, 0, obj(2), time_ns=75_000_000).cycle_status == CycleStatus.EXACT


def test_default_step_one_inference_is_diagnostic_only() -> None:
  tracker = ARS408PerceptionTracker()
  snapshots = [update(tracker, 20 + index, time_ns=index * CYCLE_NS) for index in range(6)]

  assert all(snapshot.cycle_status == CycleStatus.INVALID for snapshot in snapshots)
  assert all(not snapshot.cadence_valid and not snapshot.cadence_verified for snapshot in snapshots)
  inferred = snapshots[-1]
  assert inferred.cadence_inferred and inferred.expected_counter_step == 1
  assert inferred.step_source == CadenceStepSource.INFERRED


def test_default_step_two_inference_is_diagnostic_only() -> None:
  tracker = ARS408PerceptionTracker()
  snapshots = [update(tracker, 20 + 2 * index, time_ns=index * CYCLE_NS) for index in range(6)]

  assert all(snapshot.cycle_status == CycleStatus.INVALID for snapshot in snapshots)
  inferred = snapshots[-1]
  assert not inferred.cadence_valid and not inferred.cadence_verified
  assert inferred.cadence_inferred and inferred.expected_counter_step == 2
  assert inferred.step_source == CadenceStepSource.INFERRED


def test_skipped_step_one_frames_at_150ms_never_infer_step_two() -> None:
  tracker = ARS408PerceptionTracker()
  snapshots = [update(tracker, 10 + 2 * index, time_ns=index * 150_000_000) for index in range(6)]
  assert all(snapshot.cycle_status == CycleStatus.INVALID for snapshot in snapshots)
  assert all(not snapshot.cadence_inferred and snapshot.expected_counter_step == 0 for snapshot in snapshots)
  assert snapshots[-1].dropped_cycle_count == 0


def test_duplicate_does_not_advance_counter_learning() -> None:
  tracker = ARS408PerceptionTracker()
  assert update(tracker, 20, time_ns=0).cycle_status == CycleStatus.INVALID
  assert update(tracker, 21, time_ns=CYCLE_NS).cycle_status == CycleStatus.INVALID
  duplicate = update(tracker, 21, time_ns=2 * CYCLE_NS)
  assert duplicate.cycle_status == CycleStatus.DUPLICATE
  assert not duplicate.cadence_valid and duplicate.expected_counter_step == 0
  assert duplicate.counter_anomaly_count == duplicate.dropped_cycle_count == 0
  assert update(tracker, 22, time_ns=3 * CYCLE_NS).cycle_status == CycleStatus.INVALID
  snapshots = [update(tracker, counter, time_ns=(counter - 19) * CYCLE_NS) for counter in range(23, 28)]
  assert all(snapshot.cycle_status == CycleStatus.INVALID for snapshot in snapshots)
  assert snapshots[-1].cadence_inferred and not snapshots[-1].cadence_verified


@pytest.mark.parametrize(("step", "wrapped_counter"), [(1, 0), (2, 1)])
def test_locked_counter_cadence_accepts_modulo_wrap(step: int, wrapped_counter: int) -> None:
  tracker = verified_tracker(step=step, initial_counter=(65535 - step) & 0xFFFF)
  assert update(tracker, 65535, time_ns=0).cycle_status == CycleStatus.EXACT
  wrapped = update(tracker, wrapped_counter, time_ns=CYCLE_NS)
  assert wrapped.cycle_status == CycleStatus.EXACT
  assert wrapped.counter_gap == 0 and wrapped.dropped_cycle_count == 0
  assert wrapped.counter_anomaly_count == 0
  assert wrapped.cadence_valid and wrapped.cadence_verified and wrapped.expected_counter_step == step
  assert not wrapped.cadence_inferred and wrapped.step_source == CadenceStepSource.DEVICE_CAPTURE


@pytest.mark.parametrize(("step", "restart_counters"), [
  (1, (0, 1, 2, 3, 4, 5)),
  (2, (0, 2, 4, 6, 8, 10)),
])
def test_counter_restart_resynchronizes_verified_step_before_accepting_cycles(
  step: int, restart_counters: tuple[int, ...],
) -> None:
  tracker = verified_tracker(step=step, initial_counter=100 - step)
  assert update(tracker, 100, time_ns=0).cycle_status == CycleStatus.EXACT

  snapshots = [
    update(tracker, counter, time_ns=(index + 1) * CYCLE_NS)
    for index, counter in enumerate(restart_counters)
  ]
  restart, locked = snapshots[0], snapshots[-1]
  assert all(snapshot.cycle_status == CycleStatus.INVALID for snapshot in snapshots[:-1])
  assert not restart.cadence_valid and restart.cadence_verified and restart.expected_counter_step == step
  assert locked.cycle_status == CycleStatus.EXACT
  assert locked.cadence_valid and locked.cadence_verified and locked.expected_counter_step == step
  assert locked.step_source == CadenceStepSource.DEVICE_CAPTURE
  assert locked.counter_gap == 0 and locked.dropped_cycle_count == 0
  assert locked.counter_anomaly_count == 1


@pytest.mark.parametrize(("step", "restart_counters"), [
  (1, (0, 1, 2, 3, 4, 5)),
  (2, (0, 2, 4, 6, 8, 10)),
])
def test_high_counter_reset_to_zero_never_guesses_dropped_cycles(
  step: int, restart_counters: tuple[int, ...],
) -> None:
  tracker = verified_tracker(step=step, initial_counter=40_000 - step)
  assert update(tracker, 40_000, time_ns=0).cycle_status == CycleStatus.EXACT

  snapshots = [
    update(tracker, counter, time_ns=(index + 1) * CYCLE_NS)
    for index, counter in enumerate(restart_counters)
  ]
  restart, locked = snapshots[0], snapshots[-1]
  assert restart.cycle_status == CycleStatus.INVALID
  assert restart.counter_gap == ((0 - 40_000) & 0xFFFF)
  assert restart.dropped_cycle_count == 0 and restart.counter_anomaly_count == 1
  assert not restart.cadence_valid and restart.cadence_verified
  assert all(snapshot.cycle_status == CycleStatus.INVALID for snapshot in snapshots[:-1])
  assert locked.cycle_status == CycleStatus.EXACT
  assert locked.cadence_valid and locked.cadence_verified
  assert locked.step_source == CadenceStepSource.DEVICE_CAPTURE


def test_step_two_forward_gap_counts_only_provable_dropped_cycles() -> None:
  tracker = verified_tracker(step=2, initial_counter=8)
  assert update(tracker, 10, time_ns=0).cycle_status == CycleStatus.EXACT
  gap = update(tracker, 16, time_ns=CYCLE_NS)
  assert gap.cycle_status == CycleStatus.INVALID
  assert gap.counter_gap == 6  # raw modulo delta
  assert gap.dropped_cycle_count == 2  # 6 / expected step 2 - 1
  assert gap.counter_anomaly_count == 1
  assert not gap.cadence_valid and gap.cadence_verified


def test_non_integral_forward_gap_never_guesses_dropped_cycles() -> None:
  tracker = verified_tracker(step=2, initial_counter=8)
  assert update(tracker, 10, time_ns=0).cycle_status == CycleStatus.EXACT
  gap = update(tracker, 15, time_ns=CYCLE_NS)
  assert gap.cycle_status == CycleStatus.INVALID
  assert gap.counter_gap == 5 and gap.counter_anomaly_count == 1
  assert gap.dropped_cycle_count == 0
  assert not gap.cadence_valid and gap.cadence_verified


def test_prelocked_counter_step_must_be_supported() -> None:
  with pytest.raises(ValueError, match="1 or 2"):
    ARS408PerceptionTracker(expected_counter_step=3)
  with pytest.raises(ValueError, match="requires an expected"):
    ARS408PerceptionTracker(cadence_verified=True)


def test_explicit_unverified_step_never_makes_cadence_valid() -> None:
  tracker = ARS408PerceptionTracker(expected_counter_step=1)
  snapshots = [update(tracker, counter, time_ns=counter * CYCLE_NS) for counter in range(1, 8)]
  assert all(snapshot.cycle_status == CycleStatus.INVALID for snapshot in snapshots)
  assert all(not snapshot.cadence_valid and not snapshot.cadence_verified for snapshot in snapshots)
  assert snapshots[-1].step_source == CadenceStepSource.EXTERNAL


def test_reset_after_fault_clears_tracks_and_requires_fresh_continuity_and_confirmation() -> None:
  tracker = verified_tracker()
  for counter in range(1, 4):
    stable = update(tracker, counter, obj(1))
  old_logical_id = stable.stable_targets[0].logical_id
  # Emulate a handler fault after mutating a track but before publishing.
  tracker._tracks[1].hit_count = 99

  tracker.reset_after_fault()
  assert tracker.snapshot is None and tracker._tracks == {}
  first = update(tracker, 10, obj(1), time_ns=0)
  assert first.cycle_status == CycleStatus.INVALID
  assert not first.cadence_valid and first.cadence_verified
  assert first.stable_targets == first.uncertain_targets == ()

  second = update(tracker, 11, obj(1), time_ns=CYCLE_NS)
  assert second.cycle_status == CycleStatus.EXACT
  assert second.cadence_valid and second.cadence_verified
  assert len(second.uncertain_targets) == 1
  assert second.uncertain_targets[0].hit_count == 1
  assert second.uncertain_targets[0].logical_id != old_logical_id
  assert second.counter_anomaly_count == second.dropped_cycle_count == 0

  assert update(tracker, 12, obj(1), time_ns=2 * CYCLE_NS).stable_targets == ()
  reconfirmed = update(tracker, 13, obj(1), time_ns=3 * CYCLE_NS)
  assert len(reconfirmed.stable_targets) == 1
  assert reconfirmed.stable_targets[0].logical_id != old_logical_id
  assert reconfirmed.stable_targets[0].hit_count == 3


def test_untrusted_input_clears_targets_and_recovery_requires_new_confirmation() -> None:
  tracker = verified_tracker()
  blocked = [
    update(tracker, counter, obj(9), input_trusted=False)
    for counter in range(1, 4)
  ]
  assert all(snapshot.cycle_status == CycleStatus.INVALID for snapshot in blocked)
  assert all(snapshot.stable_targets == snapshot.uncertain_targets == () for snapshot in blocked)

  first_healthy = update(tracker, 4, obj(9))
  assert first_healthy.cycle_status == CycleStatus.INVALID
  assert first_healthy.stable_targets == first_healthy.uncertain_targets == ()
  assert not first_healthy.cadence_valid and first_healthy.cadence_verified

  second_healthy = update(tracker, 5, obj(9))
  assert second_healthy.cycle_status == CycleStatus.EXACT
  assert second_healthy.stable_targets == ()
  assert second_healthy.uncertain_targets[0].hit_count == 1
  assert update(tracker, 6, obj(9)).stable_targets == ()
  confirmed = update(tracker, 7, obj(9))
  assert len(confirmed.stable_targets) == 1
  assert confirmed.stable_targets[0].hit_count == 3


def test_snapshot_preserves_measurement_component_and_close_timestamps() -> None:
  tracker = verified_tracker()
  timed_cycle = replace(
    cycle(1, obj(1)), measurement_mono_time_ns=100,
    last_component_mono_time_ns=150, assembly_close_mono_time_ns=200,
  )
  snapshot = tracker.update(timed_cycle, timed_cycle.measurement_mono_time_ns, motion_rx_state=0)
  assert snapshot.source_mono_time == 100
  assert snapshot.last_component_mono_time == 150
  assert snapshot.assembly_close_mono_time == 200


def test_structural_clutter_band_has_total_width_limit() -> None:
  tracker = verified_tracker()
  objects = (
    obj(1, d_rel=20.0, y_rel=4.0, dynamic_property=1, object_class=0),
    obj(2, d_rel=40.0, y_rel=5.0, dynamic_property=1, object_class=0),
    obj(3, d_rel=60.0, y_rel=6.0, dynamic_property=1, object_class=0),
  )
  for counter in range(1, 4):
    result = update(tracker, counter, *objects)
  assert result.clutter_suspects == ()
  assert len(result.stable_targets) == 3


def test_logical_id_bookkeeping_is_bounded() -> None:
  tracker = verified_tracker()
  for counter in range(1, 80, 2):
    update(tracker, counter, obj(1), time_ns=counter * CYCLE_NS)
    update(tracker, counter + 1, obj(1, state=0), time_ns=(counter + 1) * CYCLE_NS)
  assert not hasattr(tracker, "_used_logical_ids")
  assert tracker._seen_raw_ids_mask.bit_length() <= 256
