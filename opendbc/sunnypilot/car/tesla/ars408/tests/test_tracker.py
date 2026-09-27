from dataclasses import replace

import pytest

from opendbc.sunnypilot.car.tesla.ars408.models import (
  AssembledObject, CycleResult, ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus,
)
from opendbc.sunnypilot.car.tesla.ars408.tracker import ARS408Tracker


def obj(raw_id: int, *, probability: int = 4, state: int = 2, d_rel: float = 30.0, y_rel: float = 0.0,
        v_rel: float = -1.0, yv_rel: float = 0.0, dynamic_property: int = 0,
        object_class: int | None = 1) -> AssembledObject:
  extended = None if object_class is None else ObjectExtended(raw_id, 0.2, object_class)
  return AssembledObject(
    ObjectGeneral(raw_id, d_rel, y_rel, v_rel, yv_rel, 8.0, dynamic_property),
    ObjectQuality(raw_id, probability, state), extended,
  )


def cycle(*objects: AssembledObject, invalid: bool = False) -> CycleResult:
  return CycleResult(ObjectStatus(len(objects), 1, 1), tuple(objects), True, invalid, len(objects), len(objects), len(objects))


def test_probability_hysteresis_and_predicted_constraint() -> None:
  tracker = ARS408Tracker()
  assert tracker.update(cycle(obj(1, probability=2))).tracks == ()
  assert tracker.update(cycle(obj(2, state=3))).tracks == ()

  first = tracker.update(cycle(obj(1, probability=3)))
  assert first.tracks[0].measured
  retained = tracker.update(cycle(obj(1, probability=2, state=3)))
  assert retained.tracks[0].track_id == 1
  assert not retained.tracks[0].measured


def test_two_missed_cycles_are_held_unmeasured_then_track_expires() -> None:
  tracker = ARS408Tracker()
  tracker.update(cycle(obj(7)))
  for _ in range(2):
    held = tracker.update(cycle())
    assert len(held.tracks) == 1 and held.tracks[0].track_id == 7 and not held.tracks[0].measured
  expired = tracker.update(cycle())
  assert expired.tracks == ()
  assert (7, "timeout") in expired.rejection_reasons


def test_reused_raw_id_gets_new_never_reused_logical_id() -> None:
  tracker = ARS408Tracker()
  tracker.update(cycle(obj(7)))
  for _ in range(3):
    tracker.update(cycle())
  result = tracker.update(cycle(obj(7)))
  assert result.tracks[0].track_id == 256


def test_explicit_merge_handover_preserves_logical_id() -> None:
  tracker = ARS408Tracker()
  tracker.update(cycle(obj(10, d_rel=40.0)))
  result = tracker.update(cycle(
    obj(10, state=4, d_rel=40.4), obj(20, state=5, d_rel=40.5),
  ))
  assert [(track.raw_id, track.track_id, track.measured) for track in result.tracks] == [(20, 10, True)]
  assert result.handover_count == 1


def test_unique_kinematic_replacement_preserves_logical_id_but_ambiguous_does_not() -> None:
  tracker = ARS408Tracker()
  tracker.update(cycle(obj(10, d_rel=40.0)))
  replaced = tracker.update(cycle(obj(20, state=1, d_rel=40.5)))
  assert [(track.raw_id, track.track_id) for track in replaced.tracks] == [(20, 10)]

  ambiguous = ARS408Tracker()
  ambiguous.update(cycle(obj(1, d_rel=30.0), obj(2, d_rel=34.0)))
  ambiguous.update(cycle(obj(1, d_rel=30.0), obj(2, d_rel=30.5)))
  result = ambiguous.update(cycle(obj(3, state=1, d_rel=30.25)))
  assert any(track.track_id == 3 for track in result.tracks)
  assert result.handover_count == 0


def test_new_overlapping_duplicate_is_suppressed_but_established_pair_is_preserved() -> None:
  tracker = ARS408Tracker()
  tracker.update(cycle(obj(10, d_rel=40.0)))
  duplicate = tracker.update(cycle(obj(10, d_rel=40.1), obj(20, state=1, d_rel=40.4)))
  assert [track.track_id for track in duplicate.tracks] == [10]
  assert duplicate.duplicate_suppression_count == 1
  assert (20, "duplicate") in duplicate.rejection_reasons

  established = ARS408Tracker()
  established.update(cycle(obj(1, d_rel=30.0), obj(2, d_rel=34.0)))
  result = established.update(cycle(obj(1, d_rel=31.0), obj(2, d_rel=31.2)))
  assert [track.track_id for track in result.tracks] == [1, 2]


@pytest.mark.parametrize("dynamic_property", [1, 3, 5])
def test_static_and_crossing_stationary_objects_use_lateral_corridor(dynamic_property: int) -> None:
  tracker = ARS408Tracker()
  kept = tracker.update(cycle(obj(1, y_rel=5.5, dynamic_property=dynamic_property)))
  assert len(kept.tracks) == 1

  rejected = ARS408Tracker().update(cycle(obj(1, y_rel=5.6, dynamic_property=dynamic_property)))
  assert rejected.tracks == ()
  assert rejected.rejection_reasons == ((1, "static_outside_corridor"),)


def test_range_and_class_constraints_are_applied() -> None:
  assert ARS408Tracker().update(cycle(obj(1, d_rel=-0.1))).tracks == ()
  assert ARS408Tracker().update(cycle(obj(1, d_rel=251.0))).tracks == ()

  tracker = ARS408Tracker()
  tracker.update(cycle(obj(1, d_rel=30.0, object_class=1)))
  different_class = tracker.update(cycle(obj(2, state=1, d_rel=30.2, object_class=2)))
  assert sorted(track.track_id for track in different_class.tracks) == [1, 2]


def test_invalid_cycles_expire_tracks_and_do_not_refresh_from_untrusted_objects() -> None:
  tracker = ARS408Tracker()
  tracker.update(cycle(obj(1)))
  for _ in range(2):
    result = tracker.update(cycle(obj(1, d_rel=99.0), obj(2), invalid=True))
    assert len(result.tracks) == 1 and not result.tracks[0].measured
    assert result.tracks[0].d_rel == 30.0
  expired = tracker.update(cycle(invalid=True))
  assert expired.tracks == ()
  assert expired.rejection_reasons == ((1, "timeout"),)
  recovered = tracker.update(cycle(obj(1)))
  assert recovered.tracks[0].track_id == 256 and recovered.tracks[0].measured


def test_valid_cycle_recovers_within_invalid_cycle_grace() -> None:
  tracker = ARS408Tracker()
  tracker.update(cycle(obj(1)))
  tracker.update(cycle(invalid=True))
  recovered = tracker.update(cycle(obj(1, d_rel=31.0)))
  assert recovered.tracks[0].track_id == 1 and recovered.tracks[0].measured
  assert recovered.tracks[0].d_rel == 31.0


def test_partial_cycle_ages_only_missing_tracks() -> None:
  tracker = ARS408Tracker()
  tracker.update(cycle(obj(1, d_rel=30.0), obj(2, d_rel=40.0)))
  partial = replace(cycle(obj(1)), exact=False)
  result = tracker.update(partial)
  assert {track.raw_id: track.measured for track in result.tracks} == {1: True, 2: False}
