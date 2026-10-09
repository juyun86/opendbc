"""Truck-related echo suppression does not merge ordinary convoy traffic."""
from dataclasses import replace

import pytest

from opendbc.sunnypilot.car.tesla.ars408.tests.test_tracker import obj, cycle as tracker_cycle
from opendbc.sunnypilot.car.tesla.ars408.tracker import ARS408Tracker


def scene():
  result = []
  for raw, distance, lateral, cls, length, width, rcs in (
    (60, 50., .6, 2, 9., 2.2, 30.),
    (29, 57.8, 1.2, 1, 4.4, 1.8, 25.),
    (43, 65.4, .2, 1, 4.4, 1.8, 0.),
  ):
    p = obj(raw, probability=7, d_rel=distance, y_rel=lateral, object_class=cls)
    result.append(replace(p, general=replace(p.general, rcs=rcs),
                          extended=replace(p.extended, length_m=length, width_m=width)))
  return result


def cycle(*objects, counter):
  result = tracker_cycle(*objects)
  return replace(result, status=replace(result.status, measurement_counter=counter))


def warm(tracker, objects, *, start_ns=1_000_000_000, counter=1, step=2):
  for i in range(12):
    result = tracker.update(cycle(*objects, counter=(counter + i * step) % 65536), timestamp_ns=start_ns + i * 100_000_000)
  return result


def test_confirmed_weak_echo_is_hidden_but_two_strong_measurements_keep_identity():
  t = ARS408Tracker()
  s = scene()
  first = t.update(cycle(*s, counter=65535), timestamp_ns=900_000_000)
  assert {p.raw_id for p in first.tracks} == {60, 29, 43}
  result = warm(t, s)
  assert {(p.raw_id, p.track_id, p.measured) for p in result.tracks} == {(60, 60, True), (29, 29, True), (43, 43, True)}
  assert 43 in result.accepted_raw_ids
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=25), timestamp_ns=2_200_000_000)
  assert {p.raw_id for p in result.tracks} == {60, 29}
  assert (43, 'duplicate') in result.rejection_reasons
  assert 43 not in result.accepted_raw_ids
  assert 43 in t._tracks  # Filtering must not delete the sensor's occupancy state.
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=2))
  result = t.update(cycle(*s, counter=27), timestamp_ns=2_300_000_000)
  assert {(p.raw_id, p.track_id, p.measured) for p in result.tracks} == {(60, 60, True), (29, 29, True), (43, 43, True)}


@pytest.mark.parametrize('fault', ['differentSpeed', 'adjacentLane', 'differentSpacing', 'strongEcho',
                                   'truckEcho', 'missingSize', 'notTruck', 'bodyBeyondLength', 'bodyPredicted'])
def test_platoon_and_uncertain_objects_are_not_suppressed(fault):
  s = scene()
  if fault == 'differentSpeed': s[2] = replace(s[2], general=replace(s[2].general, v_rel=-2.))
  if fault == 'adjacentLane': s[2] = replace(s[2], general=replace(s[2].general, y_rel=3.6))
  if fault == 'differentSpacing': s[2] = replace(s[2], general=replace(s[2].general, d_rel=70.))
  if fault == 'strongEcho': s[2] = replace(s[2], general=replace(s[2].general, rcs=25.))
  if fault == 'truckEcho': s[2] = replace(s[2], extended=replace(s[2].extended, object_class=2))
  if fault == 'missingSize': s[0] = replace(s[0], extended=replace(s[0].extended, length_m=0.))
  if fault == 'notTruck': s[0] = replace(s[0], extended=replace(s[0].extended, object_class=1))
  if fault == 'bodyBeyondLength':
    s[1] = replace(s[1], general=replace(s[1].general, d_rel=62.))
    s[2] = replace(s[2], general=replace(s[2].general, d_rel=74.))
  if fault == 'bodyPredicted': s[1] = replace(s[1], quality=replace(s[1].quality, measurement_state=3))
  t = ARS408Tracker();warm(t, s)
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=25), timestamp_ns=2_200_000_000)
  assert 43 in {p.raw_id for p in result.tracks}


def test_two_synchronous_points_alone_do_not_establish_ghost():
  result = warm(ARS408Tracker(), scene()[:2])
  assert {p.raw_id for p in result.tracks} == {60, 29}


def test_ambiguous_two_truck_references_do_not_hide_a_target():
  t = ARS408Tracker();s = scene()
  second = replace(s[0], general=replace(s[0].general, raw_id=61, d_rel=40.),
                   quality=replace(s[0].quality, raw_id=61), extended=replace(s[0].extended, raw_id=61))
  t.update(cycle(*s, second, counter=65535), timestamp_ns=900_000_000)
  second = replace(second, general=replace(second.general, d_rel=50.2))
  warm(t, [*s, second])
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, second, counter=25), timestamp_ns=2_200_000_000)
  assert 43 in {p.raw_id for p in result.tracks}


def test_same_position_new_report_does_not_inherit_cluster_confirmation():
  t = ARS408Tracker();s = scene();warm(t, s)
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=1))
  result = t.update(cycle(*s, counter=25), timestamp_ns=2_200_000_000)
  assert 43 in {p.raw_id for p in result.tracks}
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=27), timestamp_ns=2_300_000_000)
  assert 43 in {p.raw_id for p in result.tracks}


@pytest.mark.parametrize('fault', ['partial', 'invalid', 'timestampGap', 'anchorPrediction', 'newObject'])
def test_lost_cluster_evidence_exposes_prediction_immediately(fault):
  t = ARS408Tracker();s = scene();warm(t, s)
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  stamp = 2_200_000_000
  c = cycle(*s, counter=25)
  if fault == 'partial': c = replace(c, exact=False)
  if fault == 'invalid': c = replace(c, invalid=True)
  if fault == 'timestampGap': stamp += 200_000_000
  if fault == 'anchorPrediction':
    s[0] = replace(s[0], quality=replace(s[0].quality, measurement_state=3));c = cycle(*s, counter=25)
  if fault == 'newObject':
    s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=1),
                   general=replace(s[2].general, d_rel=80.));c = cycle(*s, counter=25)
  result = t.update(c, timestamp_ns=stamp)
  assert 43 in {p.raw_id for p in result.tracks}


def test_prediction_cannot_earn_confirmation():
  t = ARS408Tracker();s = scene()
  t.update(cycle(*s, counter=65535), timestamp_ns=900_000_000)
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = warm(t, s)
  assert 43 in {p.raw_id for p in result.tracks}


def test_prediction_at_confirmation_boundary_cannot_complete_warmup():
  t = ARS408Tracker();s = scene()
  for i in range(10):
    t.update(cycle(*s, counter=1 + 2 * i), timestamp_ns=1_000_000_000 + i * 100_000_000)
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=21), timestamp_ns=2_000_000_000)
  assert 43 in {p.raw_id for p in result.tracks}


def test_missing_echo_keeps_cluster_only_inside_original_two_cycle_grace():
  t = ARS408Tracker();s = scene();warm(t, s)
  for i in range(3):
    result = t.update(cycle(*s[:2], counter=25 + 2 * i), timestamp_ns=2_200_000_000 + i * 100_000_000)
    assert {p.raw_id for p in result.tracks} == {60, 29}
  assert 43 not in t._tracks


def test_repeated_measurement_cannot_confirm_echo():
  t = ARS408Tracker();s = scene()
  for i in range(15):
    result = t.update(cycle(*s, counter=99), timestamp_ns=1_000_000_000 + i * 100_000_000)
    assert {p.raw_id for p in result.tracks} == {60, 29, 43}
  assert not t._truck_echo_history
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=101), timestamp_ns=2_500_000_000)
  assert 43 in {p.raw_id for p in result.tracks}


def test_repeated_measurement_at_confirmation_boundary_cannot_complete_warmup():
  t = ARS408Tracker();s = scene()
  for i in range(10):
    t.update(cycle(*s, counter=1 + 2 * i), timestamp_ns=1_000_000_000 + i * 100_000_000)
  result = t.update(cycle(*s, counter=19), timestamp_ns=2_000_000_000)
  assert 43 in {p.raw_id for p in result.tracks}
  assert next(iter(t._truck_echo_history.values()))[1] == 1_900_000_000
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=21), timestamp_ns=2_050_000_000)
  assert 43 in {p.raw_id for p in result.tracks}


def test_confirmed_echo_short_retransmission_does_not_extend_freshness():
  t = ARS408Tracker();s = scene();warm(t, s)
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  t.update(cycle(*s, counter=25), timestamp_ns=2_200_000_000)
  for stamp in (2_300_000_000, 2_350_000_000):
    result = t.update(cycle(*s, counter=25), timestamp_ns=stamp)
    assert {p.raw_id for p in result.tracks} == {60, 29}
    assert next(iter(t._truck_echo_history.values()))[1] == 2_200_000_000
  result = t.update(cycle(*s, counter=25), timestamp_ns=2_350_000_001)
  assert {p.raw_id for p in result.tracks} == {60, 29, 43}
  assert not t._truck_echo_history
  assert {(p.raw_id, p.track_id, p.measured) for p in result.tracks} == {(60, 60, True), (29, 29, True), (43, 43, False)}
  result = t.update(cycle(*s, counter=27), timestamp_ns=2_400_000_000)
  assert 43 in {p.raw_id for p in result.tracks}
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=2))
  warm(t, s, start_ns=2_500_000_000, counter=29)
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=53), timestamp_ns=3_700_000_000)
  assert {p.raw_id for p in result.tracks} == {60, 29}


@pytest.mark.parametrize('counter,step', [(1, 2), (1, 4), (65535, 2)])
def test_fresh_counter_progression_and_wrap_confirm_echo(counter, step):
  t = ARS408Tracker();s = scene();warm(t, s, counter=counter, step=step)
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=(counter + 12 * step) % 65536), timestamp_ns=2_200_000_000)
  assert {p.raw_id for p in result.tracks} == {60, 29}


@pytest.mark.parametrize('interval_ns', [140_000_000, 154_000_000])
def test_short_retransmissions_between_fresh_cycles_keep_confirmation(interval_ns):
  t = ARS408Tracker();s = scene()
  for i in range(12):
    stamp = 1_000_000_000 + i * interval_ns
    t.update(cycle(*s, counter=1 + 2 * i), timestamp_ns=stamp)
    result = t.update(cycle(*s, counter=1 + 2 * i), timestamp_ns=stamp + interval_ns // 2)
    assert {p.raw_id for p in result.tracks} == {60, 29, 43}
  s[2] = replace(s[2], quality=replace(s[2].quality, measurement_state=3))
  result = t.update(cycle(*s, counter=25), timestamp_ns=1_000_000_000 + 12 * interval_ns)
  assert {p.raw_id for p in result.tracks} == {60, 29}
