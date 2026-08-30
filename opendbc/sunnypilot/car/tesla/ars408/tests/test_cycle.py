import pytest

from opendbc.sunnypilot.car.tesla.ars408.cycle import ARS408CycleAssembler
from opendbc.sunnypilot.car.tesla.ars408.models import ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus


def general(raw_id: int) -> ObjectGeneral:
  return ObjectGeneral(raw_id, 30.0 + raw_id, 0.2, -1.0, 0.0, 5.0, 0)


def quality(raw_id: int) -> ObjectQuality:
  return ObjectQuality(raw_id, 4, 2)


def close(assembler: ARS408CycleAssembler, next_count: int = 0, timestamp_ns: int = 200):
  return assembler.push(ObjectStatus(next_count, 2, 1), timestamp_ns)


def test_complete_cycle_pairs_all_parts_by_raw_id() -> None:
  assembler = ARS408CycleAssembler()
  assert assembler.push(ObjectStatus(2, 1, 1), 100) is None
  assembler.push(general(2), 110)
  assembler.push(quality(1), 120)
  assembler.push(ObjectExtended(1, 0.3, 1), 130)
  assembler.push(general(1), 140)
  assembler.push(quality(2), 150)

  result = close(assembler)
  assert result is not None and result.exact and not result.invalid and result.timestamp_valid
  assert [obj.raw_id for obj in result.objects] == [1, 2]
  assert result.objects[0].extended == ObjectExtended(1, 0.3, 1)
  assert result.objects[1].extended is None


def test_partial_cycle_salvages_only_general_quality_intersection() -> None:
  assembler = ARS408CycleAssembler()
  assembler.push(ObjectStatus(3, 1, 1), 100)
  assembler.push(general(1), 110)
  assembler.push(quality(1), 120)
  assembler.push(general(2), 130)
  assembler.push(quality(3), 140)

  result = close(assembler)
  assert result is not None and not result.exact and not result.invalid
  assert [obj.raw_id for obj in result.objects] == [1]


def test_duplicate_core_part_is_not_silently_selected() -> None:
  assembler = ARS408CycleAssembler()
  assembler.push(ObjectStatus(1, 1, 1), 100)
  assembler.push(general(1), 110)
  assembler.push(general(1), 120)
  assembler.push(quality(1), 130)

  result = close(assembler)
  assert result is not None and not result.exact
  assert result.objects == ()
  assert result.duplicate_ids == (1,)


def test_invalid_status_and_zero_object_cycle_are_distinct() -> None:
  assembler = ARS408CycleAssembler()
  assembler.push(ObjectStatus(101, 1, 1), 100)
  invalid = close(assembler)
  assert invalid is not None and invalid.invalid

  empty = close(assembler, timestamp_ns=300)
  assert empty is not None and empty.exact and not empty.invalid and empty.objects == ()


def test_object_parts_before_status_are_ignored() -> None:
  assembler = ARS408CycleAssembler()
  assert assembler.push(general(1), 80) is None
  assert assembler.push(quality(1), 90) is None
  assert assembler.push(ObjectStatus(0, 1, 1), 100) is None
  result = close(assembler)
  assert result is not None and result.objects == ()


def test_cycle_timestamp_belongs_to_closed_counter_not_next_status() -> None:
  assembler = ARS408CycleAssembler()
  assert assembler.push(ObjectStatus(1, 1, 1), 100) is None
  assembler.push(general(1), 140)
  assembler.push(quality(1), 150)
  result = assembler.push(ObjectStatus(0, 2, 1), 200)
  assert result is not None and result.status.measurement_counter == 1
  assert result.measurement_mono_time_ns == 100
  assert result.last_component_mono_time_ns == 150
  assert result.assembly_close_mono_time_ns == 200


def test_equal_or_increasing_cycle_timestamps_are_valid() -> None:
  assembler = ARS408CycleAssembler()
  assembler.push(ObjectStatus(1, 1, 1), 100)
  assembler.push(general(1), 100)
  assembler.push(quality(1), 100)
  result = close(assembler, timestamp_ns=100)
  assert result is not None and result.exact and not result.invalid and result.timestamp_valid


@pytest.mark.parametrize(("timestamps", "expected_last", "expected_close"), [
  ((100, 150, 140, 200), 140, 200),  # component time rolls back
  ((100, 90, 100, 200), 100, 200),  # component precedes measurement status
  ((100, 140, 150, 149), 150, 149),  # close precedes the last component
  ((0, 10, 20, 30), 20, 30),  # missing/zero measurement epoch
])
def test_invalid_timestamp_order_marks_cycle_invalid(
  timestamps: tuple[int, int, int, int], expected_last: int, expected_close: int,
) -> None:
  status_time, general_time, quality_time, close_time = timestamps
  assembler = ARS408CycleAssembler()
  assembler.push(ObjectStatus(1, 1, 1), status_time)
  assembler.push(general(1), general_time)
  assembler.push(quality(1), quality_time)
  result = close(assembler, timestamp_ns=close_time)
  assert result is not None and not result.invalid and result.exact and not result.timestamp_valid
  assert result.measurement_mono_time_ns == status_time
  assert result.last_component_mono_time_ns == expected_last
  assert result.assembly_close_mono_time_ns == expected_close
