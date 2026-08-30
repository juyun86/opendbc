from collections import deque
from dataclasses import replace

import pytest

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_BUS, ARS408_GENERAL, ARS408_QUALITY
from opendbc.sunnypilot.car.tesla.ars408.interface import ARS408RadarInterface
from opendbc.sunnypilot.car.tesla.ars408.models import (
  ObjectGeneral, ObjectQuality, ObjectStatus, ParsedFrame, RadarStateSnapshot,
)


GENERAL_GOLDEN = bytes.fromhex("11 54 64 05 7e 20 40 94")
QUALITY_GOLDEN = bytes.fromhex("11 02 8a ff 53 e0 c8")


def radar_state() -> RadarStateSnapshot:
  return RadarStateSnapshot(False, False, False, False, False, 0, 1, True, True, 0, 250, 0, 0, 1, False, 0)


class ScriptedParser:
  def __init__(self, frames: list[ParsedFrame]) -> None:
    self.frames = deque(frames)
    self.can_valid = True

  def parse(self, _timestamp: int, _frame: tuple[int, bytes, int]) -> ParsedFrame | None:
    return self.frames.popleft()


def interface(frames: list[ParsedFrame], *, now_ns: int = 1_000) -> ARS408RadarInterface:
  cp = structs.CarParams()
  cp.radarUnavailable = False
  radar = ARS408RadarInterface(cp, structs.CarParamsSP(), clock=lambda: now_ns)
  radar.parser = ScriptedParser(frames)  # type: ignore[assignment]
  return radar


def packets(count: int, *, start_ns: int = 100) -> list[tuple[int, list[tuple[int, bytes, int]]]]:
  return [(start_ns + index, [(0x60A, b"", 1)]) for index in range(count)]


def timed_packets(*timestamps: int) -> list[tuple[int, list[tuple[int, bytes, int]]]]:
  return [(timestamp, [(0x60A, b"", 1)]) for timestamp in timestamps]


def real_interface(*, now_ns: int = 1_000) -> ARS408RadarInterface:
  cp = structs.CarParams()
  cp.radarUnavailable = False
  return ARS408RadarInterface(cp, structs.CarParamsSP(), clock=lambda: now_ns)


def object_cycle(raw_id: int = 7, counter: int = 1) -> list[ParsedFrame]:
  return [
    ObjectStatus(1, counter, 1), ObjectGeneral(raw_id, 40.0, -1.5, -2.0, 0.2, 4.0, 0),
    ObjectQuality(raw_id, 4, 2), ObjectStatus(0, counter + 1, 1),
  ]


def test_waits_for_radar_state_before_publishing_standard_points() -> None:
  frames = object_cycle()
  result = interface(frames).update(packets(len(frames)))
  assert result is not None and list(result.points) == []

  result = interface([radar_state(), *frames]).update(packets(len(frames) + 1))
  assert result is not None and len(result.points) == 1
  point = result.points[0]
  assert point.trackId == 7
  assert (point.dRel, point.yRel, point.vRel, point.deprecated.yvRel, point.deprecated.aRel) == pytest.approx(
    (40.0, -1.5, -2.0, 0.2, 0.0),
  )
  assert point.deprecated.measured


def test_internal_metadata_does_not_require_schema_fields() -> None:
  frames: list[ParsedFrame] = [radar_state(), *object_cycle(raw_id=9)]
  radar = interface(frames)
  result = radar.update(packets(len(frames)))
  assert result is not None and result.points[0].trackId == 9
  assert radar.tracker._tracks[9].obj.quality.probability == 4
  assert not hasattr(result.points[0], "objectClass")


def test_radar_state_faults_map_to_standard_errors() -> None:
  faulty = replace(radar_state(), persistent_error=True)
  frames: list[ParsedFrame] = [faulty, ObjectStatus(0, 1, 1), ObjectStatus(0, 2, 1)]
  result = interface(frames).update(packets(len(frames)))
  assert result is not None and result.errors.radarFault


@pytest.mark.parametrize("measurement_time_ns", [0, 2_000])
def test_shadow_timestamp_metadata_does_not_change_standard_radar(measurement_time_ns: int) -> None:
  frames: list[ParsedFrame] = [radar_state(), *object_cycle()]
  timestamps = (10, measurement_time_ns, measurement_time_ns + 1,
                measurement_time_ns + 2, measurement_time_ns + 3)
  result = interface(frames, now_ns=1_000).update(timed_packets(*timestamps))
  assert result is not None and len(result.points) == 1 and result.points[0].trackId == 7


def test_second_cycle_timestamp_regression_does_not_change_standard_radar() -> None:
  frames: list[ParsedFrame] = [
    radar_state(), ObjectStatus(1, 1, 1),
    ObjectGeneral(7, 40.0, -1.5, -2.0, 0.2, 4.0, 0), ObjectQuality(7, 4, 2),
    ObjectStatus(1, 2, 1),
    ObjectGeneral(7, 39.8, -1.5, -2.0, 0.2, 4.0, 0), ObjectQuality(7, 4, 2),
    ObjectStatus(0, 3, 1),
  ]
  baseline = interface(frames.copy(), now_ns=1_000).update(timed_packets(10, 100, 110, 120, 200, 210, 220, 230))
  regressed = interface(frames.copy(), now_ns=1_000).update(timed_packets(10, 100, 110, 120, 200, 210, 220, 190))
  assert baseline is not None and regressed is not None
  assert [point.to_dict() for point in regressed.points] == [point.to_dict() for point in baseline.points]
  assert regressed.errors.to_dict() == baseline.errors.to_dict()


def test_standard_interface_has_no_shadow_producer_state() -> None:
  radar = interface([])
  assert not hasattr(radar, "perception_tracker")
  assert not hasattr(radar, "perception_state")
  assert not hasattr(radar, "perception_snapshot")


def test_standard_radar_data_golden_none_cadence_order_fields_and_errors() -> None:
  radar = interface([])
  assert radar.update([]) is None
  frames = [radar_state(), *object_cycle()]
  radar.parser = ScriptedParser(frames)  # type: ignore[assignment]
  result = radar.update(packets(len(frames)))
  assert result is not None
  assert result.to_dict() == {
    "deprecated": {},
    "points": [{
      "trackId": 7, "dRel": 40.0, "yRel": -1.5, "vRel": -2.0,
      "deprecated": {"aRel": 0.0, "yvRel": pytest.approx(0.2), "measured": True},
    }],
    "errors": {
      "canError": False, "radarFault": False, "wrongConfig": False,
      "radarUnavailableTemporary": False,
    },
  }


def test_fixed_raw_lateral_values_reach_standard_radar_data_with_documented_transform() -> None:
  packer = CANPacker("ARS408")
  state = packer.make_can_msg("RadarState", ARS408_BUS, {
    "RadarState_MaxDistanceCfg": 250, "RadarState_SensorID": 0, "RadarState_OutputTypeCfg": 1,
    "RadarState_SendQualityCfg": 1, "RadarState_SendExtInfoCfg": 1, "RadarState_MotionRxState": 0,
  })
  status = packer.make_can_msg("Obj_0_Status", ARS408_BUS, {
    "Obj_NofObjects": 1, "Obj_MeasCounter": 1, "Obj_InterfaceVersion": 1,
  })
  close_status = packer.make_can_msg("Obj_0_Status", ARS408_BUS, {
    "Obj_NofObjects": 0, "Obj_MeasCounter": 2, "Obj_InterfaceVersion": 1,
  })
  frames = [state, status, (ARS408_GENERAL, GENERAL_GOLDEN, ARS408_BUS),
            (ARS408_QUALITY, QUALITY_GOLDEN, ARS408_BUS), close_status]
  result = real_interface().update([(index * 10 + 1, [frame]) for index, frame in enumerate(frames)])
  assert result is not None and len(result.points) == 1
  point = result.points[0]
  assert point.trackId == 17
  assert point.yRel == pytest.approx(-1.2)
  assert point.deprecated.yvRel == pytest.approx(-0.5)


def test_reserved_quality_frame_keeps_standard_cycle_incomplete() -> None:
  packer = CANPacker("ARS408")
  state = packer.make_can_msg("RadarState", ARS408_BUS, {
    "RadarState_MaxDistanceCfg": 250, "RadarState_SensorID": 0, "RadarState_OutputTypeCfg": 1,
    "RadarState_SendQualityCfg": 1, "RadarState_SendExtInfoCfg": 1, "RadarState_MotionRxState": 0,
  })
  status = packer.make_can_msg("Obj_0_Status", ARS408_BUS, {
    "Obj_NofObjects": 1, "Obj_MeasCounter": 1, "Obj_InterfaceVersion": 1,
  })
  close_status = packer.make_can_msg("Obj_0_Status", ARS408_BUS, {
    "Obj_NofObjects": 0, "Obj_MeasCounter": 2, "Obj_InterfaceVersion": 1,
  })
  reserved_quality = (ARS408_QUALITY, bytes.fromhex("01 00 00 00 00 00 98"), ARS408_BUS)
  radar = real_interface()
  frames = [state, status, (ARS408_GENERAL, GENERAL_GOLDEN, ARS408_BUS), reserved_quality, close_status]
  result = radar.update([(index * 10 + 1, [frame]) for index, frame in enumerate(frames)])
  assert result is not None and list(result.points) == []
  assert radar.parser.rejected_frames == 1
  assert radar.diagnostics.partial_cycles == 1
