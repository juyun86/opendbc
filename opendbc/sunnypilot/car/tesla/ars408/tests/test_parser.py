import math

import pytest

from opendbc.can import CANPacker
from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_BUS, ARS408_GENERAL, ARS408_STATUS
from opendbc.sunnypilot.car.tesla.ars408.models import ObjectGeneral, ObjectStatus, RadarStateSnapshot
from opendbc.sunnypilot.car.tesla.ars408.parser import ARS408Parser


@pytest.fixture
def packer() -> CANPacker:
  return CANPacker("ARS408")


def test_parser_decodes_sensor_zero_object_frames(packer: CANPacker) -> None:
  parser = ARS408Parser()
  status = packer.make_can_msg("Obj_0_Status", ARS408_BUS, {
    "Obj_NofObjects": 1, "Obj_MeasCounter": 42, "Obj_InterfaceVersion": 1,
  })
  general = packer.make_can_msg("Obj_1_General", ARS408_BUS, {
    "Obj_ID": 7, "Obj_DistLong": 40.0, "Obj_DistLat": -1.2, "Obj_VrelLong": -2.0,
    "Obj_VrelLat": 0.5, "Obj_DynProp": 0, "Obj_RCS": 10.0,
  })

  assert parser.parse(1, status) == ObjectStatus(1, 42, 1)
  parsed = parser.parse(2, general)
  assert isinstance(parsed, ObjectGeneral)
  assert parsed.raw_id == 7
  assert parsed.d_rel == pytest.approx(40.0)
  assert parsed.y_rel == pytest.approx(1.2)
  assert parsed.v_rel == pytest.approx(-2.0)
  assert parsed.yv_rel == pytest.approx(-0.5)


def test_parser_rejects_wrong_bus_dlc_and_non_protocol_address(packer: CANPacker) -> None:
  parser = ARS408Parser()
  valid = packer.make_can_msg("Obj_0_Status", ARS408_BUS, {"Obj_InterfaceVersion": 1})
  assert parser.parse(0, (ARS408_STATUS, valid[1], 0)) is None
  assert parser.parse(0, (ARS408_STATUS, valid[1][:-1], ARS408_BUS)) is None
  assert parser.parse(0, (0x777, b"\x00" * 8, ARS408_BUS)) is None
  assert parser.rejected_frames == 3


def test_parser_decodes_radar_state(packer: CANPacker) -> None:
  parser = ARS408Parser()
  frame = packer.make_can_msg("RadarState", ARS408_BUS, {
    "RadarState_MaxDistanceCfg": 250, "RadarState_SensorID": 0, "RadarState_OutputTypeCfg": 1,
    "RadarState_SendQualityCfg": 1, "RadarState_SendExtInfoCfg": 1, "RadarState_MotionRxState": 0,
    "RadarState_SortIndex": 1,
  })
  state = parser.parse(0, frame)
  assert isinstance(state, RadarStateSnapshot)
  assert state.sensor_id == 0
  assert state.max_distance_m == 250
  assert state.output_type == 1
  assert state.quality_enabled


def test_internal_model_rejects_non_finite_values() -> None:
  with pytest.raises(ValueError, match="finite"):
    ObjectGeneral(1, math.nan, 0.0, 0.0, 0.0, 0.0, 0)


def test_protocol_constants_are_sensor_zero_base_addresses() -> None:
  assert ARS408_STATUS == 0x60A
  assert ARS408_GENERAL == 0x60B
