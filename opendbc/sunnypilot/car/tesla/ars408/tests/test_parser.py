import math

import pytest

from opendbc.can import CANPacker
from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_BUS, ARS408_GENERAL, ARS408_STATUS
from opendbc.sunnypilot.car.tesla.ars408.models import (
  FilterStateRecord, ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus, RadarStateSnapshot, SoftwareVersion,
)
from opendbc.sunnypilot.car.tesla.ars408.parser import ARS408Parser


@pytest.fixture
def packer() -> CANPacker:
  return CANPacker("ARS408")


def test_runtime_parser_preserves_tracking_fields_without_changing_shared_dbc(packer):
  full = ARS408Parser()
  runtime = ARS408Parser(diagnostic_fields=False)
  later_full = ARS408Parser()
  quality = packer.make_can_msg("Obj_2_Quality", ARS408_BUS, {
    "Obj_ID": 7, "Obj_ProbOfExist": 6, "Obj_MeasState": 2, "Obj_DistLong_rms": 15,
  })
  extended = packer.make_can_msg("Obj_3_Extended", ARS408_BUS, {
    "Obj_ID": 7, "Obj_ArelLong": 0.2, "Obj_Class": 1, "Obj_Length": 4.2,
  })
  for stamp in (1, 2):
    a, b = full.parse(stamp, quality), runtime.parse(stamp, quality)
    assert (a.raw_id, a.probability, a.measurement_state) == (b.raw_id, b.probability, b.measurement_state)
    assert a.dist_long_rms == 15 and b.dist_long_rms is None
    assert later_full.parse(stamp, quality) == a
    a, b = full.parse(stamp, extended), runtime.parse(stamp, extended)
    assert (a.raw_id, a.a_rel, a.object_class) == (b.raw_id, b.a_rel, b.object_class)
    assert a.length_m == pytest.approx(4.2) and b.length_m is None
    assert later_full.parse(stamp, extended) == a


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


@pytest.mark.parametrize("inverted", [False, True])
@pytest.mark.parametrize("vehicle_left_m", [-3.0, 3.0])
def test_same_physical_target_has_same_position_and_velocity_for_both_mounts(packer, inverted, vehicle_left_m):
  sensor_sign = -1 if inverted else 1
  general = packer.make_can_msg("Obj_1_General", ARS408_BUS, {
    "Obj_ID": 7, "Obj_DistLong": 40, "Obj_DistLat": sensor_sign * vehicle_left_m,
    "Obj_VrelLong": -2, "Obj_VrelLat": sensor_sign * 0.5,
  })
  parsed = ARS408Parser(mount_inverted=inverted).parse(1, general)
  assert isinstance(parsed, ObjectGeneral)
  assert parsed.y_rel == pytest.approx(vehicle_left_m)
  assert parsed.yv_rel == pytest.approx(0.5)
  assert parsed.d_rel == pytest.approx(40)
  assert parsed.v_rel == pytest.approx(-2)


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


@pytest.mark.parametrize(("payload", "expected"), [
  # Manufacturer Fig. 13/14: low 12/13 bits, independent of CANPacker.
  ("8c006407d0", FilterStateRecord(1, True, 10.0, 200.0)),
  ("d409c41388", FilterStateRecord(10, True, 0.0, 500.0)),
  ("c400030007", FilterStateRecord(8, True, 3.0, 7.0)),
  ("840fff0064", FilterStateRecord(0, True, 0.0, 100.0)),
  ("88f064f7d0", FilterStateRecord(1, False, 10.0, 200.0)),
])
def test_filter_state_wire_layout(payload: str, expected: FilterStateRecord) -> None:
  assert ARS408Parser().parse(0, (0x204, bytes.fromhex(payload), ARS408_BUS)) == expected


@pytest.mark.parametrize("payload", ["0c006407d0", "fc00000001", "8c07d00064", "c400080009"])
def test_filter_state_rejects_unsupported_or_invalid_records(payload: str) -> None:
  assert ARS408Parser().parse(0, (0x204, bytes.fromhex(payload), ARS408_BUS)) is None


def test_captured_quality_retains_raw_rms_indices() -> None:
  # CAN1 capture 2026-09-24; independent of the local packer.
  quality = ARS408Parser().parse(0, (0x60C, bytes.fromhex("2e84673a8220e8"), ARS408_BUS))
  assert quality == ObjectQuality(46, 7, 2, 16, 17, 19, 19, 21, 0, 17)
  invalid_rms = ARS408Parser().parse(0, (0x60C, bytes.fromhex("01ffffffffffff"), ARS408_BUS))
  assert invalid_rms == ObjectQuality(1, 7, 7, 31, 31, 31, 31, 31, 31, 31)


@pytest.mark.parametrize(("payload", "expected"), [
  ("2780cfa470400a05", (39, 0.3, 4, 0.0, -0.4, 2.0, 1.0)),
  # Independent asymmetric wire vector exercises signed offsets and bit boundaries.
  ("097d09617d001709", (9, 0.0, 1, -1.0, 20.0, 4.6, 1.8)),
])
def test_extended_wire_fields(payload: str, expected: tuple) -> None:
  extended = ARS408Parser().parse(0, (0x60D, bytes.fromhex(payload), ARS408_BUS))
  assert isinstance(extended, ObjectExtended)
  assert (extended.raw_id, extended.a_rel, extended.object_class, extended.sensor_a_rel_lat,
          extended.sensor_orientation_deg, extended.length_m, extended.width_m) == pytest.approx(expected)


def test_captured_software_version_is_binary_integer_not_bcd() -> None:
  parser = ARS408Parser()
  assert parser.parse(0, (0x700, bytes.fromhex("04220100"), ARS408_BUS)) == SoftwareVersion(4, 34, 1, False, 0)
  assert parser.parse(1, (0x700, bytes.fromhex("04220103"), ARS408_BUS)) == SoftwareVersion(4, 34, 1, True, 1)
  assert parser.parse(2, (0x700, bytes.fromhex("042201"), ARS408_BUS)) is None
