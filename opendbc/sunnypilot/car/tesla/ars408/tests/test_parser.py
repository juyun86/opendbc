import math

import pytest

from opendbc.can import CANPacker
from opendbc.sunnypilot.car.tesla.ars408.constants import (
  ARS408_BUS, ARS408_EXTENDED, ARS408_GENERAL, ARS408_QUALITY, ARS408_STATUS, LATERAL_KINEMATICS_VERIFIED,
  RMS_INVALID_CODE, RMS_LINEAR_UPPER_BOUNDS, RMS_ORIENTATION_UPPER_BOUNDS_DEG,
)
from opendbc.sunnypilot.car.tesla.ars408.models import (
  ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus, RadarStateSnapshot,
)
from opendbc.sunnypilot.car.tesla.ars408.parser import ARS408Parser


GENERAL_GOLDEN = bytes.fromhex("11 54 64 05 7e 20 40 94")
QUALITY_GOLDEN = bytes.fromhex("11 02 8a ff 53 e0 c8")
EXTENDED_GOLDEN = bytes.fromhex("11 6d 71 d3 64 00 09 04")


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


def test_fixed_raw_general_reflects_all_lateral_kinematics() -> None:
  parser = ARS408Parser()
  parsed = parser.parse(0, (ARS408_GENERAL, GENERAL_GOLDEN, ARS408_BUS))

  assert isinstance(parsed, ObjectGeneral)
  assert parsed.raw_id == 17
  assert parsed.y_rel == pytest.approx(-1.2)
  assert parsed.yv_rel == pytest.approx(-0.5)
  assert LATERAL_KINEMATICS_VERIFIED is False


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


def test_parser_decodes_full_quality_and_extended_protocol_fields(packer: CANPacker) -> None:
  parser = ARS408Parser()
  quality_frame = packer.make_can_msg("Obj_2_Quality", ARS408_BUS, {
    "Obj_ID": 17, "Obj_DistLong_rms": 0, "Obj_VrelLong_rms": 5, "Obj_DistLat_rms": 10,
    "Obj_VrelLat_rms": 15, "Obj_ArelLat_rms": 20, "Obj_ArelLong_rms": 30,
    "Obj_Orientation_rms": RMS_INVALID_CODE, "Obj_MeasState": 2, "Obj_ProbOfExist": 6,
  })
  assert quality_frame == (ARS408_QUALITY, QUALITY_GOLDEN, ARS408_BUS)
  quality = parser.parse(1, (ARS408_QUALITY, QUALITY_GOLDEN, ARS408_BUS))
  assert isinstance(quality, ObjectQuality)
  assert quality.rms_codes == (0, 5, 10, 15, 20, 30, RMS_INVALID_CODE)
  assert quality.rms_upper_bounds == (
    RMS_LINEAR_UPPER_BOUNDS[0], RMS_LINEAR_UPPER_BOUNDS[5], RMS_LINEAR_UPPER_BOUNDS[10],
    RMS_LINEAR_UPPER_BOUNDS[15], RMS_LINEAR_UPPER_BOUNDS[20], RMS_LINEAR_UPPER_BOUNDS[30], None,
  )
  assert quality.rms_sigmas == quality.rms_upper_bounds

  extended_frame = packer.make_can_msg("Obj_3_Extended", ARS408_BUS, {
    "Obj_ID": 17, "Obj_ArelLong": -1.25, "Obj_ArelLat": 0.35, "Obj_Class": 3,
    "Obj_OrientationAngle": -20.0, "Obj_Length": 1.8, "Obj_Width": 0.8,
  })
  assert extended_frame == (ARS408_EXTENDED, EXTENDED_GOLDEN, ARS408_BUS)
  extended = parser.parse(2, (ARS408_EXTENDED, EXTENDED_GOLDEN, ARS408_BUS))
  assert isinstance(extended, ObjectExtended)
  assert (extended.a_rel, extended.a_rel_lat, extended.orientation_angle,
          extended.length, extended.width) == pytest.approx((-1.25, -0.35, 20.0, 1.8, 0.8))
  assert extended.object_class == 3


def test_protocol_rms_tables_are_exact_upper_bounds() -> None:
  assert RMS_LINEAR_UPPER_BOUNDS == (
    0.005, 0.006, 0.008, 0.011, 0.014, 0.018, 0.023, 0.029,
    0.038, 0.049, 0.063, 0.081, 0.105, 0.135, 0.174, 0.224,
    0.288, 0.371, 0.478, 0.616, 0.794, 1.023, 1.317, 1.697,
    2.187, 2.817, 3.630, 4.676, 6.025, 7.762, 10.000,
  )
  assert RMS_ORIENTATION_UPPER_BOUNDS_DEG == (
    0.005, 0.007, 0.010, 0.014, 0.020, 0.029, 0.041, 0.058,
    0.082, 0.116, 0.165, 0.234, 0.332, 0.471, 0.669, 0.949,
    1.346, 1.909, 2.709, 3.843, 5.451, 7.734, 10.971, 15.565,
    22.081, 31.325, 44.439, 63.044, 89.437, 126.881, 180.000,
  )
  assert len(RMS_LINEAR_UPPER_BOUNDS) == RMS_INVALID_CODE
  assert len(RMS_ORIENTATION_UPPER_BOUNDS_DEG) == RMS_INVALID_CODE


@pytest.mark.parametrize("state,payload", [
  (6, bytes.fromhex("01 00 00 00 00 00 98")),
  (7, bytes.fromhex("01 00 00 00 00 00 9c")),
])
def test_parser_safely_rejects_reserved_measurement_states(state: int, payload: bytes) -> None:
  parser = ARS408Parser()
  assert parser.parse(0, (ARS408_QUALITY, payload, ARS408_BUS)) is None, state
  assert parser.rejected_frames == 1


@pytest.mark.parametrize("payload,expected_orientation", [
  (bytes.fromhex("01 7d 0f a0 00 00 00 00"), 180.0),
  (bytes.fromhex("01 7d 0f a0 e1 00 00 00"), -180.0),
])
def test_parser_accepts_official_orientation_boundaries(payload: bytes, expected_orientation: float) -> None:
  parser = ARS408Parser()
  parsed = parser.parse(0, (ARS408_EXTENDED, payload, ARS408_BUS))
  assert isinstance(parsed, ObjectExtended)
  assert parsed.orientation_angle == pytest.approx(expected_orientation)


@pytest.mark.parametrize("payload,expected_acceleration", [
  (bytes.fromhex("01 00 00 00 70 80 00 00"), (-10.0, 2.5)),
  (bytes.fromhex("01 ff ff f0 70 80 00 00"), (10.47, -2.61)),
])
def test_parser_accepts_protocol_acceleration_boundaries(payload: bytes, expected_acceleration: tuple[float, float]) -> None:
  parser = ARS408Parser()
  parsed = parser.parse(0, (ARS408_EXTENDED, payload, ARS408_BUS))
  assert isinstance(parsed, ObjectExtended)
  assert (parsed.a_rel, parsed.a_rel_lat) == pytest.approx(expected_acceleration)


def test_parser_safely_rejects_out_of_range_orientation() -> None:
  parser = ARS408Parser()
  # Orientation raw 901 decodes to +180.4 degrees, outside the official range.
  payload = bytes.fromhex("01 7d 0f a0 e1 40 00 00")
  assert parser.parse(0, (ARS408_EXTENDED, payload, ARS408_BUS)) is None
  assert parser.rejected_frames == 1


def test_extended_model_enforces_protocol_acceleration_ranges() -> None:
  with pytest.raises(ValueError, match="longitudinal acceleration"):
    ObjectExtended(1, 10.48, 0)
  with pytest.raises(ValueError, match="lateral acceleration"):
    ObjectExtended(1, 0.0, 0, a_rel_lat=-2.62)


def test_internal_model_rejects_non_finite_values() -> None:
  with pytest.raises(ValueError, match="finite"):
    ObjectGeneral(1, math.nan, 0.0, 0.0, 0.0, 0.0, 0)


def test_protocol_constants_are_sensor_zero_base_addresses() -> None:
  assert ARS408_STATUS == 0x60A
  assert ARS408_GENERAL == 0x60B
