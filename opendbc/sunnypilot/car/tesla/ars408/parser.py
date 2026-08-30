import math

from opendbc.can import CANParser
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.tesla.ars408.constants import (
  ARS408_BUS, ARS408_DBC, ARS408_EXTENDED, ARS408_FILTER_STATE_CONFIG, ARS408_FILTER_STATE_HEADER,
  ARS408_GENERAL, ARS408_QUALITY, ARS408_RADAR_STATE, ARS408_RX_DLC, ARS408_STATUS, FILTER_SIGNAL_SPECS,
  SENSOR_TO_OPENPILOT_LATERAL_SIGN,
)
from opendbc.sunnypilot.car.tesla.ars408.models import (
  FilterStateHeader, FilterStateRecord, ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus, ParsedFrame,
  RadarStateSnapshot,
)


MESSAGE_NAMES: dict[int, str] = {
  ARS408_RADAR_STATE: "RadarState",
  ARS408_FILTER_STATE_HEADER: "FilterState_Header",
  ARS408_FILTER_STATE_CONFIG: "FilterState_Cfg",
  ARS408_STATUS: "Obj_0_Status",
  ARS408_GENERAL: "Obj_1_General",
  ARS408_QUALITY: "Obj_2_Quality",
  ARS408_EXTENDED: "Obj_3_Extended",
}


def _sensor_to_openpilot_lateral(value: float) -> float:
  """Reflect a sensor-frame lateral scalar into the openpilot frame."""
  return SENSOR_TO_OPENPILOT_LATERAL_SIGN * value


class ARS408Parser:
  def __init__(self) -> None:
    self.can_parser = CANParser(ARS408_DBC, [(name, math.nan) for name in MESSAGE_NAMES.values()], ARS408_BUS)
    self.rejected_frames = 0

  @property
  def can_valid(self) -> bool:
    return bool(self.can_parser.can_valid)

  def parse(self, timestamp: int, frame: CanData) -> ParsedFrame | None:
    address, data, bus = frame
    if bus != ARS408_BUS or ARS408_RX_DLC.get(address) != len(data):
      self.rejected_frames += 1
      return None

    self.can_parser.update([(timestamp, [frame])])
    values = self.can_parser.vl[MESSAGE_NAMES[address]]

    try:
      return self._parse_values(address, data, values)
    except (OverflowError, ValueError):
      # The DBC covers the complete raw bit range, while the ARS408 protocol
      # reserves some decoded values. Reject those frames without taking down
      # the radar interface.
      self.rejected_frames += 1
      return None

  @staticmethod
  def _parse_values(address: int, data: bytes, values: dict[str, float]) -> ParsedFrame | None:
    if address == ARS408_STATUS:
      return ObjectStatus(int(values["Obj_NofObjects"]), int(values["Obj_MeasCounter"]), int(values["Obj_InterfaceVersion"]))
    if address == ARS408_GENERAL:
      return ObjectGeneral(
        raw_id=int(values["Obj_ID"]), d_rel=float(values["Obj_DistLong"]),
        y_rel=_sensor_to_openpilot_lateral(float(values["Obj_DistLat"])), v_rel=float(values["Obj_VrelLong"]),
        yv_rel=_sensor_to_openpilot_lateral(float(values["Obj_VrelLat"])), rcs=float(values["Obj_RCS"]),
        dynamic_property=int(values["Obj_DynProp"]),
      )
    if address == ARS408_QUALITY:
      return ObjectQuality(
        raw_id=int(values["Obj_ID"]), probability=int(values["Obj_ProbOfExist"]),
        measurement_state=int(values["Obj_MeasState"]), dist_long_rms=int(values["Obj_DistLong_rms"]),
        vrel_long_rms=int(values["Obj_VrelLong_rms"]), dist_lat_rms=int(values["Obj_DistLat_rms"]),
        vrel_lat_rms=int(values["Obj_VrelLat_rms"]), arel_lat_rms=int(values["Obj_ArelLat_rms"]),
        arel_long_rms=int(values["Obj_ArelLong_rms"]), orientation_rms=int(values["Obj_Orientation_rms"]),
      )
    if address == ARS408_EXTENDED:
      return ObjectExtended(
        raw_id=int(values["Obj_ID"]), a_rel=float(values["Obj_ArelLong"]), object_class=int(values["Obj_Class"]),
        a_rel_lat=_sensor_to_openpilot_lateral(float(values["Obj_ArelLat"])),
        orientation_angle=_sensor_to_openpilot_lateral(float(values["Obj_OrientationAngle"])),
        length=float(values["Obj_Length"]), width=float(values["Obj_Width"]),
      )
    if address == ARS408_RADAR_STATE:
      return RadarStateSnapshot(
        interference=bool(values["RadarState_Interference"]), voltage_error=bool(values["RadarState_Voltage_Error"]),
        temporary_error=bool(values["RadarState_Temporary_Error"]), temperature_error=bool(values["RadarState_Temperature_Error"]),
        persistent_error=bool(values["RadarState_Persistent_Error"]), sensor_id=int(values["RadarState_SensorID"]),
        output_type=int(values["RadarState_OutputTypeCfg"]), quality_enabled=bool(values["RadarState_SendQualityCfg"]),
        extended_enabled=bool(values["RadarState_SendExtInfoCfg"]), motion_rx_state=int(values["RadarState_MotionRxState"]),
        max_distance_m=int(values["RadarState_MaxDistanceCfg"]), nvm_read_status=int(values["RadarState_NVMReadStatus"]),
        nvm_write_status=int(values["RadarState_NVMwriteStatus"]), sort_index=int(values["RadarState_SortIndex"]),
        ctrl_relay_enabled=bool(values["RadarState_CtrlRelayCfg"]), rcs_threshold=int(values["RadarState_RCS_Threshold"]),
      )
    if address == ARS408_FILTER_STATE_HEADER:
      return FilterStateHeader(int(values["FilterState_NofClusterFilterCfg"]), int(values["FilterState_NofObjectFilterCfg"]))
    return ARS408Parser._parse_filter_state(data)

  @staticmethod
  def _parse_filter_state(data: bytes) -> FilterStateRecord | None:
    filter_type = data[0] >> 7
    index = (data[0] >> 3) & 0xF
    if filter_type != 1 or index not in FILTER_SIGNAL_SPECS:
      return None
    spec = FILTER_SIGNAL_SPECS[index]
    raw_min = ((data[1] << 8) | data[2]) >> (16 - spec.bits)
    raw_max = ((data[3] << 8) | data[4]) >> (16 - spec.bits)
    minimum = raw_min * spec.resolution + spec.offset
    maximum = raw_max * spec.resolution + spec.offset
    if index == 0:
      minimum = 0.0
    if not (spec.lower <= minimum <= spec.upper and spec.lower <= maximum <= spec.upper and minimum <= maximum):
      return None
    return FilterStateRecord(index, bool(data[0] & 0x4), minimum, maximum)
