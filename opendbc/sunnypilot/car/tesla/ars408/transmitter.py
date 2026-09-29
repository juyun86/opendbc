import math

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_BUS, ARS408_DBC, ARS408_MOUNT_INVERTED, FILTER_SIGNAL_SPECS
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


MOTION_TX_INTERVAL_FRAMES = 5
MAX_SPEED_MPS = 85.0
MAX_ABS_YAW_RATE_DEG_S = 100.0
MOTION_MAX_AGE_NS = 100_000_000


class ARS408Transmitter:
  def __init__(self, CP_SP: structs.CarParamsSP, *, mount_inverted: bool = ARS408_MOUNT_INVERTED) -> None:
    self.enabled = bool(CP_SP.flags & TeslaFlagsSP.ARS408_RADAR)
    self.yaw_sign = 1.0 if mount_inverted else -1.0
    self.packer = CANPacker(ARS408_DBC)
    self.yaw_rate_rad_s: float | None = None
    self.yaw_rate_timestamp_ns = 0

  def update_yaw_rate(self, yaw_rate_rad_s: float | None, timestamp_ns: int) -> None:
    # Calibrated deviceMotion input supplied by card; CarState.yawRate has no
    # source on Tesla. None explicitly invalidates a previously good sample.
    self.yaw_rate_rad_s = yaw_rate_rad_s
    self.yaw_rate_timestamp_ns = timestamp_ns

  def yaw_rate_valid(self, now_nanos: int) -> bool:
    return (self.yaw_rate_rad_s is not None and math.isfinite(self.yaw_rate_rad_s) and
            abs(math.degrees(self.yaw_rate_rad_s)) <= MAX_ABS_YAW_RATE_DEG_S and self.yaw_rate_timestamp_ns > 0 and
            0 <= now_nanos - self.yaw_rate_timestamp_ns <= MOTION_MAX_AGE_NS)

  def encode_radar_configuration(self, field: str, value: int | bool) -> CanData:
    values: dict[str, int] = {}
    if field == "max_distance":
      distance = int(value)
      if distance < 200 or distance > 250 or distance % 2:
        raise ValueError("ARS408 maximum distance must be even and within 200..250 m")
      values = {"RadarCfg_MaxDistance_valid": 1, "RadarCfg_MaxDistance": distance}
    elif field == "send_extended":
      enabled = self._binary(value, field)
      values = {"RadarCfg_SendExtInfo_valid": 1, "RadarCfg_SendExtInfo": enabled}
    elif field == "output_type":
      output_type = int(value)
      if output_type not in (0, 1):
        raise ValueError("ARS408 output type must be disabled or Objects")
      values = {"RadarCfg_OutputType_valid": 1, "RadarCfg_OutputType": output_type}
    else:
      raise ValueError(f"unsupported ARS408 configuration field: {field}")
    return self.packer.make_can_msg("RadarConfiguration", ARS408_BUS, values)

  def encode_filter_configuration(self, index: int, active: bool, minimum: float, maximum: float) -> CanData:
    if index not in FILTER_SIGNAL_SPECS:
      raise ValueError(f"unsupported ARS408 filter index: {index}")
    active_value = self._binary(active, "active")
    minimum, maximum = float(minimum), float(maximum)
    if not math.isfinite(minimum) or not math.isfinite(maximum):
      raise ValueError("ARS408 filter bounds must be finite")
    spec = FILTER_SIGNAL_SPECS[index]
    if index == 0:
      minimum = 0.0
    if not (spec.lower <= minimum <= maximum <= spec.upper):
      raise ValueError(f"invalid ARS408 filter range: {minimum}..{maximum}")
    raw_min = self._filter_raw(minimum, spec.resolution, spec.offset, spec.bits)
    raw_max = self._filter_raw(maximum, spec.resolution, spec.offset, spec.bits)
    return self.packer.make_can_msg("FilterCfg", ARS408_BUS, {
      "FilterCfg_Type": 1, "FilterCfg_Index": index, "FilterCfg_Active": active_value,
      "FilterCfg_Valid": 1, "FilterCfg_MinRaw": raw_min, "FilterCfg_MaxRaw": raw_max,
    })

  def encode_filter_query(self, index: int) -> CanData:
    if index not in FILTER_SIGNAL_SPECS:
      raise ValueError(f"unsupported ARS408 filter index: {index}")
    return self.packer.make_can_msg("FilterCfg", ARS408_BUS, {
      "FilterCfg_Type": 1, "FilterCfg_Index": index, "FilterCfg_Active": 0, "FilterCfg_Valid": 0,
    })

  def encode_speed(self, speed_mps: float, direction: int) -> CanData:
    speed_mps = float(speed_mps)
    if not math.isfinite(speed_mps) or not 0.0 <= speed_mps <= MAX_SPEED_MPS:
      raise ValueError(f"ARS408 speed is outside 0..{MAX_SPEED_MPS} m/s")
    if direction not in (0, 1, 2):
      raise ValueError("ARS408 speed direction must be standstill, forward, or reverse")
    return self.packer.make_can_msg("SpeedInformation", ARS408_BUS, {
      "RadarDevice_SpeedDirection": direction, "RadarDevice_Speed": speed_mps,
    })

  def encode_yaw_rate(self, yaw_rate_deg_s: float) -> CanData:
    yaw_rate_deg_s = float(yaw_rate_deg_s)
    if not math.isfinite(yaw_rate_deg_s) or abs(yaw_rate_deg_s) > MAX_ABS_YAW_RATE_DEG_S:
      raise ValueError(f"ARS408 yaw rate is outside +/-{MAX_ABS_YAW_RATE_DEG_S} deg/s")
    return self.packer.make_can_msg("YawRateInformation", ARS408_BUS, {"RadarDevice_YawRate": yaw_rate_deg_s})

  def update(self, frame: int, car_state: structs.CarState, now_nanos: int) -> list[CanData]:
    if not self.enabled or frame < 0 or frame % MOTION_TX_INTERVAL_FRAMES or not car_state.canValid:
      return []
    if not self.yaw_rate_valid(now_nanos):
      return []
    speed_mps = abs(float(car_state.vEgoRaw))
    if not math.isfinite(speed_mps) or speed_mps > MAX_SPEED_MPS:
      return []
    if car_state.standstill or speed_mps <= 0.01:
      # The stationary direction requires zero speed, including residual wheel speed.
      speed_mps = 0.0
      direction = 0
    elif car_state.gearShifter == structs.CarState.GearShifter.reverse:
      direction = 2
    else:
      direction = 1
    try:
      # Calibrated frame is forward/right/down (right turn positive).
      # Upright ARS408 requires left-positive degrees/s; inverted mounting
      # reverses its vertical axis and therefore the input yaw sign as well.
      return [self.encode_speed(speed_mps, direction), self.encode_yaw_rate(self.yaw_sign * math.degrees(self.yaw_rate_rad_s))]
    except ValueError:
      return []

  @staticmethod
  def _binary(value: int | bool, field: str) -> int:
    converted = int(value)
    if converted not in (0, 1):
      raise ValueError(f"ARS408 {field} must be 0 or 1")
    return converted

  @staticmethod
  def _filter_raw(value: float, resolution: float, offset: float, bits: int) -> int:
    raw = round((value - offset) / resolution)
    if not 0 <= raw < 1 << bits:
      raise ValueError("ARS408 filter value is not representable")
    decoded = raw * resolution + offset
    if abs(decoded - value) > resolution / 2.0 + 1e-9:
      raise ValueError("ARS408 filter value loses excessive precision")
    return raw
