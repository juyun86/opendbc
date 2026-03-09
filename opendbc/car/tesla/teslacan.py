from opendbc.car import DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams, TeslaFlags


SCCM_LEFT_STALK_MAGIC_BYTES = (0x9B, 0xE8, 0x2A, 0xD3, 0xD3, 0x83, 0x4C, 0x5E,
                               0x3F, 0x5E, 0xE2, 0x28, 0x3A, 0x13, 0xAF, 0xCE)
SCCM_LEFT_STALK_ADDRESS = 0x249
SCCM_TURN_IDLE = 0
SCCM_TURN_RIGHT = 2
SCCM_TURN_LEFT = 6


def _crc8_opensafety(data: bytes) -> int:
  crc = 0
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x2F) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc


def create_sccm_left_stalk(turn_state: int, counter: int) -> CanData:
  """Build the observed 4-byte Model 3/Y left-stalk frame for a validation pulse."""
  if turn_state not in (SCCM_TURN_IDLE, SCCM_TURN_RIGHT, SCCM_TURN_LEFT):
    raise ValueError(f"unvalidated SCCM turn state: {turn_state}")
  if not 0 <= counter <= 15:
    raise ValueError(f"invalid SCCM counter: {counter}")

  data = bytearray(4)
  data[1] = counter
  data[2] = turn_state
  crc_payload = bytes((data[1] & 0xF0, data[2], data[3], 0))
  data[0] = _crc8_opensafety(crc_payload) ^ SCCM_LEFT_STALK_MAGIC_BYTES[counter]
  return CanData(SCCM_LEFT_STALK_ADDRESS, bytes(data), CANBUS.vehicle)


def get_steer_ctrl_type(flags: int, ctrl_type: int) -> int:
  # Returns the flipped signal value for DAS_steeringControlType on FSD 14
  if flags & TeslaFlags.FSD_14:
    return {1: 2, 2: 1}.get(ctrl_type, ctrl_type)
  else:
    return ctrl_type


class TeslaCAN:
  def __init__(self, CP, packer):
    self.CP = CP
    self.packer = packer
    self.jerk = 0.0

  def create_steering_control(self, angle, enabled, control_type=1):
    # On FSD 14+, ANGLE_CONTROL behavior changed to allow user winddown while actuating.
    # with openpilot, after overriding w/ ANGLE_CONTROL the wheel snaps back to the original angle abruptly
    # so we now use LANE_KEEP_ASSIST to match stock FSD.
    # see carstate.py for more details
    values = {
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": get_steer_ctrl_type(self.CP.flags, control_type if enabled else 0),
    }

    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_stock_lateral_handoff(self, steering_angle):
    values = {
      "DAS_steeringAngleRequest": -steering_angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 3,  # Internal handoff marker; panda safety blocks this frame.
    }
    return self.packer.make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active, cruise_override):
    set_speed = min(max(v_ego + accel, 0) * CV.MS_TO_KPH, 400)

    # ramping max jerk fixes jerkiness after gas override when above max speed
    self.jerk = 0 if cruise_override else (self.jerk + CarControllerParams.JERK_RATE_UP * DT_CTRL * 4)

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": min(self.jerk, CarControllerParams.JERK_LIMIT_MAX), # ramping max jerk is enough for some reason
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_stock_longitudinal_handoff(self, das_control):
    values = {
      "DAS_setSpeed": das_control["DAS_setSpeed"],
      "DAS_accState": das_control["DAS_accState"],
      "DAS_aebEvent": 3,  # Internal handoff marker; panda safety blocks this frame.
      "DAS_jerkMin": das_control["DAS_jerkMin"],
      "DAS_jerkMax": das_control["DAS_jerkMax"],
      "DAS_accelMin": das_control["DAS_accelMin"],
      "DAS_accelMax": das_control["DAS_accelMax"],
      "DAS_controlCounter": das_control["DAS_controlCounter"],
    }
    return self.packer.make_can_msg("DAS_control", CANBUS.party, values)

  def create_steering_allowed(self):
    values = {
      "APS_eacAllow": 1,
    }

    return self.packer.make_can_msg("APS_eacMonitor", CANBUS.party, values)


def tesla_checksum(address: int, sig, d: bytearray) -> int:
  checksum = (address & 0xFF) + ((address >> 8) & 0xFF)
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum += d[i]
  return checksum & 0xFF
