#!/usr/bin/env python3
import random
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.teslacan import get_steer_ctrl_type
from opendbc.car.tesla.values import CANBUS, CarControllerParams, STEER_DISENGAGE_THRESHOLD, TeslaSafetyFlags, TeslaFlags
from opendbc.car.tesla.carcontroller import get_safety_CP
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.can import CANDefine
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, MAX_SPEED_DELTA, MAX_WRONG_COUNTERS, away_round, round_speed

from opendbc.sunnypilot.car.tesla.values import TeslaSafetyFlagsSP

MSG_DAS_steeringControl = 0x488
MSG_APS_eacMonitor = 0x27d
MSG_DAS_Control = 0x2b9
MSG_VCLEFT_SWITCH_STATUS = 0x3C2
MSG_DAS_BODY_CONTROLS = 0x3E9
MSG_UI_TRIP_PLANNING = 0x082
MSG_UI_AUTOPILOT_CONTROL = 0x3FD
MSG_EPAS3S_SYS_STATUS = 0x370
MSG_ISA_CHIME_SUPPRESS = 0x399
MSG_ARS408_SPEED = 0x300
MSG_ARS408_YAW_RATE = 0x301
MSG_GATEWAY_CONFIG = 0x6E0
OBSERVED_SPEED_WHEEL_IDLE = bytes.fromhex("010000c000000000")
OBSERVED_BODY_CONTROLS_IDLE = bytes.fromhex("008802000000b026")


def round_angle(apply_angle, can_offset=0):
  apply_angle_can = (apply_angle + 1638.35) / 0.1 + can_offset
  # 0.49999_ == 0.5
  rnd_offset = 1e-5 if apply_angle >= 0 else -1e-5
  return away_round(apply_angle_can + rnd_offset) * 0.1 - 1638.35


class TestTeslaSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest, common.LongitudinalAccelSafetyTest):
  SAFETY_PARAM = 0

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor]}
  TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_APS_eacMonitor, 0], [MSG_DAS_Control, 0],
             [0x679, CANBUS.vehicle], [MSG_VCLEFT_SWITCH_STATUS, CANBUS.vehicle], [MSG_DAS_BODY_CONTROLS, CANBUS.vehicle],
             [MSG_UI_TRIP_PLANNING, 0], [MSG_UI_AUTOPILOT_CONTROL, 0],
             [MSG_EPAS3S_SYS_STATUS, 0], [MSG_ISA_CHIME_SUPPRESS, 0]]
  TX_MSGS += [[MSG_GATEWAY_CONFIG, CANBUS.vehicle]]

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  # Tesla uses get_max_angle_delta_vm and get_max_angle_vm for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  cnt_epas = 0
  cnt_angle_cmd = 0

  packer: CANPackerSafety

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    self.VM = VehicleModel(get_safety_CP())
    self.packer = CANPackerSafety("tesla_model3_party")
    self.packer_adas = CANPackerSafety("tesla_model3_vehicle")
    self.define = CANDefine("tesla_model3_party")
    self.acc_states = {d: v for v, d in self.define.dv["DAS_control"]["DAS_accState"].items()}
    self.autopark_states = {d: v for v, d in self.define.dv["DI_state"]["DI_autoparkState"].items()}
    self.active_autopark_states = [self.autopark_states[s] for s in ('ACTIVE', 'COMPLETE', 'SELFPARK_STARTED')]

    self.steer_control_types = {d: v for v, d in self.define.dv["DAS_steeringControl"]["DAS_steeringControlType"].items()}

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, state: bool | int, increment_timer: bool = True, bus: int = 0):
    # If FSD 14, translate steer control type to new flipped definition
    if self.safety.get_current_safety_param() & TeslaSafetyFlags.FSD_14:
      state = get_steer_ctrl_type(TeslaFlags.FSD_14, int(state))

    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": state}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values)

  def _angle_meas_msg(self, angle: float, hands_on_level: int = 0, eac_status: int = 1, eac_error_code: int = 0,
                      torsion_bar_torque: float = 0.0):
    values = {"EPAS3S_internalSAS": angle, "EPAS3S_handsOnLevel": hands_on_level,
              "EPAS3S_torsionBarTorque": torsion_bar_torque,
              "EPAS3S_eacStatus": eac_status, "EPAS3S_eacErrorCode": eac_error_code,
              "EPAS3S_sysStatusCounter": self.cnt_epas % 16}
    self.__class__.cnt_epas += 1
    return self.packer.make_can_msg_safety("EPAS3S_sysStatus", 0, values)

  def _user_brake_msg(self, brake, quality_flag: bool = True):
    values = {"ESP_driverBrakeApply": 2 if brake else 1}
    if not quality_flag:
      values["ESP_driverBrakeApply"] = random.choice((0, 3))  # NotInit_orOff, Faulty_SNA
    return self.packer.make_can_msg_safety("ESP_status", 0, values)

  def _speed_msg(self, speed):
    values = {"DI_vehicleSpeed": speed * 3.6}
    return self.packer.make_can_msg_safety("DI_speed", 0, values)

  def _speed_msg_2(self, speed, quality_flag=True):
    values = {"ESP_vehicleSpeed": speed * 3.6, "ESP_wheelSpeedsQF": quality_flag}
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _vehicle_moving_msg(self, speed: float, quality_flag=True):
    values = {"ESP_vehicleStandstillSts": 1 if speed <= self.STANDSTILL_THRESHOLD else 0,
              "ESP_wheelSpeedsQF": quality_flag}
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _user_gas_msg(self, gas):
    values = {"DI_accelPedalPressed": gas > 0}
    return self.packer.make_can_msg_safety("DI_speed", 0, values)

  def _pcm_status_msg(self, enable, autopark_state=0):
    values = {
      "DI_cruiseState": 2 if enable else 0,
      "DI_autoparkState": autopark_state,
    }
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _long_control_msg(self, set_speed, acc_state=0, jerk_limits=(0, 0), accel_limits=(0, 0), aeb_event=0, bus=0, counter=None):
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    if counter is not None:
      values["DAS_controlCounter"] = counter
    return self.packer.make_can_msg_safety("DAS_control", bus, values)

  @staticmethod
  def _speed_wheel_msg(right_ticks=0, data=OBSERVED_SPEED_WHEEL_IDLE):
    payload = bytearray(data)
    payload[3] = (payload[3] & 0xC0) | (right_ticks & 0x3F)
    return libsafety_py.make_CANPacket(MSG_VCLEFT_SWITCH_STATUS, CANBUS.vehicle, payload)

  @staticmethod
  def _body_control_msg(turn_request=0, reason=0, counter=11, data=OBSERVED_BODY_CONTROLS_IDLE):
    payload = bytearray(data)
    payload[1] = (payload[1] & 0xFC) | turn_request
    payload[2] = (payload[2] & 0xE1) | ((reason & 0xF) << 1)
    payload[6] = (payload[6] & 0x0F) | ((counter & 0xF) << 4)
    payload[7] = (0xE9 + 0x03 + sum(payload[:7])) & 0xFF
    return libsafety_py.make_CANPacket(MSG_DAS_BODY_CONTROLS, CANBUS.vehicle, payload)

  def _enable_turn_signal_validation(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.HAS_VEHICLE_BUS | TeslaSafetyFlagsSP.TURN_SIGNAL_VALIDATION)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _enable_speed_button_validation(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.HAS_VEHICLE_BUS | TeslaSafetyFlagsSP.SPEED_BUTTON_VALIDATION)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _enable_auto_speed_limit(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.HAS_VEHICLE_BUS | TeslaSafetyFlagsSP.AUTO_SPEED_LIMIT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()

  def _enable_ars408_radar(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.ARS408_RADAR)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.assertTrue(self._rx(self._pcm_status_msg(False)))

  @staticmethod
  def _ars408_speed_msg(raw_speed, direction, bus=CANBUS.vehicle, length=2, reserved=False):
    payload = bytearray(max(length, 2))
    payload[0] = ((direction & 0x3) << 6) | ((raw_speed >> 8) & 0x1F) | (0x20 if reserved else 0)
    payload[1] = raw_speed & 0xFF
    return libsafety_py.make_CANPacket(MSG_ARS408_SPEED, bus, payload[:length])

  @staticmethod
  def _ars408_yaw_msg(raw_yaw_rate, bus=CANBUS.vehicle, length=2):
    payload = bytearray(max(length, 2))
    payload[0] = (raw_yaw_rate >> 8) & 0xFF
    payload[1] = raw_yaw_rate & 0xFF
    return libsafety_py.make_CANPacket(MSG_ARS408_YAW_RATE, bus, payload[:length])

  def test_ars408_motion_requires_feature_flag(self):
    self.assertFalse(self._tx(self._ars408_speed_msg(1000, 1)))
    self.assertFalse(self._tx(self._ars408_yaw_msg(32768)))

  def test_ars408_motion_requires_exact_bus_and_dlc(self):
    self._enable_ars408_radar()
    self.assertTrue(self._tx(self._ars408_speed_msg(1000, 1)))
    self.assertTrue(self._tx(self._ars408_yaw_msg(32768)))
    for bus in (0, 2):
      self.assertFalse(self._tx(self._ars408_speed_msg(1000, 1, bus=bus)))
      self.assertFalse(self._tx(self._ars408_yaw_msg(32768, bus=bus)))
    for length in (1, 3, 8):
      self.assertFalse(self._tx(self._ars408_speed_msg(1000, 1, length=length)))
      self.assertFalse(self._tx(self._ars408_yaw_msg(32768, length=length)))

  def test_ars408_speed_fields_and_operational_range(self):
    self._enable_ars408_radar()
    for raw_speed, direction in ((0, 0), (1, 1), (4250, 2)):
      self.assertTrue(self._tx(self._ars408_speed_msg(raw_speed, direction)))
    for raw_speed, direction, reserved in ((4251, 1, False), (1000, 0, False), (0, 1, False),
                                           (1000, 3, False), (1000, 1, True)):
      self.assertFalse(self._tx(self._ars408_speed_msg(raw_speed, direction, reserved=reserved)))

  def test_ars408_yaw_rate_operational_range(self):
    self._enable_ars408_radar()
    for raw_yaw_rate in (22768, 32768, 42768):
      self.assertTrue(self._tx(self._ars408_yaw_msg(raw_yaw_rate)))
    for raw_yaw_rate in (22767, 42769):
      self.assertFalse(self._tx(self._ars408_yaw_msg(raw_yaw_rate)))

  def test_ars408_configuration_frames_remain_blocked(self):
    self._enable_ars408_radar()
    for address, length in ((0x200, 8), (0x202, 5)):
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(address, CANBUS.vehicle, b"\x00" * length)))

  def test_turn_signal_validation_requires_flag(self):
    self.assertFalse(self._tx(self._body_control_msg(1, 8, 12)))

  def test_turn_signal_validation_replays_only_fresh_rx_template(self):
    self._enable_turn_signal_validation()
    self.assertFalse(self._tx(self._body_control_msg(1, 8, 12)))
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 11)))
    self.assertTrue(self._tx(self._body_control_msg(1, 8, 12)))
    self.assertFalse(self._tx(self._body_control_msg(2, 8, 12)))

    self.assertTrue(self._rx(self._body_control_msg(0, 0, 12)))
    self.assertTrue(self._tx(self._body_control_msg(3, 4, 13)))

  def test_turn_signal_validation_allows_continuous_fresh_same_direction_frames(self):
    self._enable_turn_signal_validation()
    for index in range(20):
      counter = (11 + index) & 0xF
      self.assertTrue(self._rx(self._body_control_msg(0, 0, counter)))
      self.assertTrue(self._tx(self._body_control_msg(1, 8, (counter + 1) & 0xF)))

    # Cancellation resets the session and permits the other direction.
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 1)))
    self.assertTrue(self._tx(self._body_control_msg(3, 4, 2)))
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 2)))
    self.assertTrue(self._tx(self._body_control_msg(2, 8, 3)))

  def test_turn_signal_validation_times_out_until_cancel(self):
    self._enable_turn_signal_validation()
    self.safety.set_timer(100)
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 11)))
    self.assertTrue(self._tx(self._body_control_msg(1, 8, 12)))

    self.safety.set_timer(59_000_000)
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 12)))
    self.assertTrue(self._tx(self._body_control_msg(1, 8, 13)))

    self.safety.set_timer(60_000_101)
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 13)))
    self.assertFalse(self._tx(self._body_control_msg(1, 8, 14)))

    self.assertTrue(self._rx(self._body_control_msg(0, 0, 14)))
    self.assertTrue(self._tx(self._body_control_msg(3, 4, 15)))
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 15)))
    self.assertTrue(self._tx(self._body_control_msg(2, 8, 0)))

  def test_turn_signal_validation_has_independent_safety_frame_cap(self):
    self._enable_turn_signal_validation()
    for index in range(1200):
      counter = index & 0xF
      self.assertTrue(self._rx(self._body_control_msg(0, 0, counter)))
      self.assertTrue(self._tx(self._body_control_msg(1, 8, (counter + 1) & 0xF)))

    self.assertTrue(self._rx(self._body_control_msg(0, 0, 0)))
    self.assertFalse(self._tx(self._body_control_msg(1, 8, 1)))
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 1)))
    self.assertTrue(self._tx(self._body_control_msg(3, 4, 2)))

  def test_turn_signal_validation_rejects_direction_change_without_cancel(self):
    self._enable_turn_signal_validation()
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 11)))
    self.assertTrue(self._tx(self._body_control_msg(1, 8, 12)))
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 12)))
    self.assertFalse(self._tx(self._body_control_msg(2, 8, 13)))

  def test_turn_signal_validation_rejects_mutated_fields_and_bad_checksum(self):
    self._enable_turn_signal_validation()
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 11)))
    mutated = self._body_control_msg(1, 8, 12)
    mutated[0].data[4] ^= 1
    mutated[0].data[7] = (0xE9 + 0x03 + sum(mutated[0].data[i] for i in range(7))) & 0xFF
    self.assertFalse(self._tx(mutated))

    bad_checksum = self._body_control_msg(1, 8, 12)
    bad_checksum[0].data[7] ^= 0xFF
    self.assertFalse(self._tx(bad_checksum))

  def test_turn_signal_validation_rejects_stale_template(self):
    self._enable_turn_signal_validation()
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 11)))
    self.safety.set_timer(1_500_001)
    self.assertFalse(self._tx(self._body_control_msg(1, 8, 12)))

  def test_turn_signal_validation_allows_while_controls_allowed(self):
    self._enable_turn_signal_validation()
    self.assertTrue(self._rx(self._body_control_msg(0, 0, 11)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._body_control_msg(1, 8, 12)))

  def test_speed_button_validation_replays_only_fresh_rx_template(self):
    self._enable_speed_button_validation()
    self.assertFalse(self._tx(self._speed_wheel_msg(1)))
    self.assertTrue(self._rx(self._speed_wheel_msg(0)))
    self.assertTrue(self._tx(self._speed_wheel_msg(1)))
    self.assertFalse(self._tx(self._speed_wheel_msg(-1)))
    self.safety.set_timer(250_001)
    self.assertTrue(self._rx(self._speed_wheel_msg(0)))
    self.assertTrue(self._tx(self._speed_wheel_msg(-1)))
    self.safety.set_timer(1_750_002)
    self.assertFalse(self._tx(self._speed_wheel_msg(1)))

  def test_speed_button_validation_requires_vehicle_bus_flag(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.SPEED_BUTTON_VALIDATION)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.assertTrue(self._rx(self._speed_wheel_msg(0)))
    self.assertFalse(self._tx(self._speed_wheel_msg(1)))

  def test_speed_button_validation_rejects_other_field_changes(self):
    self._enable_speed_button_validation()
    self.assertTrue(self._rx(self._speed_wheel_msg(0)))
    mutated = self._speed_wheel_msg(1)
    mutated[0].data[4] ^= 1
    self.assertFalse(self._tx(mutated))

  def test_speed_button_validation_allows_while_controls_allowed(self):
    self._enable_speed_button_validation()
    self.assertTrue(self._rx(self._speed_wheel_msg(0)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._speed_wheel_msg(1)))

  def test_auto_speed_limit_requires_controls_allowed(self):
    self._enable_auto_speed_limit()
    self.assertTrue(self._rx(self._speed_wheel_msg(0)))
    self.assertFalse(self._tx(self._speed_wheel_msg(1)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._speed_wheel_msg(1)))

  def _accel_msg(self, accel: float):
    # For common.LongitudinalAccelSafetyTest
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def test_rx_hook(self):
    # counter check
    for msg_type in ("angle", "long", "speed", "speed_2"):
      # send multiple times to verify counter checks
      for i in range(10):
        if msg_type == "angle":
          msg = self._angle_cmd_msg(0, True, bus=2)
        elif msg_type == "long":
          msg = self._long_control_msg(0, bus=2)
        elif msg_type == "speed":
          msg = self._speed_msg(0)
        elif msg_type == "speed_2":
          msg = self._speed_msg_2(0)

        should_rx = i >= 5
        if not should_rx:
          # mess with checksums
          if msg_type == "angle":
            msg[0].data[3] = 0
          elif msg_type == "long":
            msg[0].data[7] = 0
          elif msg_type == "speed":
            msg[0].data[0] = 0
          elif msg_type == "speed_2":
            msg[0].data[7] = 0

        self.safety.set_controls_allowed(True)
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

      # Send static counters
      for i in range(MAX_WRONG_COUNTERS + 1):
        should_rx = i + 1 < MAX_WRONG_COUNTERS
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_vehicle_speed_measurements(self):
    # OVERRIDDEN: 79.1667 is the max speed in m/s
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_rx_hook_speed_mismatch(self):
    # TODO: overridden because of custom rounding
    # Tesla relies on speed for lateral limits close to ISO 11270, so it checks two sources
    for speed in np.arange(0, 40, 0.5):
      # match signal rounding on CAN
      speed = away_round(speed / 0.08 * 3.6) * 0.08 / 3.6
      for speed_delta in np.arange(-5, 5, 0.1):
        speed_2 = max(speed + speed_delta, 0)
        speed_2 = away_round(speed_2 * 2 * 3.6) / 2 / 3.6

        # Set controls allowed in between rx since first message can reset it
        self.assertTrue(self._rx(self._speed_msg(speed)))
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._rx(self._speed_msg_2(speed_2)))

        within_delta = abs(speed - speed_2) <= MAX_SPEED_DELTA
        self.assertEqual(self.safety.get_controls_allowed(), within_delta)

    # Test ESP_B quality flag
    for quality_flag in (True, False):
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._rx(self._speed_msg(0)))
      self.assertEqual(quality_flag, self._rx(self._speed_msg_2(0, quality_flag=quality_flag)))
      self.assertEqual(quality_flag, self.safety.get_controls_allowed())

  def test_user_brake_quality_flag(self):
    for quality_flag in (True, False):
      msg = self._user_brake_msg(True, quality_flag=quality_flag)
      self.assertEqual(quality_flag, self._rx(msg))

  def test_steering_wheel_disengage(self):
    # Tesla disengages when the user forcibly overrides the locked-in angle steering control
    # Either when the hands on level is high, or if there is a high angle rate fault
    for hands_on_level in range(4):
      for eac_status in range(8):
        for eac_error_code in range(16):
          self.safety.set_controls_allowed(True)

          should_disengage = hands_on_level >= 3 or (eac_status == 0 and eac_error_code == 9)
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=hands_on_level, eac_status=eac_status,
                                                        eac_error_code=eac_error_code)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())

          # Should not recover
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=1, eac_error_code=0)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertFalse(self.safety.get_steering_disengage_prev())

  def test_steering_wheel_torque_disengage(self):
    for torsion_bar_torque, should_disengage in (
      (-STEER_DISENGAGE_THRESHOLD - 0.01, True),
      (-STEER_DISENGAGE_THRESHOLD, False),
      (STEER_DISENGAGE_THRESHOLD, False),
      (STEER_DISENGAGE_THRESHOLD + 0.01, True),
    ):
      self.safety.set_controls_allowed(True)

      self.assertTrue(self._rx(self._angle_meas_msg(0, torsion_bar_torque=torsion_bar_torque)))
      self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
      self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())

      # Should not recover
      self.assertTrue(self._rx(self._angle_meas_msg(0)))
      self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
      self.assertFalse(self.safety.get_steering_disengage_prev())

  def test_autopark_summon_while_enabled(self):
    # We should not respect Autopark that activates while controls are allowed
    self._rx(self._pcm_status_msg(True, 0))

    self._rx(self._pcm_status_msg(True, self.autopark_states["SELFPARK_STARTED"]))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))

    # We should still not respect Autopark if we disengage cruise
    self._rx(self._pcm_status_msg(False, self.autopark_states["SELFPARK_STARTED"]))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))

  def test_autopark_summon_behavior(self):
    for autopark_state in range(16):
      self._rx(self._pcm_status_msg(False, 0))

      # We shouldn't allow controls if Autopark is an active state
      autopark_active = autopark_state in self.active_autopark_states
      self._rx(self._pcm_status_msg(False, autopark_state))
      self._rx(self._pcm_status_msg(True, autopark_state))
      self.assertNotEqual(autopark_active, self.safety.get_controls_allowed())

      # We should also start blocking all inactive/active openpilot msgs
      self.assertNotEqual(autopark_active, self._tx(self._angle_cmd_msg(0, False)))
      self.assertNotEqual(autopark_active, self._tx(self._angle_cmd_msg(0, True)))
      self.assertNotEqual(autopark_active, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))
      self.assertNotEqual(autopark_active or not self.LONGITUDINAL, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_ON"])))

      # Regain controls when Autopark disables
      self._rx(self._pcm_status_msg(True, 0))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
      self.assertTrue(self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"])))
      self.assertEqual(self.LONGITUDINAL, self._tx(self._long_control_msg(0, acc_state=self.acc_states["ACC_ON"])))

  def test_steering_control_type(self):
    # Angle control and LANE_KEEP_ASSIST are allowed (no EMERGENCY_LANE_KEEP)
    self.safety.set_controls_allowed(True)
    for steer_control_type in range(4):
      should_tx = steer_control_type in (self.steer_control_types["NONE"],
                                         self.steer_control_types["ANGLE_CONTROL"],
                                         self.steer_control_types["LANE_KEEP_ASSIST"])
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, state=steer_control_type)))

  def test_stock_lkas_passthrough(self):
    # TODO: make these generic passthrough tests
    no_lkas_msg = self._angle_cmd_msg(0, state=False)
    no_lkas_msg_cam = self._angle_cmd_msg(0, state=self.steer_control_types['NONE'], bus=2)
    lkas_msg_cam = self._angle_cmd_msg(0, state=self.steer_control_types['LANE_KEEP_ASSIST'], bus=2)

    # stock system sends no LKAS -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_lkas_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_lkas_msg_cam.addr))
    self.assertTrue(self._tx(no_lkas_msg))

    # stock system sends LKAS -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(lkas_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, lkas_msg_cam.addr))
    self.assertFalse(self._tx(no_lkas_msg))

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match DI_vehicleSpeed rounding on CAN
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        # angle signal can't represent 0, so it biases one unit down
        angle_unit_offset = -1 if sign == -1 else 0

        # at limit (safety tolerance adds 1)
        max_angle = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # 1 unit above limit
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 2) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = max(speed, 1)
      # match DI_vehicleSpeed rounding on CAN
      speed = round_speed(away_round(speed / 0.08 * 3.6) * 0.08 / 3.6)
      for sign in (-1, 1):  # (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # angle signal can't represent 0, so it biases one unit down
        angle_unit_offset = 1 if sign == -1 else 0

        # Stay within limits
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams), angle_unit_offset) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TestTeslaStockSafety(TestTeslaSafetyBase):

  LONGITUDINAL = False

  def test_cancel(self):
    for acc_state in range(16):
      self.safety.set_controls_allowed(True)
      should_tx = acc_state == self.acc_states["ACC_CANCEL_GENERIC_SILENT"]
      self.assertFalse(self._tx(self._long_control_msg(0, acc_state=acc_state, accel_limits=(self.MIN_ACCEL, self.MAX_ACCEL))))
      self.assertEqual(should_tx, self._tx(self._long_control_msg(0, acc_state=acc_state)))

  def test_no_aeb(self):
    for aeb_event in range(4):
      should_tx = aeb_event == 0
      ret = self._tx(self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=aeb_event))
      self.assertEqual(ret, should_tx)

  def test_stock_aeb_no_cancel(self):
    # No passthrough logic since we always forward DAS_control,
    # but ensure we can't send cancel cmd while stock AEB is active
    no_aeb_msg = self._long_control_msg(10, acc_state=self.acc_states["ACC_CANCEL_GENERIC_SILENT"], aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))


class TestTeslaFSD14StockSafety(TestTeslaStockSafety):
  SAFETY_PARAM = TeslaSafetyFlags.FSD_14


class TestTeslaLongitudinalSafety(TestTeslaSafetyBase):
  SAFETY_PARAM = TeslaSafetyFlags.LONG_CONTROL

  RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor, MSG_DAS_Control)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor, MSG_DAS_Control]}

  def test_four_finger_requires_python_handoff_decision(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.HAS_VEHICLE_BUS)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()
    packer_adas = CANPackerSafety("tesla_model3_vehicle")
    touch = packer_adas.make_can_msg_safety("UI_status2", CANBUS.vehicle, {"UI_activeTouchPoints": 4})
    handoff = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                     accel_limits=(0, 0), aeb_event=3, bus=0, counter=3)

    self.assertTrue(self._rx(touch))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_Control))
    self.assertFalse(self._tx(handoff))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

    op_cmd = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                    accel_limits=(0, 0), bus=0, counter=4)
    self.assertTrue(self._tx(op_cmd))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

  def test_dynamic_stock_handoff_is_atomic(self):
    param_sp = TeslaSafetyFlagsSP.DYNAMIC_AUTO_STOCK
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(param_sp)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.assertTrue(self._rx(self._pcm_status_msg(True, 0)))
    self.safety.set_controls_allowed(True)

    for stock_counter in (3, 6):
      stock = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                     accel_limits=(0, 0), bus=2, counter=stock_counter)
      handoff = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                       accel_limits=(0, 0), aeb_event=3, bus=0, counter=stock_counter)
      self.assertTrue(self._rx(stock))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

      self.assertFalse(self._tx(handoff))
      self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

      op_counter = (stock_counter + 1) % 8
      op_cmd = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                      accel_limits=(0, 0), bus=0, counter=op_counter)
      self.assertTrue(self._tx(op_cmd))
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

  def test_ap_hybrid_uses_atomic_stock_handoff_permission(self):
    self.assertNotEqual(TeslaSafetyFlagsSP.DYNAMIC_AUTO_STOCK, TeslaSafetyFlagsSP.AP_HYBRID_HANDOFF)
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.AP_HYBRID_HANDOFF)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()

    stock = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                   accel_limits=(0, 0), bus=2, counter=3)
    handoff = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                     accel_limits=(0, 0), aeb_event=3, bus=0, counter=3)
    self.assertTrue(self._rx(stock))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_Control))
    self.assertFalse(self._tx(handoff))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

  def test_ap_hybrid_lateral_handoff_is_atomic(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.AP_HYBRID_LATERAL_HANDOFF)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)

    stock = self._angle_cmd_msg(0, True, bus=2)
    handoff = self._angle_cmd_msg(0, 3, bus=0)
    self.assertTrue(self._rx(stock))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_steeringControl))

    self.assertFalse(self._tx(handoff))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_steeringControl))

    sp_takeover = self._angle_cmd_msg(0, True, bus=0)
    self.assertTrue(self._tx(sp_takeover))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_steeringControl))

  def test_rejected_sp_lateral_takeover_keeps_oem_forwarding_open(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.AP_HYBRID_LATERAL_HANDOFF)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)

    handoff = self._angle_cmd_msg(0, 3, bus=0)
    self.assertFalse(self._tx(handoff))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_steeringControl))

    rejected_takeover = self._angle_cmd_msg(self.STEER_ANGLE_MAX + 10, True, bus=0)
    self.assertFalse(self._tx(rejected_takeover))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_steeringControl))

    safe_inactive_takeover = self._angle_cmd_msg(0, False, bus=0)
    self.assertTrue(self._tx(safe_inactive_takeover))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_steeringControl))

  def test_rejected_sp_takeover_keeps_oem_forwarding_open(self):
    self.addCleanup(self.safety.set_current_safety_param_sp, 0)
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.DYNAMIC_AUTO_STOCK)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, self.SAFETY_PARAM)
    self.safety.init_tests()

    handoff = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                     accel_limits=(0, 0), aeb_event=3, bus=0, counter=3)
    rejected_takeover = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                               accel_limits=(1.0, 1.0), bus=0, counter=4)
    safe_takeover = self._long_control_msg(50, acc_state=self.acc_states["ACC_ON"],
                                           accel_limits=(0.0, 0.0), bus=0, counter=4)
    self.assertFalse(self._tx(handoff))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(rejected_takeover))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

    self.assertTrue(self._tx(safe_takeover))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, MSG_DAS_Control))

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> no forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def test_prevent_reverse(self):
    # Note: Tesla can reverse while at a standstill if both accel_min and accel_max are negative.
    self.safety.set_controls_allowed(True)

    # accel_min and accel_max are positive
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(1.1, 0.8))))

    # accel_min and accel_max are both zero
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, 0))))

    # accel_min and accel_max have opposing signs
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0.8, -1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, -1.3))))

    # accel_min and accel_max are negative
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.6, -1.1))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.1, -0.1))))


class TestTeslaFSD14LongitudinalSafety(TestTeslaLongitudinalSafety):
  SAFETY_PARAM = TeslaSafetyFlags.LONG_CONTROL | TeslaSafetyFlags.FSD_14


class TestTeslaIgnition(unittest.TestCase):
  TX_MSGS: list = []

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.packer = CANPackerSafety("tesla_model3_party")

  def _gear_msg(self, counter, gear):
    return self.packer.make_can_msg_safety("DI_systemStatus", 0,
                                           {"DI_systemStatusCounter": counter,
                                            "DI_gear": gear})

  def _power_msg(self, counter, state):
    return self.packer.make_can_msg_safety("VCFRONT_LVPowerState", 0,
                                           {"VCFRONT_LVPowerStateCounter": counter,
                                            "VCFRONT_vehiclePowerState": state})

  def test_power_state_drive_sets_ignition(self):
    self.safety.ignition_can_hook(self._power_msg(0, 3))
    self.assertFalse(self.safety.get_ignition_can())
    self.safety.ignition_can_hook(self._power_msg(1, 3))
    self.assertTrue(self.safety.get_ignition_can())

  def test_power_state_leaving_drive_clears_ignition(self):
    self.safety.ignition_can_hook(self._power_msg(0, 3))
    self.safety.ignition_can_hook(self._power_msg(1, 3))
    self.assertTrue(self.safety.get_ignition_can())

    self.safety.ignition_can_hook(self._power_msg(2, 2))
    self.safety.ignition_can_hook(self._power_msg(3, 2))
    self.assertFalse(self.safety.get_ignition_can())

  def test_gear_does_not_set_ignition(self):
    self.safety.ignition_can_hook(self._gear_msg(0, 4))
    self.safety.ignition_can_hook(self._gear_msg(1, 4))
    self.assertFalse(self.safety.get_ignition_can())

class TestTeslaVehicleBusSafety(TestTeslaSafetyBase):

  LONGITUDINAL = False

  def setUp(self):
    super().setUp()
    self.safety = libsafety_py.libsafety
    self.packer_adas = CANPackerSafety("tesla_model3_vehicle")
    self.safety.set_current_safety_param_sp(TeslaSafetyFlagsSP.HAS_VEHICLE_BUS | TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_3_FINGER)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0)
    self.safety.init_tests()

  def _lkas_button_msg(self, enabled):
    values = {"UI_activeTouchPoints": 3 if enabled else 0}
    return self.packer_adas.make_can_msg_safety("UI_status2", CANBUS.vehicle, values)

  def _gateway_gear_msg(self, gear=1, counter=0):
    return self.packer.make_can_msg_safety("DI_systemStatus", 0,
                                           {"DI_systemStatusCounter": counter, "DI_gear": gear})

  @staticmethod
  def _gateway_config_msg(payload):
    return libsafety_py.make_CANPacket(MSG_GATEWAY_CONFIG, CANBUS.vehicle, bytes(payload))

  def _set_gateway_config_safe_state(self, now=100_000):
    self.safety.set_timer(now)
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertTrue(self._rx(self._gateway_gear_msg()))
    self.assertTrue(self._rx(self._speed_msg(0)))
    self.assertTrue(self._rx(self._vehicle_moving_msg(0)))

  def test_gateway_config_payloads_parked_only(self):
    valid = [
      [0x01, 1, 0, 0, 0, 0, 0, 0],
      [0x02, 2, 47, 0, 0, 0, 0, 0],
      [0x03, 3, 0, 0, 0, 0, 0, 0],
      [0x10, 4, 48, 0, 1, 2, 3, 4],
      [0x11, 5, 47, 4, 0xFF, 0x07, 0xFF, 1],
      [0x12, 6, 0, 0, 1, 2, 3, 4],
      [0x13, 7, 0, 0, 0, 0, 0, 0],
    ]
    self._set_gateway_config_safe_state()
    for index, payload in enumerate(valid):
      self.safety.set_timer(100_000 + index * 25_000)
      self.assertTrue(self._tx(self._gateway_config_msg(payload)), payload)

  def test_gateway_config_rejects_malformed_payloads(self):
    invalid = [
      [0x00, 1, 0, 0, 0, 0, 0, 0],
      [0x01, 1, 0, 0, 0, 0, 0, 1],
      [0x02, 1, 48, 0, 0, 0, 0, 0],
      [0x03, 1, 1, 0, 0, 0, 0, 0],
      [0x10, 1, 49, 0, 0, 0, 0, 0],
      [0x10, 1, 1, 1, 0, 0, 0, 0],
      [0x11, 1, 48, 2, 1, 0, 8, 1],
      [0x11, 1, 0, 1, 1, 0, 8, 1],
      [0x11, 1, 0, 2, 0x00, 0x08, 8, 1],
      [0x11, 1, 0, 2, 0xE0, 0x06, 8, 1],
      [0x11, 1, 0, 2, 1, 0, 9, 1],
      [0x11, 1, 0, 2, 1, 0, 8, 2],
      [0x12, 1, 1, 0, 0, 0, 0, 0],
      [0x13, 1, 0, 0, 0, 0, 0, 1],
    ]
    for index, payload in enumerate(invalid):
      self._set_gateway_config_safe_state(100_000 + index * 50_000)
      self.assertFalse(self._tx(self._gateway_config_msg(payload)), payload)

  def test_gateway_config_requires_fresh_parked_disabled_state(self):
    msg = self._gateway_config_msg([0x01, 1, 0, 0, 0, 0, 0, 0])
    self.assertFalse(self._tx(msg))

    self._set_gateway_config_safe_state(100_000)
    self.assertTrue(self._tx(msg))
    self.assertFalse(self._tx(msg))  # Rate limit.
    self.safety.set_timer(125_000)
    self.assertTrue(self._tx(msg))

    self._set_gateway_config_safe_state(200_000)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(msg))
    self._set_gateway_config_safe_state(250_000)
    self.safety.set_controls_allowed_lateral(True)
    self.assertFalse(self._tx(msg))

    self._set_gateway_config_safe_state(300_000)
    self.assertTrue(self._rx(self._gateway_gear_msg(gear=4, counter=1)))
    self.assertFalse(self._tx(msg))
    self._set_gateway_config_safe_state(350_000)
    self.assertTrue(self._rx(self._speed_msg(1)))
    self.assertFalse(self._tx(msg))
    self._set_gateway_config_safe_state(400_000)
    self.assertTrue(self._rx(self._vehicle_moving_msg(1)))
    self.assertFalse(self._tx(msg))

    self._set_gateway_config_safe_state(500_000)
    self.safety.set_timer(1_000_001)
    self.assertFalse(self._tx(msg))

  def _set_mads_screen_button_config(self, finger_flag):
    param_sp = TeslaSafetyFlagsSP.HAS_VEHICLE_BUS
    if finger_flag is not None:
      param_sp |= finger_flag
    self.safety.set_current_safety_param_sp(param_sp)
    self.safety.set_safety_hooks(CarParams.SafetyModel.tesla, 0)
    self.safety.init_tests()

  def _touch_points_msg(self, touch_points):
    return self.packer_adas.make_can_msg_safety("UI_status2", CANBUS.vehicle, {"UI_activeTouchPoints": touch_points})

  def test_mads_screen_button_finger_count_match(self):
    """Configured finger count must match received touch-point count to register a MADS button press."""
    configs = [
      (TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_3_FINGER, 3),
      (TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_5_FINGER, 5),
    ]
    for config_flag, config_count in configs:
      for actual in (0, 3, 4, 5):
        with self.subTest(configured=config_count, actual=actual):
          self._set_mads_screen_button_config(config_flag)
          self._rx(self._touch_points_msg(actual))
          expected = 1 if actual == config_count else 0  # PRESSED vs NOT_PRESSED
          self.assertEqual(expected, self.safety.get_mads_button_press())

  def test_mads_screen_button_disabled(self):
    """With no finger-count flag set, touch messages must not change the MADS button state from UNAVAILABLE."""
    self._set_mads_screen_button_config(None)
    for actual in (0, 3, 4, 5):
      with self.subTest(actual=actual):
        self._rx(self._touch_points_msg(actual))
        self.assertEqual(-1, self.safety.get_mads_button_press())  # UNAVAILABLE


if __name__ == "__main__":
  unittest.main()
