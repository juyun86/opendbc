import json
import re
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.structs import CarParams
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState, blindspot_warning_active, blindspot_warning_level
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.teslacan import TeslaCAN, create_sccm_left_stalk
from opendbc.car.tesla.values import CANBUS, CAR, LEGACY_DAS_STEERING_FW, TeslaFlags
from opendbc.sunnypilot.car.tesla.carstate_ext import (AP_HYBRID_EXIT_RECOVERY_CONFIRM_SAMPLES, CarStateExt,
                                                        TeslaLongitudinalSource, publish_tesla_road_context)
from opendbc.sunnypilot.car.tesla import dynamic_acc_debug
from opendbc.sunnypilot.car.interfaces import (_initialize_tesla_ap_hybrid, _initialize_tesla_auto_speed_limit,
                                               _initialize_tesla_dynamic_auto_stock,
                                               _initialize_tesla_speed_button_validation,
                                               _initialize_tesla_turn_signal_validation)
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP, TeslaSafetyFlagsSP

Ecu = CarParams.Ecu


class TestTeslaBlindspotState(unittest.TestCase):
  def test_only_warning_levels_are_active(self):
    self.assertFalse(blindspot_warning_active(0))
    self.assertTrue(blindspot_warning_active(1))
    self.assertTrue(blindspot_warning_active(2))
    self.assertFalse(blindspot_warning_active(3))  # SNA is not an occupied blind spot.

  def test_preserves_the_two_warning_levels(self):
    self.assertEqual([blindspot_warning_level(value) for value in range(4)], [0, 1, 2, 0])


class TestTeslaSccmLeftStalk(unittest.TestCase):
  def test_matches_observed_vehicle_frames(self):
    observed_frames = {
      (0, 0): "9b000000",
      (10, 2): "fe0a0200",  # right turn
      (11, 2): "340b0200",
      (4, 6): "f7040600",   # left turn
      (5, 6): "a7050600",
    }
    for (counter, turn_state), expected in observed_frames.items():
      with self.subTest(counter=counter, turn_state=turn_state):
        msg = create_sccm_left_stalk(turn_state, counter)
        self.assertEqual(msg.address, 0x249)
        self.assertEqual(msg.src, CANBUS.vehicle)
        self.assertEqual(msg.dat.hex(), expected)

  def test_rejects_unvalidated_stalk_states(self):
    for turn_state in (1, 3, 4, 5, 7, 8, 9):
      with self.subTest(turn_state=turn_state):
        with self.assertRaises(ValueError):
          create_sccm_left_stalk(turn_state, 0)

# Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# Only `platform` has evidence-backed semantic meaning (matches car_model in FW_VERSIONS).
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# After the comma, the version string decomposes into:
#   platform             : E/Y/X = car model (Model 3 / Y / X). The only field with known meaning.
#   variant_code         : differentiator WITHIN a platform — hardware/trim/calibration bits packed
#                          into <digit?><letters?><3-digit series>, e.g. '4HP015', '4003', 'L014',
#                          'PR003'. We don't fully know what the parts mean individually, but the
#                          whole string identifies a specific variant within the car model.
#   software_major/minor : numeric components after the first '.' — conventional release numbers.
#                          minor is optional (e.g. 'E4S014.27' has no minor).
#
# Suspected (not confirmed): for M3/MY, `TeM3_*` outer + no-leading-digit variant_code == HW3, and
# `TeMYG4_*` outer + leading-'4' variant_code == HW4 (the 'G4' in TeMYG4 likely denotes Gen 4).
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  variant_code=4HP015  software_major=05  software_minor=0
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),' +
  rb'(?P<platform>[EYX])' +
  rb'(?P<variant_code>\d?[A-Z]*\d{3})' +
  rb'\.(?P<software_major>\d+)' +
  rb'(?:\.(?P<software_minor>\d+))?$'
)

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}

class TestTeslaFingerprint(unittest.TestCase):
  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)

        assert m is not None, f"Unparsable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_legacy_das_steering_fw(self):
    for car_model, fws in LEGACY_DAS_STEERING_FW.items():
      known = {fw for fws_list in FW_VERSIONS.get(car_model, {}).values() for fw in fws_list}
      for fw in fws:
        self.assertIn(fw, known, f"Legacy FW not in FW_VERSIONS for {car_model.value}: {fw}")

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined.
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_vehicle_can_parser_requires_vehicle_bus_fingerprint(self):
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], False, False, False)
    CP_SP = structs.CarParamsSP()

    assert Bus.adas not in CarState.get_can_parsers(CP, CP_SP)

    CP_SP.flags = TeslaFlagsSP.HAS_VEHICLE_BUS
    assert Bus.adas in CarState.get_can_parsers(CP, CP_SP)

  def test_road_context_uses_struct_state_until_capnp_conversion(self):
    state = structs.CarStateSP()
    publish_tesla_road_context(state, {"DAS_trafficLightColor": 2, "DAS_stopLineDist": 12.5}, 1_000, 1_100)
    assert state.teslaRoadContext.available
    assert state.teslaRoadContext.trafficLightColor == 2
    assert state.teslaRoadContext.stopLineDistance == 12.5

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC


class TestTeslaLongitudinalHandoff(unittest.TestCase):
  def test_controller_reads_cruise_state_from_carstate_output(self):
    car_state = SimpleNamespace(out=SimpleNamespace(cruiseState=SimpleNamespace(enabled=True)))
    self.assertTrue(CarController._cruise_enabled(car_state))

  def test_ap_hybrid_initialization_sets_runtime_and_safety_flags(self):
    cp = SimpleNamespace(brand="tesla", openpilotLongitudinalControl=True)
    cp_sp = SimpleNamespace(flags=0, safetyParam=0)

    _initialize_tesla_ap_hybrid(cp, cp_sp, {"TeslaApHybrid": "1"})

    self.assertTrue(cp_sp.flags & TeslaFlagsSP.AP_HYBRID)
    self.assertTrue(cp_sp.safetyParam & TeslaSafetyFlagsSP.AP_HYBRID_HANDOFF)
    self.assertTrue(cp_sp.safetyParam & TeslaSafetyFlagsSP.AP_HYBRID_LATERAL_HANDOFF)
    self.assertFalse(cp_sp.safetyParam & TeslaSafetyFlagsSP.DYNAMIC_AUTO_STOCK)

  def test_dynamic_ap_longitudinal_initialization_requires_ap_hybrid(self):
    cp = SimpleNamespace(brand="tesla", openpilotLongitudinalControl=True)
    cp_sp = SimpleNamespace(flags=0, safetyParam=0)

    _initialize_tesla_ap_hybrid(cp, cp_sp, {
      "TeslaApHybrid": "0",
      "TeslaDynamicApLongitudinal": "1",
    })
    self.assertFalse(cp_sp.flags & TeslaFlagsSP.DYNAMIC_AP_LONGITUDINAL)
    self.assertFalse(cp_sp.safetyParam & TeslaSafetyFlagsSP.AP_HYBRID_HANDOFF)

    _initialize_tesla_ap_hybrid(cp, cp_sp, {
      "TeslaApHybrid": "1",
      "TeslaDynamicApLongitudinal": "1",
    })

    self.assertTrue(cp_sp.flags & TeslaFlagsSP.AP_HYBRID)
    self.assertTrue(cp_sp.flags & TeslaFlagsSP.DYNAMIC_AP_LONGITUDINAL)
    self.assertTrue(cp_sp.safetyParam & TeslaSafetyFlagsSP.AP_HYBRID_HANDOFF)

  def test_dynamic_thresholds_enable_only_dynamic_handoff(self):
    cp = SimpleNamespace(brand="tesla", openpilotLongitudinalControl=True)
    cp_sp = SimpleNamespace(flags=0, safetyParam=0)

    _initialize_tesla_dynamic_auto_stock(cp, cp_sp, {
      "DynamicAutoStock": "1",
      "DynamicAutoStockSpeedKph": "85",
      "DynamicAutoStockSpeedLowKph": "70",
    })

    self.assertEqual(TeslaSafetyFlagsSP.DYNAMIC_AUTO_STOCK, cp_sp.safetyParam)

  def test_turn_signal_validation_initialization_requires_vehicle_bus(self):
    cp = SimpleNamespace(brand="tesla")
    cp_sp = SimpleNamespace(flags=0, safetyParam=0)
    _initialize_tesla_turn_signal_validation(cp, cp_sp, {"TeslaTurnSignalValidation": "1"})
    self.assertEqual(cp_sp.flags, 0)
    self.assertEqual(cp_sp.safetyParam, 0)

    cp_sp.flags = TeslaFlagsSP.HAS_VEHICLE_BUS
    _initialize_tesla_turn_signal_validation(cp, cp_sp, {"TeslaTurnSignalValidation": "1"})
    self.assertTrue(cp_sp.flags & TeslaFlagsSP.TURN_SIGNAL_VALIDATION)
    self.assertTrue(cp_sp.safetyParam & TeslaSafetyFlagsSP.TURN_SIGNAL_VALIDATION)

  def test_speed_button_validation_does_not_enable_icbm(self):
    cp = SimpleNamespace(brand="tesla")
    cp_sp = SimpleNamespace(flags=0, safetyParam=0, intelligentCruiseButtonManagementAvailable=False)
    _initialize_tesla_speed_button_validation(cp, cp_sp, {"TeslaSpeedButtonValidation": "1"})
    self.assertEqual(cp_sp.flags, 0)
    self.assertEqual(cp_sp.safetyParam, 0)

    cp_sp.flags = TeslaFlagsSP.HAS_VEHICLE_BUS
    _initialize_tesla_speed_button_validation(cp, cp_sp, {"TeslaSpeedButtonValidation": "1"})
    self.assertTrue(cp_sp.flags & TeslaFlagsSP.SPEED_BUTTON_VALIDATION)
    self.assertTrue(cp_sp.safetyParam & TeslaSafetyFlagsSP.SPEED_BUTTON_VALIDATION)
    self.assertFalse(cp_sp.intelligentCruiseButtonManagementAvailable)

  def test_auto_speed_limit_initialization_requires_vehicle_bus_and_sp_longitudinal(self):
    cp = SimpleNamespace(brand="tesla", openpilotLongitudinalControl=True)
    cp_sp = SimpleNamespace(flags=0, safetyParam=0)
    _initialize_tesla_auto_speed_limit(cp, cp_sp, {})
    self.assertEqual(cp_sp.flags, 0)

    cp_sp.flags = TeslaFlagsSP.HAS_VEHICLE_BUS
    _initialize_tesla_auto_speed_limit(cp, cp_sp, {})
    self.assertTrue(cp_sp.flags & TeslaFlagsSP.AUTO_SPEED_LIMIT)
    self.assertTrue(cp_sp.safetyParam & TeslaSafetyFlagsSP.AUTO_SPEED_LIMIT)

    cp.openpilotLongitudinalControl = False
    cp_sp.flags = TeslaFlagsSP.HAS_VEHICLE_BUS
    cp_sp.safetyParam = 0
    _initialize_tesla_auto_speed_limit(cp, cp_sp, {})
    self.assertFalse(cp_sp.flags & TeslaFlagsSP.AUTO_SPEED_LIMIT)
    self.assertFalse(cp_sp.safetyParam & TeslaSafetyFlagsSP.AUTO_SPEED_LIMIT)

  def test_speed_wheel_up_down_gesture_requests_auto_speed_resume(self):
    state = SimpleNamespace(
      tesla_speed_button_template=None,
      tesla_speed_button_template_nanos=0,
      tesla_manual_speed_adjustment_counter=0,
      tesla_speed_auto_resume_gesture_counter=0,
      _tesla_speed_resume_up_nanos=0,
      _tesla_speed_resume_down_nanos=0,
      _tesla_speed_resume_wait_idle=False,
    )
    idle = bytes.fromhex("2955000000000080")
    up = bytes.fromhex("2955000100000080")
    down = bytes.fromhex("2955003f00000080")

    CarStateExt.update_speed_button_template(state, idle, 1_000_000_000)
    CarStateExt.update_speed_button_template(state, up, 1_100_000_000)
    CarStateExt.update_speed_button_template(state, down, 2_500_000_000)
    self.assertEqual(state.tesla_manual_speed_adjustment_counter, 2)
    self.assertEqual(state.tesla_speed_auto_resume_gesture_counter, 1)

    CarStateExt.update_speed_button_template(state, idle, 2_600_000_000)
    CarStateExt.update_speed_button_template(state, up, 3_000_000_000)
    CarStateExt.update_speed_button_template(state, down, 4_500_000_001)
    self.assertEqual(state.tesla_speed_auto_resume_gesture_counter, 1)

    CarStateExt.update_speed_button_template(state, down, 5_000_000_000)
    CarStateExt.update_speed_button_template(state, up, 5_400_000_000)
    self.assertEqual(state.tesla_speed_auto_resume_gesture_counter, 2)

  def test_ap_hybrid_initialization_requires_openpilot_longitudinal(self):
    cp = SimpleNamespace(brand="tesla", openpilotLongitudinalControl=False)
    cp_sp = SimpleNamespace(flags=0, safetyParam=0)

    _initialize_tesla_ap_hybrid(cp, cp_sp, {"TeslaApHybrid": "1"})

    self.assertEqual(0, cp_sp.flags)
    self.assertEqual(0, cp_sp.safetyParam)

  def test_dynamic_acc_debug_writes_json_line(self):
    with tempfile.TemporaryDirectory() as temp_dir:
      log_path = Path(temp_dir) / "dynamic_acc_debug.log"
      with patch.object(dynamic_acc_debug, "DYNAMIC_ACC_DEBUG_PATH", str(log_path)):
        dynamic_acc_debug._append_dynamic_acc_debug({"source": "test", "event": "handoff"})

      self.assertEqual({"event": "handoff", "source": "test"}, json.loads(log_path.read_text()))

  def _stock_ready(self, *, acc_state=4, set_speed=82.0, speed_kph=80.0,
                   accel_min=0.2, accel_max=0.6, a_ego=0.2,
                   sp_requested_accel=0.4, sp_long_active=True, sp_context_valid=True):
    car_state = CarStateExt.__new__(CarStateExt)
    car_state._sp_requested_accel = sp_requested_accel
    car_state._sp_long_active = sp_long_active
    car_state._sp_longitudinal_context_valid = sp_context_valid
    car_state.das_control = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_accelMin": accel_min,
      "DAS_accelMax": accel_max,
    }
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=a_ego, cruiseState=SimpleNamespace(enabled=True))
    return car_state._stock_longitudinal_ready(ret, speed_kph)

  def _touch_toggle(self, *, initially_stock=False, acc_state=4):
    car_state = CarStateExt.__new__(CarStateExt)
    car_state.tesla_stock_longitudinal_active = initially_stock
    car_state._dyn_cooldown_frames = 0
    car_state._dyn_enter_frames = 10
    car_state._dyn_exit_frames = 10
    car_state._dyn_manual_override = False
    car_state._dyn_manual_saw_sp_off = False
    car_state._ap_dynamic_long_enabled = False
    car_state._sp_requested_accel = 0.0
    car_state._sp_long_active = True
    car_state._sp_longitudinal_context_valid = True
    car_state.das_control = {
      "DAS_setSpeed": 80.0,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_accelMin": 0.0,
      "DAS_accelMax": 0.0,
    }
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True))
    changed = car_state._toggle_stock_longitudinal_from_touch(ret, 80.0)
    return changed, car_state

  def test_stock_handoff_rejects_inactive_oem_acc(self):
    self.assertFalse(self._stock_ready(acc_state=0, set_speed=80.0, accel_min=0.0, accel_max=0.0))
    self.assertFalse(self._stock_ready(acc_state=13, set_speed=80.0, accel_min=0.0, accel_max=0.0))

  def test_four_finger_does_not_enter_inactive_oem_acc(self):
    changed, car_state = self._touch_toggle(acc_state=0)
    self.assertFalse(changed)
    self.assertFalse(car_state.tesla_stock_longitudinal_active)

  def test_four_finger_can_enter_ready_oem_acc_and_always_leave(self):
    changed, car_state = self._touch_toggle(acc_state=4)
    self.assertTrue(changed)
    self.assertTrue(car_state.tesla_stock_longitudinal_active)
    self.assertEqual(200, car_state._dyn_cooldown_frames)
    self.assertEqual(0, car_state._dyn_enter_frames)
    self.assertEqual(0, car_state._dyn_exit_frames)

    changed, car_state = self._touch_toggle(initially_stock=True, acc_state=0)
    self.assertTrue(changed)
    self.assertFalse(car_state.tesla_stock_longitudinal_active)

  def test_manual_switch_pauses_dynamic_until_sp_is_reenabled(self):
    changed, car_state = self._touch_toggle(initially_stock=True, acc_state=4)
    self.assertTrue(changed)
    self.assertTrue(car_state._dyn_manual_override)

    car_state._update_dynamic_manual_override(cruise_enabled=True)
    self.assertTrue(car_state._dyn_manual_override)
    car_state._update_dynamic_manual_override(cruise_enabled=False)
    self.assertTrue(car_state._dyn_manual_override)
    car_state._update_dynamic_manual_override(cruise_enabled=True)
    self.assertFalse(car_state._dyn_manual_override)

  def test_dynamic_stock_handoff_is_reachable_with_matched_demand(self):
    self.assertTrue(self._stock_ready())

  def test_dynamic_stock_handoff_does_not_wait_for_vehicle_speed_to_reach_cruise_setpoint(self):
    self.assertTrue(self._stock_ready(
      speed_kph=81.0,
      set_speed=120.0,
      accel_min=0.2,
      accel_max=0.8,
      sp_requested_accel=0.5,
    ))

  def test_stock_handoff_rejects_unmatched_demand(self):
    self.assertFalse(self._stock_ready(accel_min=0.8, accel_max=1.0))
    self.assertFalse(self._stock_ready(a_ego=0.5))

  def test_dynamic_stock_handoff_rejects_acceleration_spike_risk(self):
    self.assertFalse(self._stock_ready(set_speed=87.5, speed_kph=80.0, accel_min=-1.12, accel_max=0.24))
    self.assertFalse(self._stock_ready(set_speed=82.0, speed_kph=80.0, accel_min=-1.12, accel_max=2.0))

  def test_stock_handoff_requires_fresh_active_sp_acceleration(self):
    self.assertFalse(self._stock_ready(sp_long_active=False))
    self.assertFalse(self._stock_ready(sp_context_valid=False))

  @staticmethod
  def _override_state(owner=TeslaLongitudinalSource.sp):
    car_state = CarStateExt.__new__(CarStateExt)
    car_state._init_longitudinal_override_state()
    car_state._set_longitudinal_source(owner)
    car_state._dyn_enter_frames = 0
    car_state._dyn_exit_frames = 0
    car_state._dyn_cooldown_frames = 0
    car_state._dyn_debug_followup_frames = 0
    car_state._dyn_manual_override = False
    car_state._dyn_manual_saw_sp_off = False
    car_state._stock_counter_last = None
    car_state._ap_dynamic_long_enabled = False
    car_state._ap_dynamic_cooldown_frames = 0
    car_state._dyn_blinker_to_sp_enabled = True
    car_state._dyn_curve_to_sp_enabled = True
    car_state._sp_long_active = True
    car_state._sp_longitudinal_context_valid = True
    car_state._sp_requested_accel = 0.0
    car_state._oem_auto_lane_change_state = 0
    car_state._ap_lane_change_hold_logged = False
    car_state.das_control = {
      "DAS_accState": 4,
      "DAS_setSpeed": 80.0,
      "DAS_accelMin": 0.0,
      "DAS_accelMax": 0.0,
      "DAS_aebEvent": 0,
      "DAS_controlCounter": 0,
    }
    car_state.das_steering_control = {
      "DAS_steeringControlType": 0,
    }
    return car_state

  def test_blinker_requires_distinct_samples_and_300ms(self):
    car_state = self._override_state()

    car_state._update_blinker_sample(True, 1, 1.00)
    car_state._update_blinker_sample(True, 1, 1.10)  # repeated parser value, not a new CAN frame
    car_state._update_blinker_sample(True, 2, 1.10)
    car_state._update_blinker_sample(True, 3, 1.20)
    self.assertFalse(car_state._blinker_force_active(1.20))

    car_state._update_blinker_sample(True, 4, 1.30)
    self.assertTrue(car_state._blinker_force_active(1.30))

  def test_blinker_counter_wrap_and_stale_are_handled(self):
    car_state = self._override_state()
    for counter, now in ((14, 1.0), (15, 1.1), (0, 1.2), (1, 1.3)):
      car_state._update_blinker_sample(True, counter, now)

    self.assertTrue(car_state._blinker_force_active(1.3))
    self.assertFalse(car_state._blinker_force_active(1.71))
    self.assertFalse(car_state._blinker_known_inactive(1.71))

    car_state._update_blinker_sample(False, 2, 1.8)
    self.assertTrue(car_state._blinker_known_inactive(1.8))

  def test_curve_request_counts_only_new_fresh_plan_samples(self):
    car_state = self._override_state()
    car_state.update_longitudinal_context(1, True, True, 2.00, False, True, True, 2.00)
    car_state.update_longitudinal_context(1, False, True, 2.00, False, True, True, 2.01)
    self.assertFalse(car_state._curve_force_active(2.01))

    car_state.update_longitudinal_context(1, True, True, 2.05, False, True, True, 2.05)
    self.assertTrue(car_state._curve_force_active(2.05))
    self.assertFalse(car_state._curve_force_active(2.26))
    self.assertFalse(car_state._external_context_clear(2.26))

    car_state.update_longitudinal_context(1, True, True, 2.50, False, True, True, 2.50)
    self.assertFalse(car_state._curve_force_active(2.50))
    car_state.update_longitudinal_context(1, True, True, 2.55, False, True, True, 2.55)
    self.assertTrue(car_state._curve_force_active(2.55))

  def test_force_sp_only_overrides_dynamic_stock(self):
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    dynamic = self._override_state(TeslaLongitudinalSource.dynamicStock)
    self.assertTrue(dynamic._force_dynamic_stock_to_sp("blinker", ret, 80.0))
    self.assertEqual(TeslaLongitudinalSource.sp, dynamic.tesla_longitudinal_source)
    self.assertFalse(dynamic.tesla_stock_longitudinal_active)

    manual = self._override_state(TeslaLongitudinalSource.manualStock)
    self.assertFalse(manual._force_dynamic_stock_to_sp("blinker", ret, 80.0))
    self.assertEqual(TeslaLongitudinalSource.manualStock, manual.tesla_longitudinal_source)

    hybrid = self._override_state(TeslaLongitudinalSource.apHybridStock)
    self.assertFalse(hybrid._force_dynamic_stock_to_sp("curve", ret, 80.0))
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, hybrid.tesla_longitudinal_source)

  def test_dynamic_stock_force_sp_reasons_are_independently_configurable(self):
    car_state = self._override_state(TeslaLongitudinalSource.dynamicStock)
    for counter, now in ((1, 1.0), (2, 1.1), (3, 1.2), (4, 1.3)):
      car_state._update_blinker_sample(True, counter, now)
    car_state.update_longitudinal_context(1, True, True, 1.25, False, True, True, 1.3)
    car_state.update_longitudinal_context(1, True, True, 1.30, False, True, True, 1.3)

    self.assertEqual("blinker", car_state._force_sp_reason(1.3))

    car_state._dyn_blinker_to_sp_enabled = False
    self.assertEqual("visionCurve", car_state._force_sp_reason(1.3))

    car_state._dyn_curve_to_sp_enabled = False
    self.assertIsNone(car_state._force_sp_reason(1.3))
    self.assertTrue(car_state._external_context_clear(1.3))

    car_state._dyn_blinker_to_sp_enabled = True
    self.assertEqual("blinker", car_state._force_sp_reason(1.3))

  def test_runtime_flags_encode_each_longitudinal_source(self):
    expected = {
      TeslaLongitudinalSource.sp: 0,
      TeslaLongitudinalSource.dynamicStock: TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE | TeslaFlagsSP.DYNAMIC_STOCK_ACTIVE,
      TeslaLongitudinalSource.manualStock: TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE | TeslaFlagsSP.MANUAL_STOCK_ACTIVE,
      TeslaLongitudinalSource.apHybridStock: TeslaFlagsSP.STOCK_LONGITUDINAL_ACTIVE | TeslaFlagsSP.AP_HYBRID_ACTIVE,
    }
    for source, flags in expected.items():
      car_state = self._override_state(source)
      self.assertEqual(flags, car_state._longitudinal_source_flags())

    dynamic_ap_sp = self._override_state(TeslaLongitudinalSource.sp)
    dynamic_ap_sp.tesla_ap_hybrid_active = True
    self.assertEqual(TeslaFlagsSP.AP_HYBRID_ACTIVE, dynamic_ap_sp._longitudinal_source_flags())
    dynamic_ap_sp.tesla_stock_lateral_active = True
    self.assertEqual(TeslaFlagsSP.AP_HYBRID_ACTIVE | TeslaFlagsSP.AP_HYBRID_STOCK_LATERAL_ACTIVE,
                     dynamic_ap_sp._longitudinal_source_flags())
    dynamic_ap_sp.tesla_ap_hybrid_active = False
    dynamic_ap_sp.tesla_stock_lateral_active = False
    dynamic_ap_sp._ap_hybrid_exit_recovery_active = True
    self.assertEqual(TeslaFlagsSP.AP_HYBRID_EXIT_RECOVERY_ACTIVE, dynamic_ap_sp._longitudinal_source_flags())

  def test_dynamic_ap_switches_longitudinal_source_with_speed_hysteresis(self):
    car_state = self._override_state(TeslaLongitudinalSource.manualStock)
    car_state._ap_hybrid_enabled = True
    car_state._ap_dynamic_long_enabled = True
    car_state._dyn_high = 80
    car_state._dyn_low = 70
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 60.0, status_counter=1))
    self.assertTrue(car_state.tesla_ap_hybrid_active)
    self.assertEqual(TeslaLongitudinalSource.sp, car_state.tesla_longitudinal_source)
    self.assertFalse(car_state.tesla_stock_lateral_active)
    self.assertTrue(car_state._ap_hybrid_lkas_suppressed())

    car_state.das_control["DAS_setSpeed"] = 90.0
    for counter in range(2, 102):
      car_state.das_control["DAS_controlCounter"] = counter % 8
      self.assertTrue(car_state._update_ap_hybrid(ret, 3, 90.0, status_counter=counter % 16))
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)
    self.assertTrue(car_state.tesla_stock_lateral_active)

    for counter in range(102, 202):
      car_state.das_control["DAS_controlCounter"] = counter % 8
      self.assertTrue(car_state._update_ap_hybrid(ret, 3, 60.0, status_counter=counter % 16))
    self.assertTrue(car_state.tesla_ap_hybrid_active)
    self.assertEqual(TeslaLongitudinalSource.sp, car_state.tesla_longitudinal_source)
    self.assertFalse(car_state.tesla_stock_lateral_active)

    for counter in (10, 11, 12):
      active = car_state._update_ap_hybrid(ret, 2, 60.0, status_counter=counter)
    self.assertFalse(active)
    self.assertFalse(car_state.tesla_ap_hybrid_active)
    self.assertEqual(TeslaLongitudinalSource.manualStock, car_state.tesla_longitudinal_source)

  def test_dynamic_ap_accepts_compatible_oem_acceleration_envelope(self):
    car_state = self._override_state(TeslaLongitudinalSource.sp)
    car_state._ap_hybrid_enabled = True
    car_state._ap_dynamic_long_enabled = True
    car_state.tesla_ap_hybrid_active = True
    car_state._dyn_high = 80
    car_state._dyn_low = 70
    car_state.das_control.update(DAS_setSpeed=90.0, DAS_accelMin=-0.6, DAS_accelMax=-0.2)
    car_state._sp_requested_accel = 0.6
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    for counter in range(100):
      car_state.das_control["DAS_controlCounter"] = counter % 8
      car_state._update_ap_dynamic_longitudinal(ret, 90.0, 3)
    self.assertEqual(TeslaLongitudinalSource.sp, car_state.tesla_longitudinal_source)

    # DAS_accelMin/Max is an allowed acceleration envelope, not a target whose
    # midpoint must match SP. The requested 0.6 m/s² is compatible with this
    # envelope plus the handoff tolerance even though its midpoint is -0.5.
    car_state.das_control.update(DAS_accelMin=-1.4, DAS_accelMax=0.1)
    for counter in range(100, 200):
      car_state.das_control["DAS_controlCounter"] = counter % 8
      car_state._update_ap_dynamic_longitudinal(ret, 90.0, 3)
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)

  def test_dynamic_ap_switches_at_inclusive_speed_thresholds(self):
    car_state = self._override_state(TeslaLongitudinalSource.sp)
    car_state._ap_dynamic_long_enabled = True
    car_state.tesla_ap_hybrid_active = True
    car_state._dyn_high = 45
    car_state._dyn_low = 40
    car_state.das_control.update(DAS_setSpeed=45.0, DAS_accelMin=-0.2, DAS_accelMax=0.2)
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    for counter in range(100):
      car_state.das_control["DAS_controlCounter"] = counter % 8
      car_state._update_ap_dynamic_longitudinal(ret, 45.0, 3)
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)

    for counter in range(100, 200):
      car_state.das_control["DAS_controlCounter"] = counter % 8
      car_state._update_ap_dynamic_longitudinal(ret, 40.0, 3)
    self.assertEqual(TeslaLongitudinalSource.sp, car_state.tesla_longitudinal_source)

  def test_dynamic_ap_waits_for_sp_acceleration_before_leaving_stock(self):
    car_state = self._override_state(TeslaLongitudinalSource.apHybridStock)
    car_state._ap_dynamic_long_enabled = True
    car_state.tesla_ap_hybrid_active = True
    car_state._dyn_high = 80
    car_state._dyn_low = 70
    car_state.das_control.update(DAS_accelMin=0.4, DAS_accelMax=0.8)
    car_state._sp_requested_accel = -1.0
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    for counter in range(100):
      car_state.das_control["DAS_controlCounter"] = counter % 8
      car_state._update_ap_dynamic_longitudinal(ret, 60.0, 3)
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)

    car_state._sp_requested_accel = 0.4
    for counter in range(100, 200):
      car_state.das_control["DAS_controlCounter"] = counter % 8
      car_state._update_ap_dynamic_longitudinal(ret, 60.0, 3)
    self.assertEqual(TeslaLongitudinalSource.sp, car_state.tesla_longitudinal_source)

  def test_dynamic_ap_latches_sp_lateral_after_driver_override(self):
    car_state = self._override_state(TeslaLongitudinalSource.sp)
    car_state._ap_hybrid_enabled = True
    car_state._ap_dynamic_long_enabled = True
    car_state._dyn_high = 80
    car_state._dyn_low = 70
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 90.0, status_counter=1))
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)
    self.assertTrue(car_state.tesla_stock_lateral_active)

    ret.steeringTorque = 0.6
    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 90.0, status_counter=3))
    self.assertFalse(car_state.tesla_stock_lateral_active)
    self.assertTrue(car_state._ap_driver_lateral_takeover)

    ret.steeringTorque = 0.0
    for counter in range(3, 103):
      self.assertTrue(car_state._update_ap_hybrid(ret, 3, 90.0, status_counter=counter % 16))
    self.assertFalse(car_state.tesla_stock_lateral_active)

    ret.leftBlinker = True
    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 90.0, status_counter=7))
    self.assertFalse(car_state.tesla_stock_lateral_active)

    ret.leftBlinker = False
    car_state.tesla_stock_lateral_active = True
    car_state._lane_change_active = True
    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 90.0, status_counter=8))
    self.assertFalse(car_state.tesla_stock_lateral_active)

    car_state._lane_change_active = False
    car_state.tesla_stock_lateral_active = True
    self.assertTrue(car_state._update_ap_hybrid(ret, 8, 90.0, status_counter=8))
    self.assertFalse(car_state.tesla_stock_lateral_active)

  def test_ap_hybrid_brake_exit_suppresses_lkas_until_oem_state_recovers(self):
    car_state = self._override_state(TeslaLongitudinalSource.apHybridStock)
    car_state._ap_hybrid_enabled = True
    car_state.tesla_ap_hybrid_active = True
    car_state.tesla_stock_lateral_active = True
    ret = SimpleNamespace(brakePressed=True, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=-0.5, cruiseState=SimpleNamespace(enabled=False, available=True))

    self.assertFalse(car_state._update_ap_hybrid(ret, 3, 40.0, status_counter=1))
    self.assertTrue(car_state._ap_hybrid_lkas_suppressed(3))
    self.assertFalse(car_state.tesla_stock_lateral_active)

    for counter in range(2, 302):
      self.assertFalse(car_state._update_ap_hybrid(ret, 3, 40.0, status_counter=counter % 16))
      self.assertTrue(car_state._ap_hybrid_lkas_suppressed(3))

    car_state.das_steering_control["DAS_steeringControlType"] = 0
    for counter in range(14, 18):
      self.assertFalse(car_state._update_ap_hybrid(ret, 2, 40.0, status_counter=counter % 16))
      self.assertTrue(car_state._ap_hybrid_lkas_suppressed(2))
    self.assertFalse(car_state._update_ap_hybrid(ret, 2, 40.0, status_counter=2))
    self.assertFalse(car_state._ap_hybrid_lkas_suppressed(2))

  def test_dynamic_ap_sp_only_brake_exit_suppresses_residual_oem_lkas(self):
    car_state = self._override_state(TeslaLongitudinalSource.sp)
    car_state._ap_hybrid_enabled = True
    car_state._ap_dynamic_long_enabled = True
    car_state.tesla_ap_hybrid_active = True
    car_state.tesla_stock_lateral_active = False
    ret = SimpleNamespace(brakePressed=True, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=-0.5, cruiseState=SimpleNamespace(enabled=False, available=True))

    self.assertFalse(car_state._update_ap_hybrid(ret, 3, 40.0, status_counter=1))
    self.assertTrue(car_state._ap_hybrid_exit_recovery_active)
    self.assertTrue(car_state._ap_hybrid_lkas_suppressed(3))
    self.assertEqual(TeslaFlagsSP.AP_HYBRID_EXIT_RECOVERY_ACTIVE, car_state._longitudinal_source_flags())

    car_state.das_steering_control["DAS_steeringControlType"] = 1
    for counter in range(2, 8):
      self.assertFalse(car_state._update_ap_hybrid(ret, 2, 40.0, status_counter=counter))
      self.assertTrue(car_state._ap_hybrid_lkas_suppressed(2))

    car_state.das_steering_control["DAS_steeringControlType"] = 0
    for counter in range(8, 12):
      self.assertFalse(car_state._update_ap_hybrid(ret, 2, 40.0, status_counter=counter))
      self.assertTrue(car_state._ap_hybrid_lkas_suppressed(2))
    self.assertFalse(car_state._update_ap_hybrid(ret, 2, 40.0, status_counter=12))
    self.assertFalse(car_state._ap_hybrid_lkas_suppressed(2))

  def test_ap_hybrid_normal_brake_exit_starts_recovery_after_aborted_state(self):
    car_state = self._override_state(TeslaLongitudinalSource.apHybridStock)
    car_state._ap_hybrid_enabled = True
    car_state.tesla_ap_hybrid_active = True
    car_state.tesla_stock_lateral_active = True
    car_state.das_steering_control["DAS_steeringControlType"] = 1
    ret = SimpleNamespace(brakePressed=True, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=-0.5, cruiseState=SimpleNamespace(enabled=False, available=True))

    self.assertTrue(car_state._update_ap_hybrid(ret, 8, 50.0, status_counter=1))
    self.assertTrue(car_state._update_ap_hybrid(ret, 9, 50.0, status_counter=2))
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 50.0, status_counter=3))
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 50.0, status_counter=4))
    self.assertFalse(car_state._update_ap_hybrid(ret, 2, 50.0, status_counter=5))
    self.assertTrue(car_state._ap_hybrid_exit_recovery_active)
    self.assertTrue(car_state._ap_hybrid_lkas_suppressed(2))

  def test_ap_hybrid_lane_change_available_state_keeps_session_and_longitudinal_source(self):
    car_state = self._override_state(TeslaLongitudinalSource.apHybridStock)
    car_state._ap_hybrid_enabled = True
    car_state._ap_dynamic_long_enabled = True
    car_state.tesla_ap_hybrid_active = True
    car_state.tesla_stock_lateral_active = True
    car_state._oem_auto_lane_change_state = 9
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=True, rightBlinker=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    for counter in range(1, 8):
      self.assertTrue(car_state._update_ap_hybrid(ret, 2, 90.0, status_counter=counter))
      self.assertTrue(car_state.tesla_ap_hybrid_active)
      self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)
      self.assertFalse(car_state.tesla_stock_lateral_active)

    ret.leftBlinker = False
    car_state._oem_auto_lane_change_state = 0
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 90.0, status_counter=8))
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 90.0, status_counter=9))
    self.assertFalse(car_state._update_ap_hybrid(ret, 2, 90.0, status_counter=10))

  def test_ap_hybrid_exit_recovery_never_suppresses_oem_fault(self):
    car_state = self._override_state(TeslaLongitudinalSource.apHybridStock)
    car_state._ap_hybrid_enabled = True
    car_state.tesla_ap_hybrid_active = True
    ret = SimpleNamespace(brakePressed=True, gasPressed=False, accFaulted=False, steeringTorque=0.0,
                          leftBlinker=False, rightBlinker=False,
                          aEgo=-0.5, cruiseState=SimpleNamespace(enabled=False, available=True))

    self.assertFalse(car_state._update_ap_hybrid(ret, 3, 40.0, status_counter=1))
    self.assertTrue(car_state._ap_hybrid_lkas_suppressed(3))
    self.assertFalse(car_state._ap_hybrid_lkas_suppressed(14))
    self.assertFalse(car_state._update_ap_hybrid(ret, 14, 40.0, status_counter=2))
    self.assertFalse(car_state._ap_hybrid_lkas_suppressed(3))

  def test_ap_hybrid_restores_complete_previous_source(self):
    car_state = self._override_state(TeslaLongitudinalSource.manualStock)
    car_state._ap_hybrid_enabled = True
    car_state.tesla_ap_hybrid_active = False
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 80.0))
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)

    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 80.0))
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 80.0))
    self.assertFalse(car_state._update_ap_hybrid(ret, 2, 80.0))
    self.assertEqual(TeslaLongitudinalSource.manualStock, car_state.tesla_longitudinal_source)

  def test_ap_hybrid_stays_active_through_oem_exit_states(self):
    car_state = self._override_state(TeslaLongitudinalSource.sp)
    car_state._ap_hybrid_enabled = True
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 80.0))
    for autopilot_state in (8, 9):
      self.assertTrue(car_state._update_ap_hybrid(ret, autopilot_state, 80.0))
      self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)

    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 80.0))
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 80.0))
    self.assertFalse(car_state._update_ap_hybrid(ret, 2, 80.0))
    self.assertEqual(TeslaLongitudinalSource.sp, car_state.tesla_longitudinal_source)
    self.assertTrue(car_state._ap_hybrid_lkas_suppressed())
    for _ in range(AP_HYBRID_EXIT_RECOVERY_CONFIRM_SAMPLES - 1):
      self.assertFalse(car_state._update_ap_hybrid(ret, 2, 80.0))
      self.assertTrue(car_state._ap_hybrid_lkas_suppressed())
    self.assertFalse(car_state._update_ap_hybrid(ret, 2, 80.0))
    self.assertFalse(car_state._ap_hybrid_lkas_suppressed())
    self.assertFalse(car_state._ap_hybrid_lkas_suppressed(14))

  def test_ap_hybrid_exit_requires_three_distinct_stable_status_samples(self):
    car_state = self._override_state(TeslaLongitudinalSource.sp)
    car_state._ap_hybrid_enabled = True
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 80.0, status_counter=1))
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 80.0, status_counter=2))
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 80.0, status_counter=2))  # cached parser value
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 80.0, status_counter=3))
    self.assertFalse(car_state._update_ap_hybrid(ret, 2, 80.0, status_counter=4))

  def test_ap_hybrid_available_flicker_does_not_change_source(self):
    car_state = self._override_state(TeslaLongitudinalSource.dynamicStock)
    car_state._ap_hybrid_enabled = True
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 80.0, status_counter=1))
    self.assertTrue(car_state._update_ap_hybrid(ret, 2, 80.0, status_counter=2))
    self.assertTrue(car_state._update_ap_hybrid(ret, 4, 80.0, status_counter=3))
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)
    self.assertEqual(TeslaLongitudinalSource.dynamicStock, car_state._ap_hybrid_restore_source)

  def test_ap_hybrid_entry_requires_lateral_control(self):
    car_state = self._override_state(TeslaLongitudinalSource.sp)
    car_state._ap_hybrid_enabled = True
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    self.assertFalse(car_state._update_ap_hybrid(ret, 3, 80.0, lateral_control_ready=False))
    self.assertEqual(TeslaLongitudinalSource.sp, car_state.tesla_longitudinal_source)
    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 80.0, lateral_control_ready=True))

  def test_ap_hybrid_ignores_dynamic_force_requests(self):
    car_state = self._override_state(TeslaLongitudinalSource.apHybridStock)
    car_state._ap_hybrid_enabled = True
    car_state.tesla_ap_hybrid_active = True
    ret = SimpleNamespace(brakePressed=False, gasPressed=False, accFaulted=False,
                          aEgo=0.0, cruiseState=SimpleNamespace(enabled=True, available=True))

    self.assertTrue(car_state._update_ap_hybrid(ret, 3, 80.0))
    self.assertFalse(car_state._force_dynamic_stock_to_sp("curve", ret, 80.0))
    self.assertEqual(TeslaLongitudinalSource.apHybridStock, car_state.tesla_longitudinal_source)

  def test_stock_return_requires_one_second_of_clear_context(self):
    car_state = self._override_state()
    for index in range(10):
      now = 3.0 + index * 0.1
      car_state.update_longitudinal_context(0, True, True, now, False, True, True, now)
      car_state._update_blinker_sample(False, index, now)
    self.assertFalse(car_state._stock_return_context_ready(3.99))
    car_state.update_longitudinal_context(0, True, True, 4.0, False, True, True, 4.0)
    car_state._update_blinker_sample(False, 10, 4.0)
    self.assertTrue(car_state._stock_return_context_ready(4.0))

    car_state.update_longitudinal_context(1, True, True, 4.01, False, True, True, 4.01)
    car_state.update_longitudinal_context(1, True, True, 4.06, False, True, True, 4.06)
    self.assertFalse(car_state._stock_return_context_ready(4.06))

  def test_counter_resyncs_after_each_stock_period(self):
    controller = CarController.__new__(CarController)
    controller.long_control_counter = None

    self.assertEqual(3, controller._next_long_control_counter(2))
    self.assertEqual(4, controller._next_long_control_counter(7))
    self.assertEqual(7, controller._next_long_control_counter(6, resync=True))
    self.assertEqual(0, controller._next_long_control_counter(3))
    self.assertEqual(2, controller._next_long_control_counter(1, resync=True))

  def test_inactive_sp_takeover_preserves_enabled_cruise_with_zero_accel(self):
    state, accel = CarController._longitudinal_state_accel(
      leaving_stock=True, cruise_enabled=True, long_active=False, cancel=False, requested_accel=-1.2,
    )
    self.assertEqual(4, state)
    self.assertEqual(0.0, accel)

  def test_sp_takeover_cancels_only_when_cruise_is_disabled(self):
    state, accel = CarController._longitudinal_state_accel(
      leaving_stock=True, cruise_enabled=False, long_active=False, cancel=False, requested_accel=-1.2,
    )
    self.assertEqual(13, state)
    self.assertEqual(0.0, accel)

  def test_active_sp_takeover_preserves_control(self):
    state, accel = CarController._longitudinal_state_accel(
      leaving_stock=True, cruise_enabled=True, long_active=True, cancel=False, requested_accel=-1.2,
    )
    self.assertEqual(4, state)
    self.assertEqual(-1.2, accel)

  def test_sp_takeover_accel_ramps_up_from_stock_accel(self):
    controller = CarController.__new__(CarController)
    controller.frame = 100
    controller.last_long_control_frame = 96
    controller.sp_takeover_accel = 0.0
    controller.sp_takeover_ramp_frames = 100

    accel = controller._limited_sp_takeover_accel(long_active=True, requested_accel=1.2)

    self.assertGreater(accel, 0.0)
    self.assertLess(accel, 1.2)
    self.assertLess(controller.sp_takeover_ramp_frames, 100)

  def test_sp_takeover_starts_from_measured_vehicle_acceleration(self):
    state = SimpleNamespace(
      out=SimpleNamespace(aEgo=-2.31),
      das_control={"DAS_accelMin": -2.24, "DAS_accelMax": 1.52},
    )

    self.assertAlmostEqual(-2.31, CarController._stock_takeover_accel(state))

    state.out.aEgo = float("nan")
    self.assertAlmostEqual(-0.36, CarController._stock_takeover_accel(state))

  def test_sp_takeover_ramp_ignores_stale_pre_stock_command_time(self):
    controller = CarController.__new__(CarController)
    controller.frame = 1000
    controller.last_long_control_frame = 100
    controller.sp_takeover_accel = -0.44
    controller.sp_takeover_ramp_frames = 100

    accel = controller._limited_sp_takeover_accel(long_active=True, requested_accel=-2.22)

    self.assertAlmostEqual(-0.50, accel)
    self.assertEqual(96, controller.sp_takeover_ramp_frames)

  def test_inactive_sp_takeover_does_not_ramp_accel(self):
    controller = CarController.__new__(CarController)
    controller.frame = 100
    controller.last_long_control_frame = 96
    controller.sp_takeover_accel = 0.0
    controller.sp_takeover_ramp_frames = 100

    accel = controller._limited_sp_takeover_accel(long_active=False, requested_accel=1.2)

    self.assertEqual(1.2, accel)
    self.assertEqual(1.2, controller.sp_takeover_accel)

  def test_stock_handoff_uses_blocked_internal_marker(self):
    values = {
      "DAS_setSpeed": 50.0,
      "DAS_accState": 4,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": -1.0,
      "DAS_jerkMax": 0.5,
      "DAS_accelMin": -0.2,
      "DAS_accelMax": 0.3,
      "DAS_controlCounter": 6,
    }
    tesla_can = TeslaCAN(SimpleNamespace(flags=0), CANPacker("tesla_model3_party"))
    _, actual, _ = tesla_can.create_stock_longitudinal_handoff(values)
    aeb_event = actual[2] & 0x03

    self.assertEqual(3, aeb_event)

  def test_stock_lateral_handoff_uses_blocked_internal_marker(self):
    tesla_can = TeslaCAN(SimpleNamespace(flags=0), CANPacker("tesla_model3_party"))
    _, actual, _ = tesla_can.create_stock_lateral_handoff(12.5)

    self.assertEqual(7, (actual[2] >> 5) & 0x07)

    legacy_tesla_can = TeslaCAN(SimpleNamespace(flags=TeslaFlags.LEGACY_DAS_STEERING), CANPacker("tesla_model3_party"))
    _, legacy_actual, _ = legacy_tesla_can.create_stock_lateral_handoff(12.5)
    self.assertEqual(3, legacy_actual[2] >> 6)
