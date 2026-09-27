import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.tesla.coop_steering import CoopSteeringCarController
from opendbc.sunnypilot.car.tesla.dynamic_acc_debug import log_dynamic_acc
from opendbc.sunnypilot.car.tesla.ars408.transmitter import ARS408Transmitter
from opendbc.sunnypilot.car.tesla.speed_limit_controller import TeslaSpeedLimitController

SP_TAKEOVER_RAMP_FRAMES = 100
SP_TAKEOVER_ACCEL_RATE_UP = 0.6
SP_TAKEOVER_ACCEL_RATE_DOWN = 1.5


def get_safety_CP():
  # We use the TESLA_MODEL_Y platform for lateral limiting to match safety
  # A Model 3 at 40 m/s using the Model Y limits sees a <0.3% difference in max angle (from curvature factor)
  from opendbc.car.tesla.interface import CarInterface
  return CarInterface.get_non_essential_params("TESLA_MODEL_Y")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    self.coop_steer = CoopSteeringCarController()
    self.speed_limit_controller = TeslaSpeedLimitController(CP_SP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(CP, self.packer)
    self.ars408_transmitter = ARS408Transmitter(CP_SP)

    # Track longitudinal source transitions independently of the 25 Hz TX phase.
    self.prev_stock_longitudinal = False
    self.prev_stock_lateral = False
    self.leaving_stock_pending = False
    self.long_control_counter = None
    self.last_long_control_frame = -4
    self.dynamic_acc_debug_followup_frames = 0
    self.sp_takeover_accel = 0.0
    self.sp_takeover_ramp_frames = 0
    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []
    can_sends.extend(self.ars408_transmitter.update(self.frame, CS.out, now_nanos))
    speed_limit_sends = self.speed_limit_controller.update(CC, CS, now_nanos)
    can_sends.extend(speed_limit_sends)
    if speed_limit_sends:
      log_dynamic_acc(
        "carcontroller", "auto_speed_limit_tick", frame=self.frame,
        target_speed=float(CS.tesla_speed_limit_target),
        current_set_speed=float(CS.out.cruiseState.speedCluster),
        target_display_speed=self.speed_limit_controller.target_display,
        current_display_speed=self.speed_limit_controller.current_display,
        remaining_steps=self.speed_limit_controller.remaining_steps,
        data=speed_limit_sends[0].dat.hex(),
      )

    # Wait until the override condition clears before steering
    # Canceling is done on rising edge of CS.out.steeringDisengage and is handled generically with CC.cruiseControl.cancel
    lat_active = CC.latActive and not CS.out.steeringDisengage
    stock_lateral = bool(getattr(CS, "tesla_stock_lateral_active", False))
    entering_stock_lateral = stock_lateral and not self.prev_stock_lateral
    leaving_stock_lateral = not stock_lateral and self.prev_stock_lateral

    if entering_stock_lateral:
      lateral_handoff_msg = self.tesla_can.create_stock_lateral_handoff(CS.out.steeringAngleDeg)
      can_sends.append(lateral_handoff_msg)
      self._log_lateral_transition("entering_stock_lateral", CC, CS, handoff_msg=lateral_handoff_msg)
    elif leaving_stock_lateral:
      self.apply_angle_last = CS.out.steeringAngleDeg
      self.coop_steer.reset_override_state(self.apply_angle_last)
      self._log_lateral_transition("leaving_stock_lateral", CC, CS)

    if self.frame % 2 == 0 and not stock_lateral:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                          lat_active, CarControllerParams, self.VM)

      can_sends.append(self.tesla_can.create_steering_control(*self.coop_steer.update(self.apply_angle_last, lat_active, self.CP_SP, CS, self.VM)))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed())

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      entering_stock = CS.tesla_stock_longitudinal_active and not self.prev_stock_longitudinal
      leaving_stock = not CS.tesla_stock_longitudinal_active and self.prev_stock_longitudinal
      if entering_stock:
        # Safety consumes the marker as an internal handoff request; it is never
        # transmitted onto the vehicle bus.
        handoff_msg = self.tesla_can.create_stock_longitudinal_handoff(CS.das_control)
        can_sends.append(handoff_msg)
        self.dynamic_acc_debug_followup_frames = 100
        self._log_longitudinal_transition("entering_stock", CC, CS, handoff_msg=handoff_msg)
        self.leaving_stock_pending = False
        self.sp_takeover_ramp_frames = 0
      elif leaving_stock:
        self.leaving_stock_pending = True
        self.sp_takeover_accel = self._stock_takeover_accel(CS)
        self.sp_takeover_ramp_frames = SP_TAKEOVER_RAMP_FRAMES
        self.dynamic_acc_debug_followup_frames = 100
        self._log_longitudinal_transition("leaving_stock", CC, CS)

      long_control_due = (self.frame - self.last_long_control_frame) >= 4
      if long_control_due or self.leaving_stock_pending:
        # SP mode sends OP's own DAS_control. Stock mode lets panda forward the OEM DAS_control.
        if not CS.tesla_stock_longitudinal_active:
          state, accel = self._longitudinal_state_accel(
            self.leaving_stock_pending, self._cruise_enabled(CS), CC.longActive,
            CC.cruiseControl.cancel, self._limited_sp_takeover_accel(CC.longActive, actuators.accel),
          )
          cntr = self._next_long_control_counter(CS.das_control["DAS_controlCounter"], self.leaving_stock_pending)
          long_msg = self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive, CS.cruise_override)
          can_sends.append(long_msg)
          if self.leaving_stock_pending or self.dynamic_acc_debug_followup_frames > 0:
            self._log_longitudinal_transition("sp_command", CC, CS, state=state, accel=accel, counter=cntr, long_msg=long_msg)
          self.last_long_control_frame = self.frame
          self.leaving_stock_pending = False

      if self.dynamic_acc_debug_followup_frames > 0:
        self.dynamic_acc_debug_followup_frames -= 1

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if CC.cruiseControl.cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False, True))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = self.coop_steer.coop_apply_angle_sat_last # debug
    new_actuators.curvature = float(self.coop_steer.debug_angle_desired_limited) # debug
    new_actuators.torque = float(self.coop_steer.angle_override) # debug

    self.prev_stock_longitudinal = CS.tesla_stock_longitudinal_active
    self.prev_stock_lateral = stock_lateral
    self.frame += 1
    return new_actuators, can_sends

  def _next_long_control_counter(self, stock_counter, resync=False):
    if self.long_control_counter is None or resync:
      self.long_control_counter = int(stock_counter)
    self.long_control_counter = (self.long_control_counter + 1) % 8
    return self.long_control_counter

  @staticmethod
  def _longitudinal_state_accel(leaving_stock, cruise_enabled, long_active, cancel, requested_accel):
    if leaving_stock:
      state = 4 if cruise_enabled else 13
    else:
      state = 13 if cancel else 4
    accel = float(np.clip(requested_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)) if long_active else 0.0
    return state, accel

  @staticmethod
  def _cruise_enabled(CS):
    return CS.out.cruiseState.enabled

  @staticmethod
  def _stock_accel_midpoint(CS):
    das = CS.das_control
    return float(np.clip((float(das["DAS_accelMin"]) + float(das["DAS_accelMax"])) / 2.0,
                         CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

  @staticmethod
  def _stock_takeover_accel(CS):
    # The OEM min/max values are an allowed envelope, not the acceleration
    # currently applied to the vehicle. Start at measured acceleration so the
    # source edge does not briefly release or add braking.
    measured_accel = float(CS.out.aEgo)
    if not np.isfinite(measured_accel):
      return CarController._stock_accel_midpoint(CS)
    return float(np.clip(measured_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

  def _limited_sp_takeover_accel(self, long_active, requested_accel):
    requested_accel = float(np.clip(requested_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    if not long_active or self.sp_takeover_ramp_frames <= 0:
      self.sp_takeover_accel = requested_accel
      return requested_accel

    elapsed_frames = min(4, max(1, self.frame - self.last_long_control_frame))
    dt = elapsed_frames * DT_CTRL
    rate = SP_TAKEOVER_ACCEL_RATE_UP if requested_accel > self.sp_takeover_accel else SP_TAKEOVER_ACCEL_RATE_DOWN
    max_delta = rate * dt
    self.sp_takeover_accel = float(np.clip(requested_accel,
                                           self.sp_takeover_accel - max_delta,
                                           self.sp_takeover_accel + max_delta))
    self.sp_takeover_ramp_frames = max(0, self.sp_takeover_ramp_frames - elapsed_frames)
    if abs(self.sp_takeover_accel - requested_accel) < 1e-3:
      self.sp_takeover_ramp_frames = 0
    return self.sp_takeover_accel

  def _log_lateral_transition(self, event, CC, CS, **extra):
    can_payloads = {
      key: value[1].hex() if value is not None else None
      for key, value in extra.items() if key.endswith("_msg")
    }
    log_dynamic_acc(
      "carcontroller", event,
      frame=self.frame,
      stock_lateral_active=bool(getattr(CS, "tesla_stock_lateral_active", False)),
      previous_stock_lateral_active=self.prev_stock_lateral,
      cc_lat_active=CC.latActive,
      sp_steering_angle_request=CC.actuators.steeringAngleDeg,
      steering_angle=CS.out.steeringAngleDeg,
      steering_torque=CS.out.steeringTorque,
      oem_steering_angle_request=(getattr(CS, "das_steering_control", {}) or {}).get("DAS_steeringAngleRequest"),
      oem_steering_control_type=(getattr(CS, "das_steering_control", {}) or {}).get("DAS_steeringControlType"),
      **can_payloads,
    )

  def _log_longitudinal_transition(self, event, CC, CS, **extra):
    das = CS.das_control
    can_payloads = {
      key: value[1].hex() if value is not None else None
      for key, value in extra.items() if key.endswith("_msg")
    }
    values = {key: value for key, value in extra.items() if not key.endswith("_msg")}
    log_dynamic_acc(
      "carcontroller", event,
      frame=self.frame,
      python_stock_active=CS.tesla_stock_longitudinal_active,
      previous_stock_active=self.prev_stock_longitudinal,
      leaving_stock_pending=self.leaving_stock_pending,
      cc_enabled=CC.enabled,
      cc_long_active=CC.longActive,
      cc_cancel=CC.cruiseControl.cancel,
      cruise_enabled=self._cruise_enabled(CS),
      cruise_override=CS.cruise_override,
      ego_speed=CS.out.vEgo,
      requested_accel=CC.actuators.accel,
      sp_takeover_accel=self.sp_takeover_accel,
      sp_takeover_ramp_frames=self.sp_takeover_ramp_frames,
      das_acc_state=das.get("DAS_accState"),
      das_set_speed=das.get("DAS_setSpeed"),
      das_accel_min=das.get("DAS_accelMin"),
      das_accel_max=das.get("DAS_accelMax"),
      das_aeb_event=das.get("DAS_aebEvent"),
      das_counter=das.get("DAS_controlCounter"),
      **values,
      **can_payloads,
    )
