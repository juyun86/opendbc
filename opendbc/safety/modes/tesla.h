#pragma once

#include "opendbc/safety/declarations.h"

#define TESLA_COMMON_RX_CHECKS \
  {.msg = {{0x2b9, 2, 8, 25U, .max_counter = 7U, .ignore_quality_flag = true}, { 0 }, { 0 }}},    /* DAS_control */                                  \
  {.msg = {{0x488, 2, 4, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   /* DAS_steeringControl */                          \
  {.msg = {{0x257, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   /* DI_speed (speed in kph, gas pressed) */         \
  {.msg = {{0x155, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},                                /* ESP_B (2nd speed in kph) */                     \
  {.msg = {{0x370, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  /* EPAS3S_sysStatus (steering angle) */            \
  {.msg = {{0x145, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},                                /* ESP_status (brakes) */                          \
  {.msg = {{0x286, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   /* DI_state (acc state) */                         \
  {.msg = {{0x311, 0, 7, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   /* UI_warning (blinkers, buckle switch & doors) */ \

#define TESLA_VEHICLE_BUS_ADDR_CHECK \
  {.msg = {{0x3DF, 1, 8, 2U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},    /* UI_status2 */ \
  {.msg = {{0x3E9, 1, 8, 2U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},    /* DAS_bodyControls */ \

#define TESLA_SPEED_BUTTON_VALIDATION_RX_CHECK \
  {.msg = {{0x3C2, 1, 8, 2U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},   /* VCLEFT_switchStatus */ \

#define TESLA_STEERING_DISENGAGE_TORQUE 500 // cNm
#define TESLA_TURN_SIGNAL_SESSION_TIMEOUT_US 60000000U
#define TESLA_TURN_SIGNAL_SESSION_MAX_FRAMES 1200U
#define TESLA_GATEWAY_CONFIG_ADDR 0x6E0U
#define TESLA_GATEWAY_CONFIG_RESPONSE_ADDR 0x6E1U
#define TESLA_GATEWAY_CONFIG_MIN_INTERVAL_US 25000U
#define TESLA_GATEWAY_CONFIG_STATE_TIMEOUT_US 500000U

#define TESLA_STEERING_DISENGAGE_TORQUE 500 // cNm

static bool tesla_longitudinal = false;
static bool tesla_fsd_14 = false;
static bool tesla_stock_aeb = false;

// Only rising edges while controls are not allowed are considered for these systems:

// Car-initiated steering outside of autopilot:
// Lane Departure Avoidance, Emergency Lane Departure Avoidance, Autopark
static bool tesla_stock_steering_control = false;
static bool tesla_stock_steering_control_prev = false;

// Summon (includes Smart Summon)
// Only works while car is "off" when activated
// TODO: Fix when car is "on"
static bool tesla_summon = false;
static bool tesla_summon_prev = false;

// Detected VEHICLE bus
extern bool tesla_has_vehicle_bus;
bool tesla_has_vehicle_bus = false;

// Configured MADS screen button finger count (0 = disabled, 3-5 = expected touch-point count)
extern uint8_t tesla_mads_screen_button_fingers;
uint8_t tesla_mads_screen_button_fingers = 0U;

// Runtime stock longitudinal state. Python validates real 4-finger and dynamic
// requests, then changes this state through a safety-consumed handoff marker.
static bool tesla_stock_longitudinal_active = false;
static bool tesla_dynamic_auto_stock = false;
static bool tesla_ap_hybrid_handoff = false;
static bool tesla_ap_hybrid_lateral_handoff = false;
static bool tesla_ap_stock_lateral_active = false;
static bool tesla_turn_signal_validation = false;
static bool tesla_speed_button_validation = false;
static bool tesla_auto_speed_limit = false;
static bool tesla_ars408_radar = false;
static uint8_t tesla_turn_signal_active_state = 0U;
static uint16_t tesla_turn_signal_active_count = 0U;
static uint32_t tesla_turn_signal_session_timestamp = 0U;
static bool tesla_turn_signal_session_timed_out = false;
static bool tesla_turn_signal_rx_template_valid = false;
static uint8_t tesla_turn_signal_rx_template[8] = {0U};
static uint32_t tesla_turn_signal_rx_timestamp = 0U;
static bool tesla_speed_button_rx_template_valid = false;
static uint8_t tesla_speed_button_rx_template[8] = {0U};
static uint32_t tesla_speed_button_rx_timestamp = 0U;
static bool tesla_speed_button_last_tx_valid = false;
static uint32_t tesla_speed_button_last_tx_timestamp = 0U;

// Manual ambient-light test: fresh parked vehicle, one side, fixed red only.
static uint8_t tesla_ambient_template[8] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
static bool tesla_ambient_template_valid = false;
static bool tesla_ambient_park = false;
static bool tesla_ambient_stopped = false;
static bool tesla_ambient_tx_valid = false;
static uint32_t tesla_ambient_template_ts = 0U;
static uint32_t tesla_ambient_gear_ts = 0U;
static uint32_t tesla_ambient_speed_ts = 0U;
static uint32_t tesla_ambient_tx_ts = 0U;
static uint32_t tesla_ambient_session_ts = 0U;
static uint8_t tesla_ambient_session_count = 0U;
static bool tesla_gateway_config_tx_valid = false;
static uint32_t tesla_gateway_config_tx_ts = 0U;

static bool tesla_gateway_config_payload_valid(const CANPacket_t *msg) {
  const uint8_t command = msg->data[0];
  if ((command == 0x01U) || (command == 0x03U) || (command == 0x13U)) {
    for (uint8_t i = 2U; i < 8U; i++) if (msg->data[i] != 0U) return false;
    return true;
  }
  if (command == 0x02U) {
    if (msg->data[2] >= 48U) return false;
    for (uint8_t i = 3U; i < 8U; i++) if (msg->data[i] != 0U) return false;
    return true;
  }
  if (command == 0x10U) {
    return (msg->data[2] <= 48U) && (msg->data[3] == 0U);
  }
  if (command == 0x11U) {
    const uint16_t identifier = (uint16_t)msg->data[4] | ((uint16_t)msg->data[5] << 8U);
    return (msg->data[2] < 48U) && (msg->data[3] >= 2U) && (msg->data[3] <= 4U) &&
           (identifier <= 0x7FFU) && (identifier != TESLA_GATEWAY_CONFIG_ADDR) &&
           (identifier != TESLA_GATEWAY_CONFIG_RESPONSE_ADDR) &&
           ((msg->data[6] <= 8U) || (msg->data[6] == 0xFFU)) && (msg->data[7] <= 1U);
  }
  if (command == 0x12U) {
    return (msg->data[2] == 0U) && (msg->data[3] == 0U);
  }
  return false;
}

static uint8_t tesla_get_counter(const CANPacket_t *msg) {

  uint8_t cnt = 0;
  if (msg->addr == 0x2b9U) {
    // Signal: DAS_controlCounter
    cnt = msg->data[6] >> 5;
  } else if (msg->addr == 0x488U) {
    // Signal: DAS_steeringControlCounter
    cnt = msg->data[2] & 0x0FU;
  } else if ((msg->addr == 0x257U) || (msg->addr == 0x145U) || (msg->addr == 0x286U) || (msg->addr == 0x311U)) {
    // Signal: DI_speedCounter, ESP_statusCounter, DI_locStatusCounter, UI_warningCounter
    cnt = msg->data[1] & 0x0FU;
  } else if (msg->addr == 0x155U) {
    // Signal: ESP_wheelRotationCounter
    cnt = msg->data[6] >> 4;
  } else if (msg->addr == 0x370U) {
    // Signal: EPAS3S_sysStatusCounter
    cnt = msg->data[6] & 0x0FU;
  } else if (msg->addr == 0x3E9U) {
    // Signal: DAS_bodyControlsCounter
    cnt = msg->data[6] >> 4;
  } else {
  }
  return cnt;
}

static int _tesla_get_checksum_byte(const int addr) {
  int checksum_byte = -1;
  if ((addr == 0x370) || (addr == 0x2b9) || (addr == 0x155)) {
    // Signal: EPAS3S_sysStatusChecksum, DAS_controlChecksum, ESP_wheelRotationChecksum
    checksum_byte = 7;
  } else if (addr == 0x488) {
    // Signal: DAS_steeringControlChecksum
    checksum_byte = 3;
  } else if (addr == 0x3E9) {
    // Signal: DAS_bodyControlsChecksum
    checksum_byte = 7;
  } else if ((addr == 0x257) || (addr == 0x145) || (addr == 0x286) || (addr == 0x311)) {
    // Signal: DI_speedChecksum, ESP_statusChecksum, DI_locStatusChecksum, UI_warningChecksum
    checksum_byte = 0;
  } else {
  }
  return checksum_byte;
}

static uint32_t tesla_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  int checksum_byte = _tesla_get_checksum_byte(msg->addr);
  if (checksum_byte != -1) {
    chksum = msg->data[checksum_byte];
  }
  return chksum;
}

static uint32_t tesla_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  int checksum_byte = _tesla_get_checksum_byte(msg->addr);

  if (checksum_byte != -1) {
    chksum = (uint8_t)((msg->addr & 0xFFU) + ((msg->addr >> 8) & 0xFFU));
    int len = GET_LEN(msg);
    for (int i = 0; i < len; i++) {
      if (i != checksum_byte) {
        chksum += msg->data[i];
      }
    }
  }
  return chksum;
}

static bool tesla_get_quality_flag_valid(const CANPacket_t *msg) {

  bool valid = false;
  if (msg->addr == 0x155U) {
    valid = (msg->data[5] & 0x1U) == 0x1U;  // ESP_wheelSpeedsQF
  } else if (msg->addr == 0x145U) {
    int user_brake_status = (msg->data[3] >> 5) & 0x03U;
    valid = (user_brake_status != 0) && (user_brake_status != 3);  // ESP_driverBrakeApply=NotInit_orOff, Faulty_SNA
  } else {
  }
  return valid;
}

static int tesla_get_steer_ctrl_type(const int ctrl_type) {
  // Returns ANGLE_CONTROL-equivalent control type for FSD 14
  int steer_ctrl_type = ctrl_type;
  if (tesla_fsd_14) {
    if (ctrl_type == 1) {
      steer_ctrl_type = 2;
    } else if (ctrl_type == 2) {
      steer_ctrl_type = 1;
    } else {
    }
  }
  return steer_ctrl_type;
}

static void tesla_ambient_rx_observer(const CANPacket_t *msg) {
  if (!msg->extended && !msg->returned && !msg->rejected) {
    if ((GET_LEN(msg) == 7U) && (msg->bus == 1U) && (msg->addr == 0x679U)) {
      for (uint8_t i = 0U; i < 7U; i++) {
        tesla_ambient_template[i] = msg->data[i];
      }
      tesla_ambient_template_ts = microsecond_timer_get();
      tesla_ambient_template_valid = true;
    }
    if ((GET_LEN(msg) == 8U) && (msg->bus == 0U) && (msg->addr == 0x118U)) {
      uint8_t checksum = 0x19U;
      for (uint8_t i = 1U; i < 8U; i++) {
        checksum += msg->data[i];
      }
      tesla_ambient_park = (checksum == msg->data[0]) && (((msg->data[2] >> 5) & 7U) == 1U);
      tesla_ambient_gear_ts = microsecond_timer_get();
    }
  }
}

static void tesla_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Steering angle: (0.1 * val) - 819.2 in deg.
    if (msg->addr == 0x370U) {
      // Store it 1/10 deg to match steering request
      const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
      update_sample(&angle_meas, angle_meas_new);

      const int hands_on_level = msg->data[4] >> 6;  // EPAS3S_handsOnLevel
      const int torsion_bar_torque = (((msg->data[2] & 0x0FU) << 8) | msg->data[3]) - 2050;  // EPAS3S_torsionBarTorque in 0.01 Nm
      const int eac_status = msg->data[6] >> 5;  // EPAS3S_eacStatus
      const int eac_error_code = msg->data[2] >> 4;  // EPAS3S_eacErrorCode

      // Disengage on user override, or if high angle rate fault from user overriding extremely quickly
      steering_disengage = (hands_on_level >= 3) ||
                           (SAFETY_ABS(torsion_bar_torque) > TESLA_STEERING_DISENGAGE_TORQUE) ||
                           ((eac_status == 0) && (eac_error_code == 9));
    }

    // Vehicle speed (DI_speed)
    if (msg->addr == 0x257U) {
      tesla_ambient_stopped = ((((uint32_t)msg->data[2] << 4) | ((uint32_t)msg->data[1] >> 4)) == 500U);
      tesla_ambient_speed_ts = microsecond_timer_get();
      // Vehicle speed: ((val * 0.08) - 40) / MS_TO_KPH
      float speed = ((((msg->data[2] << 4) | (msg->data[1] >> 4)) * 0.08) - 40.) * KPH_TO_MS;
      UPDATE_VEHICLE_SPEED(speed);

      // Signal: DI_accelPedalPressed
      gas_pressed = GET_BIT(msg, 34U);
    }

    // 2nd vehicle speed (ESP_B)
    if (msg->addr == 0x155U) {
      // Disable controls if speeds from DI (Drive Inverter) and ESP ECUs are too far apart.
      float esp_speed = (((msg->data[6] & 0x0FU) << 6) | (msg->data[5] >> 2)) * 0.5 * KPH_TO_MS;
      speed_mismatch_check(esp_speed);
    }

    // Brake pressed
    if (msg->addr == 0x145U) {
      brake_pressed = ((msg->data[3] >> 5) & 0x03U) == 2U;
    }

    // Cruise and Summon state
    if (msg->addr == 0x286U) {
      // Summon state
      int autopark_state = (msg->data[3] >> 1) & 0x0FU;  // DI_autoparkState (used by Summon, not actually used by autopark)
      bool tesla_summon_now = (autopark_state == 3) ||  // ACTIVE
                                (autopark_state == 4) ||  // COMPLETE
                                (autopark_state == 9);    // SELFPARK_STARTED

      // Only consider rising edges while controls are not allowed
      if (tesla_summon_now && !tesla_summon_prev && !cruise_engaged_prev) {
        tesla_summon = true;
      }
      if (!tesla_summon_now) {
        tesla_summon = false;
      }
      tesla_summon_prev = tesla_summon_now;

      // Cruise state
      int cruise_state = (msg->data[1] >> 4) & 0x07U;
      bool cruise_engaged = (cruise_state == 2) ||  // ENABLED
                            (cruise_state == 3) ||  // STANDSTILL
                            (cruise_state == 4) ||  // OVERRIDE
                            (cruise_state == 6) ||  // PRE_FAULT
                            (cruise_state == 7);    // PRE_CANCEL
      cruise_engaged = cruise_engaged && !tesla_summon;

      pcm_cruise_check(cruise_engaged);
    }

    if (msg->addr == 0x155U) {
      vehicle_moving = !GET_BIT(msg, 41U);  // ESP_vehicleStandstillSts
    }
  }

  if (msg->bus == 1U) {
    // Preserve fresh, idle frames received from the vehicle. Validation TX is
    // only allowed to clone these frames and alter the documented request bits.
    if ((msg->addr == 0x3C2U) && (GET_LEN(msg) == 8U) &&
        ((msg->data[0] & 0x03U) == 1U) && ((msg->data[3] & 0x3FU) == 0U)) {
      for (uint8_t i = 0U; i < 8U; i++) {
        tesla_speed_button_rx_template[i] = msg->data[i];
      }
      tesla_speed_button_rx_timestamp = microsecond_timer_get();
      tesla_speed_button_rx_template_valid = true;
    }

    if ((msg->addr == 0x3E9U) && (GET_LEN(msg) == 8U) &&
        ((msg->data[1] & 0x03U) == 0U) && (tesla_compute_checksum(msg) == tesla_get_checksum(msg))) {
      for (uint8_t i = 0U; i < 8U; i++) {
        tesla_turn_signal_rx_template[i] = msg->data[i];
      }
      tesla_turn_signal_rx_timestamp = microsecond_timer_get();
      tesla_turn_signal_rx_template_valid = true;
    }

    if (msg->addr == 0x3DFU) {
      if (tesla_mads_screen_button_fingers != 0U) {
        mads_button_press = (msg->data[3] == tesla_mads_screen_button_fingers) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      }

    }
  }

  if (msg->bus == 2U) {
    // DAS_control
    if (msg->addr == 0x2b9U) {
      // "AEB_ACTIVE"
      tesla_stock_aeb = (msg->data[2] & 0x03U) == 1U;
    }

    // DAS_steeringControl
    if (msg->addr == 0x488U) {
      int steering_control_type = msg->data[2] >> 6;
      bool tesla_stock_steering_control_now = steering_control_type != 0;  // "NONE"

      // Only consider rising edges while controls are not allowed
      if (tesla_stock_steering_control_now && !tesla_stock_steering_control_prev && !(controls_allowed || controls_allowed_lateral)) {
        tesla_stock_steering_control = true;
      }
      if (!tesla_stock_steering_control_now) {
        tesla_stock_steering_control = false;
      }
      tesla_stock_steering_control_prev = tesla_stock_steering_control_now;
    }
  }
}


static bool tesla_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits TESLA_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // NOTE: based off TESLA_MODEL_Y to match openpilot
  const AngleSteeringParams TESLA_STEERING_PARAMS = {
    .slip_factor = -0.000580374383851451,  // calc_slip_factor(VM)
    .steer_ratio = 12.,
    .wheelbase = 2.89,
  };

  const LongitudinalLimits TESLA_LONG_LIMITS = {
    .max_accel = 425,       // 2 m/s^2
    .min_accel = 288,       // -3.48 m/s^2
    .inactive_accel = 375,  // 0. m/s^2
  };

  bool tx = true;
  bool violation = false;
  bool longitudinal_takeover = false;
  bool lateral_takeover = false;

  if (msg->addr == TESLA_GATEWAY_CONFIG_ADDR) {
    const uint32_t now = microsecond_timer_get();
    const bool state_fresh = (safety_get_ts_elapsed(now, tesla_ambient_gear_ts) <= TESLA_GATEWAY_CONFIG_STATE_TIMEOUT_US) &&
                             (safety_get_ts_elapsed(now, tesla_ambient_speed_ts) <= TESLA_GATEWAY_CONFIG_STATE_TIMEOUT_US);
    const bool rate_allowed = !tesla_gateway_config_tx_valid ||
                              (safety_get_ts_elapsed(now, tesla_gateway_config_tx_ts) >= TESLA_GATEWAY_CONFIG_MIN_INTERVAL_US);
    const bool safe_state = tesla_has_vehicle_bus && tesla_ambient_park && tesla_ambient_stopped &&
                            !vehicle_moving && !controls_allowed && !controls_allowed_lateral;
    if (!state_fresh || !rate_allowed || !safe_state || !tesla_gateway_config_payload_valid(msg)) {
      violation = true;
    } else {
      tesla_gateway_config_tx_valid = true;
      tesla_gateway_config_tx_ts = now;
    }
  }

  // Don't send any messages when Autopark is active
  if (tesla_summon) {
    violation = true;
  }

  // Steering control: (0.1 * val) - 1638.35 in deg.
  if (msg->addr == 0x488U) {
    // We use 1/10 deg as a unit here
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    const int angle_ctrl_type = tesla_get_steer_ctrl_type(1);
    const int lkas_ctrl_type = tesla_get_steer_ctrl_type(2);
    bool steer_control_enabled = (steer_control_type == angle_ctrl_type) ||  // ANGLE_CONTROL
                                 (steer_control_type == lkas_ctrl_type);     // LANE_KEEP_ASSIST

    // Control type 3 is reserved as an internal AP lateral handoff marker.
    // It opens OEM steering forwarding and is never transmitted to the vehicle.
    if (tesla_ap_hybrid_lateral_handoff && (steer_control_type == 3)) {
      tesla_ap_stock_lateral_active = true;
      return false;
    }
    lateral_takeover = tesla_ap_stock_lateral_active;

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, TESLA_STEERING_LIMITS, TESLA_STEERING_PARAMS)) {
      violation = true;
    }

    bool valid_steer_control_type = (steer_control_type == 0) ||  // NONE
                                    (steer_control_type == angle_ctrl_type) ||  // ANGLE_CONTROL
                                    (steer_control_type == lkas_ctrl_type);     // LANE_KEEP_ASSIST
    if (!valid_steer_control_type) {
      violation = true;
    }

    if (tesla_stock_steering_control) {
      // Don't allow any steering commands when stock LKAS is active
      violation = true;
    }
  }

  // DAS_control: longitudinal control message
  if (msg->addr == 0x2b9U) {
    int aeb_event = msg->data[2] & 0x03U;

    // AEB event 3 is reserved as an internal handoff marker. It is always
    // consumed here and never reaches the vehicle. The first normal OP frame
    // atomically closes OEM forwarding before it is transmitted.
    if ((tesla_has_vehicle_bus || tesla_dynamic_auto_stock || tesla_ap_hybrid_handoff) && (aeb_event == 3)) {
      tesla_stock_longitudinal_active = true;
      return false;
    }
    longitudinal_takeover = tesla_stock_longitudinal_active && (aeb_event == 0);

    // No AEB events may be sent by openpilot
    if (aeb_event != 0) {
      violation = true;
    }

    // Don't send long/cancel messages when the stock AEB system is active
    if (tesla_stock_aeb) {
      violation = true;
    }

    int raw_accel_max = ((msg->data[6] & 0x1FU) << 4) | (msg->data[5] >> 4);
    int raw_accel_min = ((msg->data[5] & 0x0FU) << 5) | (msg->data[4] >> 3);
    int acc_state = msg->data[1] >> 4;

    if (tesla_longitudinal) {
      // Stock mode echoes Tesla's own DAS_control values via TX channel.
      // These are inherently safe — skip accel checks so hard braking etc. passes.
      if (!tesla_stock_longitudinal_active || longitudinal_takeover) {
        // SP mode: OP's own DAS_control — enforce safety limits.
        if ((raw_accel_max < TESLA_LONG_LIMITS.inactive_accel) && (raw_accel_min < TESLA_LONG_LIMITS.inactive_accel)) {
          violation = true;
        }
        violation |= longitudinal_accel_checks(raw_accel_max, TESLA_LONG_LIMITS);
        violation |= longitudinal_accel_checks(raw_accel_min, TESLA_LONG_LIMITS);
      }
    } else {
      // Can only send cancel longitudinal messages when not controlling longitudinal
      if (acc_state != 13) {  // ACC_CANCEL_GENERIC_SILENT
        violation = true;
      }

      // No actuation is allowed when not controlling longitudinal
      if ((raw_accel_max != TESLA_LONG_LIMITS.inactive_accel) || (raw_accel_min != TESLA_LONG_LIMITS.inactive_accel)) {
        violation = true;
      }
    }
  }

  if (msg->addr == 0x679U) {
    const uint32_t now = microsecond_timer_get();
    const bool fresh = tesla_ambient_template_valid &&
      (safety_get_ts_elapsed(now, tesla_ambient_template_ts) <= 1000000U);
    const bool target = (((msg->data[5] & 0xF8U) == 0xA8U) && ((msg->data[6] & 1U) == 0U)) ||
                        (((msg->data[5] & 0xF8U) == 0x50U) && ((msg->data[6] & 1U) == 1U)) ||
                        (((msg->data[5] & 0xF8U) == 0xF8U) && ((msg->data[6] & 1U) == 1U));
    const uint8_t brightness = msg->data[4] & 0x7FU;
    // Only the requested blindspot palette/brightness, plus the existing red test.
    const bool alert_color = (msg->data[1] == 255U) && (msg->data[3] == 0U) &&
      (((msg->data[2] == 0U) && ((brightness == 0U) || (brightness == 50U) || (brightness == 90U) || (brightness == 100U))) ||
       ((msg->data[2] == 190U) && ((brightness == 50U) || (brightness == 90U))));
    const bool fixed_alert = (msg->data[0] == ((tesla_ambient_template[0] & 1U) | 2U)) && alert_color &&
      ((msg->data[4] & 0x80U) == (tesla_ambient_template[4] & 0x80U)) &&
      ((msg->data[5] & 7U) == (tesla_ambient_template[5] & 6U)) &&
      ((msg->data[6] & 0xFEU) == (tesla_ambient_template[6] & 0xFEU));
    const bool new_session = !tesla_ambient_tx_valid || (safety_get_ts_elapsed(now, tesla_ambient_tx_ts) >= 1000000U);
    // Host sends at 10 Hz; retain the 150-frame / 15-second accessory session cap.
    const bool rate = !tesla_ambient_tx_valid || (safety_get_ts_elapsed(now, tesla_ambient_tx_ts) >= 80000U);
    const bool session = new_session || ((safety_get_ts_elapsed(now, tesla_ambient_session_ts) < 15000000U) &&
                                        (tesla_ambient_session_count < 150U));
    if (!tesla_has_vehicle_bus || !fresh || !target || !fixed_alert || !rate || !session) {
      violation = true;
    }
    if (!violation) {
      if (new_session) {
        tesla_ambient_session_ts = now;
        tesla_ambient_session_count = 0U;
      }
      tesla_ambient_session_count++;
      tesla_ambient_tx_valid = true;
      tesla_ambient_tx_ts = now;
    }
  }

  // Right scroll wheel: one signed tick cloned from a fresh vehicle RX frame.
  if (msg->addr == 0x3C2U) {
    const uint8_t right_scroll_ticks = msg->data[3] & 0x3FU;
    const bool validation_tick_allowed = (right_scroll_ticks == 1U) || (right_scroll_ticks == 0x3FU);
    bool validation_template_matches = (msg->data[3] & 0xC0U) == (tesla_speed_button_rx_template[3] & 0xC0U);
    for (uint8_t i = 0U; i < 8U; i++) {
      if (i != 3U) {
        validation_template_matches &= msg->data[i] == tesla_speed_button_rx_template[i];
      }
    }
    const bool validation_template_fresh = tesla_speed_button_rx_template_valid &&
      (safety_get_ts_elapsed(microsecond_timer_get(), tesla_speed_button_rx_timestamp) <= 1500000U);
    const bool validation_rate_allowed = !tesla_speed_button_last_tx_valid ||
      (safety_get_ts_elapsed(microsecond_timer_get(), tesla_speed_button_last_tx_timestamp) >= 250000U);
    const bool validation_mode_allowed = tesla_speed_button_validation;
    const bool automatic_mode_allowed = tesla_auto_speed_limit && controls_allowed;
    const bool validation_button_valid = tesla_has_vehicle_bus && (validation_mode_allowed || automatic_mode_allowed) &&
      validation_tick_allowed && validation_template_matches && validation_template_fresh && validation_rate_allowed;
    if (!validation_button_valid) {
      violation = true;
    } else {
      tesla_speed_button_last_tx_valid = true;
      tesla_speed_button_last_tx_timestamp = microsecond_timer_get();
    }
  }

  // DAS body-control turn request cloned from a fresh OEM idle frame.
  if (msg->addr == 0x3E9U) {
    const uint32_t now = microsecond_timer_get();
    const uint8_t turn_request = msg->data[1] & 0x03U;
    const uint8_t turn_reason = (msg->data[2] >> 1) & 0x0FU;
    const bool active_turn = (turn_request == 1U) || (turn_request == 2U);
    const bool request_reason_valid = (active_turn && (turn_reason == 8U)) ||
                                      ((turn_request == 3U) && (turn_reason == 4U));
    bool validation_template_matches = (msg->data[0] == tesla_turn_signal_rx_template[0]) &&
                                       ((msg->data[1] & 0xFCU) == (tesla_turn_signal_rx_template[1] & 0xFCU)) &&
                                       ((msg->data[2] & 0xE1U) == (tesla_turn_signal_rx_template[2] & 0xE1U)) &&
                                       (msg->data[3] == tesla_turn_signal_rx_template[3]) &&
                                       (msg->data[4] == tesla_turn_signal_rx_template[4]) &&
                                       (msg->data[5] == tesla_turn_signal_rx_template[5]) &&
                                       ((msg->data[6] & 0x0FU) == (tesla_turn_signal_rx_template[6] & 0x0FU));
    const uint8_t expected_counter = ((tesla_turn_signal_rx_template[6] >> 4) + 1U) & 0x0FU;
    validation_template_matches &= (msg->data[6] >> 4) == expected_counter;
    const bool validation_template_fresh = tesla_turn_signal_rx_template_valid &&
      (safety_get_ts_elapsed(now, tesla_turn_signal_rx_timestamp) <= 1500000U);
    if ((tesla_turn_signal_active_state != 0U) &&
        (safety_get_ts_elapsed(now, tesla_turn_signal_session_timestamp) > TESLA_TURN_SIGNAL_SESSION_TIMEOUT_US)) {
      tesla_turn_signal_session_timed_out = true;
    }
    const bool transition_valid = active_turn ?
      (((tesla_turn_signal_active_state == 0U) && !tesla_turn_signal_session_timed_out) ||
       ((tesla_turn_signal_active_state == turn_request) && !tesla_turn_signal_session_timed_out &&
        (tesla_turn_signal_active_count < TESLA_TURN_SIGNAL_SESSION_MAX_FRAMES))) :
      (tesla_turn_signal_active_state != 0U);
    const bool checksum_valid = tesla_compute_checksum(msg) == tesla_get_checksum(msg);
    const bool valid = tesla_has_vehicle_bus && tesla_turn_signal_validation && request_reason_valid &&
                       validation_template_matches && validation_template_fresh && transition_valid && checksum_valid;
    if (!valid) {
      violation = true;
    } else if (active_turn) {
      if (tesla_turn_signal_active_state == 0U) {
        tesla_turn_signal_session_timestamp = now;
      }
      tesla_turn_signal_active_state = turn_request;
      tesla_turn_signal_active_count++;
    } else {
      tesla_turn_signal_active_state = 0U;
      tesla_turn_signal_active_count = 0U;
      tesla_turn_signal_session_timestamp = 0U;
      tesla_turn_signal_session_timed_out = false;
    }
    // Every validation TX consumes exactly one OEM template. A subsequent
    // action or cancel must wait for another real vehicle frame.
    if (valid) {
      tesla_turn_signal_rx_template_valid = false;
    }
  }

  // Extra TX messages: basic safety checks
  if (msg->addr == 0x370U) {
    // Nag killer echo: verify checksum
    uint8_t chksum = tesla_compute_checksum(msg);
    if (chksum != tesla_get_checksum(msg)) {
      violation = true;
    }
  }
  if (msg->addr == 0x399U) {
    // ISA speed chime: verify checksum
    uint8_t chksum = tesla_compute_checksum(msg);
    if (chksum != tesla_get_checksum(msg)) {
      violation = true;
    }
  }
  if (msg->addr == 0x082U) {
    // Precondition: only allow byte0 = 0x05
    if ((msg->data[0] & 0x05U) != 0x05U) {
      violation = true;
    }
  }

  // Continental ARS408 ego-motion input. The TX allowlist independently
  // constrains these messages to bus 1 with exactly two data bytes.
  if (msg->addr == 0x300U) {
    const uint8_t direction = msg->data[0] >> 6;
    const uint16_t speed_raw = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    const bool direction_matches_speed = ((direction == 0U) && (speed_raw == 0U)) ||
                                         ((direction > 0U) && (direction <= 2U) && (speed_raw > 0U));
    if (!tesla_ars408_radar || ((msg->data[0] & 0x20U) != 0U) || (speed_raw > 4250U) || !direction_matches_speed) {
      violation = true;
    }
  }
  if (msg->addr == 0x301U) {
    const uint16_t yaw_rate_raw = (msg->data[0] << 8) | msg->data[1];
    if (!tesla_ars408_radar || (yaw_rate_raw < 22768U) || (yaw_rate_raw > 42768U)) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }

  if (longitudinal_takeover && tx) {
    tesla_stock_longitudinal_active = false;
  }
  if (lateral_takeover && tx) {
    tesla_ap_stock_lateral_active = false;
  }

  return tx;
}

static bool tesla_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == 2) {
    if (!tesla_summon) {
      // APS_eacMonitor
      if (addr == 0x27d) {
        block_msg = true;
      }

      // DAS_steeringControl
      if ((addr == 0x488) && !tesla_stock_steering_control && !tesla_ap_stock_lateral_active) {
        block_msg = true;
      }

      // DAS_control - block OEM longitudinal only while SP/openpilot longitudinal is active.
      // In stock longitudinal mode, forward the OEM DAS_control instead of reconstructing it in Python.
      if (tesla_longitudinal && (addr == 0x2b9) && !tesla_stock_aeb && !tesla_stock_longitudinal_active) {
        block_msg = true;
      }
    }
  }

  return block_msg;
}

static safety_config tesla_init(uint16_t param) {

  static const CanMsg TESLA_M3_Y_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},   // DAS_steeringControl
    {0x2b9, 0, 8, .check_relay = false},                                   // DAS_control (for cancel)
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},   // APS_eacMonitor
    {0x082, 0, 8, .check_relay = false, .disable_static_blocking = true},  // UI_tripPlanning (precondition)
    {0x3FD, 0, 8, .check_relay = false, .disable_static_blocking = true},  // UI_autopilotControl (FSD unlock)
    {0x370, 0, 8, .check_relay = false, .disable_static_blocking = true},  // EPAS3S_sysStatus (nag killer)
    {0x399, 0, 8, .check_relay = false, .disable_static_blocking = true},  // ISA speed chime suppress
    {0x3E9, 1, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_bodyControls (validation only)
    {0x3C2, 1, 8, .check_relay = false, .disable_static_blocking = true},  // VCLEFT_switchStatus (validation only)
    {0x679, 1, 7, .check_relay = false, .disable_static_blocking = true}, // Fixed red ambient test, parked only
    {TESLA_GATEWAY_CONFIG_ADDR, 1, 8, .check_relay = false, .disable_static_blocking = true},
  };

  static const CanMsg TESLA_M3_Y_LONG_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2b9, 0, 8, .check_relay = true, .disable_static_blocking = true},  // DAS_control
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},  // APS_eacMonitor
    {0x082, 0, 8, .check_relay = false, .disable_static_blocking = true}, // UI_tripPlanning (precondition)
    {0x3FD, 0, 8, .check_relay = false, .disable_static_blocking = true}, // UI_autopilotControl (FSD unlock)
    {0x370, 0, 8, .check_relay = false, .disable_static_blocking = true}, // EPAS3S_sysStatus (nag killer)
    {0x399, 0, 8, .check_relay = false, .disable_static_blocking = true}, // ISA speed chime suppress
    {0x3E9, 1, 8, .check_relay = false, .disable_static_blocking = true}, // DAS_bodyControls (validation only)
    {0x3C2, 1, 8, .check_relay = false, .disable_static_blocking = true}, // VCLEFT_switchStatus (validation only)
    {0x679, 1, 7, .check_relay = false, .disable_static_blocking = true}, // Fixed red ambient test, parked only
    {TESLA_GATEWAY_CONFIG_ADDR, 1, 8, .check_relay = false, .disable_static_blocking = true},
  };

  static const CanMsg TESLA_M3_Y_ARS408_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},
    {0x2b9, 0, 8, .check_relay = false},
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},
    {0x082, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x3FD, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x370, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x399, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x3E9, 1, 8, .check_relay = false, .disable_static_blocking = true},
    {0x3C2, 1, 8, .check_relay = false, .disable_static_blocking = true},
    {0x679, 1, 7, .check_relay = false, .disable_static_blocking = true}, // Fixed red ambient test, parked only
    {TESLA_GATEWAY_CONFIG_ADDR, 1, 8, .check_relay = false, .disable_static_blocking = true},
    {0x300, 1, 2, .check_relay = false, .disable_static_blocking = true},
    {0x301, 1, 2, .check_relay = false, .disable_static_blocking = true},
  };

  static const CanMsg TESLA_M3_Y_LONG_ARS408_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = true, .disable_static_blocking = true},
    {0x2b9, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {0x27D, 0, 3, .check_relay = true, .disable_static_blocking = true},
    {0x082, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x3FD, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x370, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x399, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x3E9, 1, 8, .check_relay = false, .disable_static_blocking = true},
    {0x3C2, 1, 8, .check_relay = false, .disable_static_blocking = true},
    {0x679, 1, 7, .check_relay = false, .disable_static_blocking = true}, // Fixed red ambient test, parked only
    {TESLA_GATEWAY_CONFIG_ADDR, 1, 8, .check_relay = false, .disable_static_blocking = true},
    {0x300, 1, 2, .check_relay = false, .disable_static_blocking = true},
    {0x301, 1, 2, .check_relay = false, .disable_static_blocking = true},
  };

  const uint16_t TESLA_FLAG_FSD_14 = 2;
  tesla_fsd_14 = GET_FLAG(param, TESLA_FLAG_FSD_14);

#ifdef ALLOW_DEBUG
  const uint16_t TESLA_FLAG_LONGITUDINAL_CONTROL = 1;
  tesla_longitudinal = GET_FLAG(param, TESLA_FLAG_LONGITUDINAL_CONTROL);
#endif

  const uint16_t TESLA_PARAM_SP_VEHICLE_BUS = 1;
  const uint16_t TESLA_PARAM_SP_MADS_SCREEN_BUTTON_3_FINGER = 2;
  const uint16_t TESLA_PARAM_SP_MADS_SCREEN_BUTTON_5_FINGER = 8;
  const uint16_t TESLA_PARAM_SP_DYNAMIC_AUTO_STOCK = 16;
  const uint16_t TESLA_PARAM_SP_AP_HYBRID_HANDOFF = 64;
  const uint16_t TESLA_PARAM_SP_AP_HYBRID_LATERAL_HANDOFF = 128;
  const uint16_t TESLA_PARAM_SP_TURN_SIGNAL_VALIDATION = 256;
  const uint16_t TESLA_PARAM_SP_SPEED_BUTTON_VALIDATION = 512;
  const uint16_t TESLA_PARAM_SP_AUTO_SPEED_LIMIT = 1024;
  const uint16_t TESLA_PARAM_SP_ARS408_RADAR = 2048;

  tesla_has_vehicle_bus = GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_VEHICLE_BUS);

  if (GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_MADS_SCREEN_BUTTON_3_FINGER)) {
    tesla_mads_screen_button_fingers = 3U;
  } else if (GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_MADS_SCREEN_BUTTON_5_FINGER)) {
    tesla_mads_screen_button_fingers = 5U;
  } else {
    tesla_mads_screen_button_fingers = 0U;
  }

  tesla_dynamic_auto_stock = GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_DYNAMIC_AUTO_STOCK);
  tesla_ap_hybrid_handoff = GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_AP_HYBRID_HANDOFF);
  tesla_ap_hybrid_lateral_handoff = GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_AP_HYBRID_LATERAL_HANDOFF);
  tesla_turn_signal_validation = GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_TURN_SIGNAL_VALIDATION);
  tesla_speed_button_validation = GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_SPEED_BUTTON_VALIDATION);
  tesla_auto_speed_limit = GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_AUTO_SPEED_LIMIT);
  tesla_ars408_radar = GET_FLAG(current_safety_param_sp, TESLA_PARAM_SP_ARS408_RADAR);

  tesla_ambient_template_valid = false;
  tesla_ambient_park = false;
  tesla_ambient_stopped = false;
  tesla_ambient_tx_valid = false;
  tesla_ambient_template_ts = 0U;
  tesla_ambient_gear_ts = 0U;
  tesla_ambient_speed_ts = 0U;
  tesla_ambient_tx_ts = 0U;
  tesla_ambient_session_ts = 0U;
  tesla_ambient_session_count = 0U;
  tesla_gateway_config_tx_valid = false;
  tesla_gateway_config_tx_ts = 0U;
  tesla_stock_aeb = false;
  tesla_stock_steering_control = false;
  tesla_stock_steering_control_prev = false;
  tesla_stock_longitudinal_active = false;
  tesla_ap_stock_lateral_active = false;
  tesla_turn_signal_active_state = 0U;
  tesla_turn_signal_active_count = 0U;
  tesla_turn_signal_session_timestamp = 0U;
  tesla_turn_signal_session_timed_out = false;
  tesla_turn_signal_rx_template_valid = false;
  tesla_turn_signal_rx_timestamp = 0U;
  tesla_speed_button_rx_template_valid = false;
  tesla_speed_button_rx_timestamp = 0U;
  tesla_speed_button_last_tx_valid = false;
  tesla_speed_button_last_tx_timestamp = 0U;
  for (uint8_t i = 0U; i < 8U; i++) {
    tesla_turn_signal_rx_template[i] = 0U;
    tesla_speed_button_rx_template[i] = 0U;
  }
  // we need to assume Autopark/Summon on startup since DI_state is a low freq msg.
  // this is so that we don't fault if starting while these systems are active
  tesla_summon = true;
  tesla_summon_prev = false;

  static RxCheck tesla_model3_y_rx_checks[] = {
    TESLA_COMMON_RX_CHECKS
  };

  static RxCheck tesla_model3_y_vehicle_bus_rx_checks[] = {
    TESLA_COMMON_RX_CHECKS
    TESLA_VEHICLE_BUS_ADDR_CHECK
  };

  static RxCheck tesla_model3_y_speed_button_validation_rx_checks[] = {
    TESLA_COMMON_RX_CHECKS
    TESLA_SPEED_BUTTON_VALIDATION_RX_CHECK
  };

  static RxCheck tesla_model3_y_vehicle_bus_speed_button_validation_rx_checks[] = {
    TESLA_COMMON_RX_CHECKS
    TESLA_VEHICLE_BUS_ADDR_CHECK
    TESLA_SPEED_BUTTON_VALIDATION_RX_CHECK
  };

  safety_config ret;
  if (tesla_longitudinal && tesla_ars408_radar) {
    SET_TX_MSGS(TESLA_M3_Y_LONG_ARS408_TX_MSGS, ret);
  } else if (tesla_longitudinal) {
    SET_TX_MSGS(TESLA_M3_Y_LONG_TX_MSGS, ret);
  } else if (tesla_ars408_radar) {
    SET_TX_MSGS(TESLA_M3_Y_ARS408_TX_MSGS, ret);
  } else {
    SET_TX_MSGS(TESLA_M3_Y_TX_MSGS, ret);
  }

  if (tesla_has_vehicle_bus && (tesla_speed_button_validation || tesla_auto_speed_limit)) {
    SET_RX_CHECKS(tesla_model3_y_vehicle_bus_speed_button_validation_rx_checks, ret);
  } else if (tesla_speed_button_validation || tesla_auto_speed_limit) {
    SET_RX_CHECKS(tesla_model3_y_speed_button_validation_rx_checks, ret);
  } else if (tesla_has_vehicle_bus) {
    SET_RX_CHECKS(tesla_model3_y_vehicle_bus_rx_checks, ret);
  } else {
    SET_RX_CHECKS(tesla_model3_y_rx_checks, ret);
  }
  return ret;
}

const safety_hooks tesla_hooks = {
  .init = tesla_init,
  .rx = tesla_rx_hook,
  .rx_observer = tesla_ambient_rx_observer,
  .tx = tesla_tx_hook,
  .fwd = tesla_fwd_hook,
  .get_counter = tesla_get_counter,
  .get_checksum = tesla_get_checksum,
  .compute_checksum = tesla_compute_checksum,
  .get_quality_flag_valid = tesla_get_quality_flag_valid,
};
