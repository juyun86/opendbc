import math

import pytest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_BUS, FILTER_SIGNAL_SPECS
from opendbc.sunnypilot.car.tesla.ars408.transmitter import ARS408Transmitter, MOTION_MAX_AGE_NS
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP


def transmitter(*, enabled: bool = True) -> ARS408Transmitter:
  cp_sp = structs.CarParamsSP()
  if enabled:
    cp_sp.flags |= TeslaFlagsSP.ARS408_RADAR
  return ARS408Transmitter(cp_sp)


def decode(message: tuple[int, bytes, int], name: str) -> dict[str, float]:
  parser = CANParser("ARS408", [(name, math.nan)], ARS408_BUS)
  parser.update([(1, [message])])
  return parser.vl[name]


@pytest.mark.parametrize(("field", "value", "valid", "signal"), [
  ("max_distance", 250, "RadarCfg_MaxDistance_valid", "RadarCfg_MaxDistance"),
  ("send_extended", 0, "RadarCfg_SendExtInfo_valid", "RadarCfg_SendExtInfo"),
  ("output_type", 1, "RadarCfg_OutputType_valid", "RadarCfg_OutputType"),
])
def test_radar_configuration_is_field_scoped(field: str, value: int, valid: str, signal: str) -> None:
  state = decode(transmitter().encode_radar_configuration(field, value), "RadarConfiguration")
  valid_signals = [name for name in state if name.endswith("_valid")]
  assert state[valid] == 1 and state[signal] == value
  assert all(state[name] == (1 if name == valid else 0) for name in valid_signals)


@pytest.mark.parametrize(("field", "value"), [
  ("max_distance", 199), ("max_distance", 251), ("max_distance", 201),
  ("output_type", 2), ("sensor_id", 0), ("store_nvm", 1),
])
def test_unsupported_radar_configuration_is_rejected(field: str, value: int) -> None:
  with pytest.raises(ValueError):
    transmitter().encode_radar_configuration(field, value)


def test_filter_configuration_and_query_are_bounded_object_records() -> None:
  tx = transmitter()
  state = decode(tx.encode_filter_configuration(1, True, 10.0, 200.0), "FilterCfg")
  assert (state["FilterCfg_Type"], state["FilterCfg_Index"], state["FilterCfg_Active"], state["FilterCfg_Valid"]) == (1, 1, 1, 1)
  assert state["FilterCfg_MinRaw"] == 100 and state["FilterCfg_MaxRaw"] == 2000
  query = decode(tx.encode_filter_query(8), "FilterCfg")
  assert (query["FilterCfg_Type"], query["FilterCfg_Index"], query["FilterCfg_Valid"]) == (1, 8, 0)
  with pytest.raises(ValueError):
    tx.encode_filter_configuration(15, True, 0.0, 1.0)
  with pytest.raises(ValueError):
    tx.encode_filter_configuration(1, True, math.nan, 1.0)


@pytest.mark.parametrize("index", sorted(FILTER_SIGNAL_SPECS))
def test_all_reviewed_filter_indices_are_encodable(index: int) -> None:
  spec = FILTER_SIGNAL_SPECS[index]
  minimum = 0.0 if index == 0 else spec.lower
  state = decode(transmitter().encode_filter_configuration(index, True, minimum, spec.upper), "FilterCfg")
  assert state["FilterCfg_Index"] == index


def test_motion_encoders_reject_nonfinite_and_operational_overflow() -> None:
  tx = transmitter()
  for speed in (-0.1, 85.01, math.inf, math.nan):
    with pytest.raises(ValueError):
      tx.encode_speed(speed, 1)
  with pytest.raises(ValueError):
    tx.encode_speed(10.0, 3)
  for yaw_rate in (-100.01, 100.01, math.inf, math.nan):
    with pytest.raises(ValueError):
      tx.encode_yaw_rate(yaw_rate)


def car_state(*, valid: bool = True, speed: float = 20.0, yaw_rate: float = 0.1,
              gear: structs.CarState.GearShifter = structs.CarState.GearShifter.drive,
              standstill: bool = False) -> structs.CarState:
  state = structs.CarState()
  state.canValid = valid
  state.vEgoRaw = speed
  state.yawRate = yaw_rate
  state.gearShifter = gear
  state.standstill = standstill
  return state


def test_periodic_motion_tx_uses_cached_enablement_direction_and_yaw_convention() -> None:
  tx = transmitter()
  tx.update_yaw_rate(0.1, 1_000_000_000)
  assert tx.update(1, car_state(), 1_000_000_000) == []
  sends = tx.update(5, car_state(gear=structs.CarState.GearShifter.reverse, yaw_rate=0.0), 1_000_000_000)
  assert [(address, len(data), bus) for address, data, bus in sends] == [(0x300, 2, 1), (0x301, 2, 1)]
  speed = decode(sends[0], "SpeedInformation")
  yaw = decode(sends[1], "YawRateInformation")
  assert speed["RadarDevice_SpeedDirection"] == 2 and speed["RadarDevice_Speed"] == 20.0
  assert yaw["RadarDevice_YawRate"] == pytest.approx(math.degrees(0.1), abs=0.01)


def test_motion_tx_is_atomic_and_fail_closed() -> None:
  tx = transmitter()
  tx.update_yaw_rate(0.1, 1_000_000_000)
  disabled = transmitter(enabled=False)
  disabled.update_yaw_rate(0.1, 1_000_000_000)
  assert disabled.update(5, car_state(), 1_000_000_000) == []
  assert tx.update(5, car_state(valid=False), 1_000_000_000) == []
  assert tx.update(5, car_state(speed=86.0), 1_000_000_000) == []
  standstill = tx.update(5, car_state(speed=0.0, standstill=True), 1_000_000_000)
  assert decode(standstill[0], "SpeedInformation")["RadarDevice_SpeedDirection"] == 0


@pytest.mark.parametrize("yaw", [None, math.nan, math.inf, -math.inf, math.radians(101), math.radians(-101)])
def test_invalid_source_never_falls_back_to_carstate_or_last_good_yaw(yaw) -> None:
  tx = transmitter()
  tx.update_yaw_rate(0.1, 1_000_000_000)
  assert len(tx.update(5, car_state(), 1_000_000_000)) == 2
  tx.update_yaw_rate(yaw, 1_000_000_000)
  assert tx.update(10, car_state(), 1_000_000_000) == []


def test_source_freshness_is_rechecked_at_send_time_and_recovers() -> None:
  tx = transmitter()
  assert tx.update(5, car_state(), 1_000_000_000) == []
  tx.update_yaw_rate(0.0, 1_000_000_000)
  assert tx.update(5, car_state(), 999_999_999) == []
  assert len(tx.update(5, car_state(), 1_000_000_000 + MOTION_MAX_AGE_NS)) == 2
  assert tx.update(10, car_state(), 1_000_000_001 + MOTION_MAX_AGE_NS) == []
  tx.update_yaw_rate(0.0, 2_000_000_000)
  assert tx.update(15, car_state(), 2_000_000_000)[1][1] == bytes.fromhex("8000")


@pytest.mark.parametrize(("inverted", "source_deg_s", "wire_hex"), [
  (False, 1.0, "7f9c"), (False, -1.0, "8064"), (False, 0.0, "8000"),
  (True, 1.0, "8064"), (True, -1.0, "7f9c"), (True, 0.0, "8000"),
])
def test_mounting_direction_controls_yaw_wire_bytes(inverted, source_deg_s, wire_hex) -> None:
  tx = ARS408Transmitter(structs.CarParamsSP(flags=TeslaFlagsSP.ARS408_RADAR), mount_inverted=inverted)
  tx.update_yaw_rate(math.radians(source_deg_s), 1_000_000_000)
  assert tx.update(5, car_state(yaw_rate=99.0), 1_000_000_000)[1][1].hex() == wire_hex
