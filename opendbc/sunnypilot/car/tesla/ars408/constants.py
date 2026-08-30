from dataclasses import dataclass
from enum import IntEnum, StrEnum


ARS408_DBC = "ARS408"
ARS408_BUS = 1
ARS408_SENSOR_ID = 0
ARS408_MAX_DISTANCE_M = 250.0
ARS408_MAX_OBJECTS = 100

# The protocol reports lateral quantities in the sensor coordinate frame. This
# integration reflects that axis into the openpilot frame. The same transform
# must be applied to lateral position, velocity, acceleration, and orientation.
# No installation-specific left/right capture currently verifies the physical
# sign, so consumers must not treat the sign as road-validated evidence.
SENSOR_TO_OPENPILOT_LATERAL_SIGN = -1.0
LATERAL_KINEMATICS_VERIFIED = False

ARS408_RADAR_CONFIG = 0x200
ARS408_RADAR_STATE = 0x201
ARS408_FILTER_CONFIG = 0x202
ARS408_FILTER_STATE_HEADER = 0x203
ARS408_FILTER_STATE_CONFIG = 0x204
ARS408_SPEED = 0x300
ARS408_YAW_RATE = 0x301
ARS408_STATUS = 0x60A
ARS408_GENERAL = 0x60B
ARS408_QUALITY = 0x60C
ARS408_EXTENDED = 0x60D

ARS408_RX_DLC: dict[int, int] = {
  ARS408_RADAR_STATE: 8,
  ARS408_FILTER_STATE_HEADER: 2,
  ARS408_FILTER_STATE_CONFIG: 5,
  ARS408_STATUS: 4,
  ARS408_GENERAL: 8,
  ARS408_QUALITY: 7,
  ARS408_EXTENDED: 8,
}


class TeslaRadarBackend(IntEnum):
  OEM = 0
  ARS408 = 1
  OFF = 2


class MeasurementState(IntEnum):
  DELETED = 0
  NEW = 1
  MEASURED = 2
  PREDICTED = 3
  DELETED_FOR_MERGE = 4
  NEW_FROM_MERGE = 5


class DynamicProperty(IntEnum):
  MOVING = 0
  STATIONARY = 1
  ONCOMING = 2
  STATIONARY_CANDIDATE = 3
  UNKNOWN = 4
  CROSSING_STATIONARY = 5
  CROSSING_MOVING = 6
  STOPPED = 7


class ObjectClass(IntEnum):
  POINT = 0
  CAR = 1
  TRUCK = 2
  PEDESTRIAN = 3
  MOTORCYCLE = 4
  BICYCLE = 5
  WIDE = 6
  RESERVED = 7


class MotionRxState(IntEnum):
  INPUT_OK = 0
  SPEED_MISSING = 1
  YAW_RATE_MISSING = 2
  SPEED_AND_YAW_RATE_MISSING = 3


RMS_INVALID_CODE = 0x1F
# Protocol RMS codes 0..30 select bin upper bounds (strictly ``value < bound``),
# not exact standard deviations. Code 31 means invalid/unavailable.
RMS_LINEAR_UPPER_BOUNDS = (
  0.005, 0.006, 0.008, 0.011, 0.014, 0.018, 0.023, 0.029,
  0.038, 0.049, 0.063, 0.081, 0.105, 0.135, 0.174, 0.224,
  0.288, 0.371, 0.478, 0.616, 0.794, 1.023, 1.317, 1.697,
  2.187, 2.817, 3.630, 4.676, 6.025, 7.762, 10.000,
)
RMS_ORIENTATION_UPPER_BOUNDS_DEG = (
  0.005, 0.007, 0.010, 0.014, 0.020, 0.029, 0.041, 0.058,
  0.082, 0.116, 0.165, 0.234, 0.332, 0.471, 0.669, 0.949,
  1.346, 1.909, 2.709, 3.843, 5.451, 7.734, 10.971, 15.565,
  22.081, 31.325, 44.439, 63.044, 89.437, 126.881, 180.000,
)


class RejectionReason(StrEnum):
  INVALID = "invalid"
  LOW_PROBABILITY = "low_probability"
  OUT_OF_RANGE = "out_of_range"
  STATIC_OUTSIDE_CORRIDOR = "static_outside_corridor"
  DUPLICATE = "duplicate"
  TIMEOUT = "timeout"


@dataclass(frozen=True, slots=True)
class FilterSignalSpec:
  suffix: str
  lower: float
  upper: float
  resolution: float
  offset: float = 0.0
  bits: int = 12


FILTER_SIGNAL_SPECS: dict[int, FilterSignalSpec] = {
  0: FilterSignalSpec("NofObj", 0.0, 100.0, 1.0),
  1: FilterSignalSpec("Distance", 0.0, 409.5, 0.1),
  2: FilterSignalSpec("Azimuth", -50.0, 52.375, 0.025, -50.0),
  3: FilterSignalSpec("VrelOncome", 0.0, 128.9925, 0.0315),
  4: FilterSignalSpec("VrelDepart", 0.0, 128.9925, 0.0315),
  5: FilterSignalSpec("RCS", -50.0, 52.375, 0.025, -50.0),
  6: FilterSignalSpec("Lifetime", 0.0, 409.5, 0.1),
  7: FilterSignalSpec("Size", 0.0, 102.375, 0.025),
  8: FilterSignalSpec("ProbExists", 0.0, 7.0, 1.0),
  9: FilterSignalSpec("Y", -409.5, 409.5, 0.2, -409.5),
  10: FilterSignalSpec("X", -500.0, 1138.2, 0.2, -500.0, 13),
  11: FilterSignalSpec("VYLeftRight", 0.0, 128.9925, 0.0315),
  12: FilterSignalSpec("VXOncome", 0.0, 128.9925, 0.0315),
  13: FilterSignalSpec("VYRightLeft", 0.0, 128.9925, 0.0315),
  14: FilterSignalSpec("VXDepart", 0.0, 128.9925, 0.0315),
}
