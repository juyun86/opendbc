from dataclasses import dataclass
from enum import IntEnum, StrEnum


ARS408_DBC = "ARS408"
ARS408_BUS = 1
ARS408_SENSOR_ID = 0
ARS408_MAX_DISTANCE_M = 250.0
ARS408_MAX_OBJECTS = 100
# User-confirmed 2026-09-24: lettering upside down. Roll is 180 degrees
# relative to the manufacturer's upright, forward-facing installation.
# Change this shared setting only after the physical installation is restored.
ARS408_MOUNT_INVERTED = True

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
ARS408_VERSION = 0x700

ARS408_RX_DLC: dict[int, int] = {
  ARS408_RADAR_STATE: 8,
  ARS408_FILTER_STATE_HEADER: 2,
  ARS408_FILTER_STATE_CONFIG: 5,
  ARS408_STATUS: 4,
  ARS408_GENERAL: 8,
  ARS408_QUALITY: 7,
  ARS408_EXTENDED: 8,
  ARS408_VERSION: 4,
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
