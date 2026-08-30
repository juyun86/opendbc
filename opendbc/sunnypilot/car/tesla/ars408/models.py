import math
from dataclasses import dataclass

from opendbc.sunnypilot.car.tesla.ars408.constants import (
  ARS408_MAX_OBJECTS, MeasurementState, RMS_INVALID_CODE, RMS_LINEAR_UPPER_BOUNDS, RMS_ORIENTATION_UPPER_BOUNDS_DEG,
)


_VALID_MEASUREMENT_STATES = frozenset(state.value for state in MeasurementState)
_PROTOCOL_FLOAT_EPSILON = 1e-9


def _finite(*values: float) -> None:
  if not all(math.isfinite(value) for value in values):
    raise ValueError("ARS408 values must be finite")


def _raw_id(raw_id: int) -> None:
  if not 0 <= raw_id <= 255:
    raise ValueError(f"invalid ARS408 raw object ID: {raw_id}")


def _in_protocol_range(value: float, lower: float, upper: float) -> bool:
  return lower - _PROTOCOL_FLOAT_EPSILON <= value <= upper + _PROTOCOL_FLOAT_EPSILON


@dataclass(frozen=True, slots=True)
class ObjectStatus:
  object_count: int
  measurement_counter: int
  interface_version: int

  def __post_init__(self) -> None:
    if not 0 <= self.object_count <= 255:
      raise ValueError(f"invalid ARS408 object count: {self.object_count}")
    if not 0 <= self.measurement_counter <= 65535:
      raise ValueError(f"invalid ARS408 measurement counter: {self.measurement_counter}")
    if not 0 <= self.interface_version <= 15:
      raise ValueError(f"invalid ARS408 interface version: {self.interface_version}")

  @property
  def protocol_valid(self) -> bool:
    return self.object_count <= ARS408_MAX_OBJECTS and self.interface_version == 1


@dataclass(frozen=True, slots=True)
class ObjectGeneral:
  raw_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  rcs: float
  dynamic_property: int

  def __post_init__(self) -> None:
    _raw_id(self.raw_id)
    _finite(self.d_rel, self.y_rel, self.v_rel, self.yv_rel, self.rcs)
    if not 0 <= self.dynamic_property <= 7:
      raise ValueError(f"invalid ARS408 dynamic property: {self.dynamic_property}")


@dataclass(frozen=True, slots=True)
class ObjectQuality:
  raw_id: int
  probability: int
  measurement_state: int
  dist_long_rms: int = RMS_INVALID_CODE
  vrel_long_rms: int = RMS_INVALID_CODE
  dist_lat_rms: int = RMS_INVALID_CODE
  vrel_lat_rms: int = RMS_INVALID_CODE
  arel_lat_rms: int = RMS_INVALID_CODE
  arel_long_rms: int = RMS_INVALID_CODE
  orientation_rms: int = RMS_INVALID_CODE

  def __post_init__(self) -> None:
    _raw_id(self.raw_id)
    if not 0 <= self.probability <= 7:
      raise ValueError(f"invalid ARS408 existence probability: {self.probability}")
    if self.measurement_state not in _VALID_MEASUREMENT_STATES:
      raise ValueError(f"invalid ARS408 measurement state: {self.measurement_state}")
    if not all(0 <= code <= RMS_INVALID_CODE for code in self.rms_codes):
      raise ValueError(f"invalid ARS408 RMS code: {self.rms_codes}")

  @property
  def rms_codes(self) -> tuple[int, ...]:
    return (
      self.dist_long_rms, self.vrel_long_rms, self.dist_lat_rms, self.vrel_lat_rms,
      self.arel_lat_rms, self.arel_long_rms, self.orientation_rms,
    )

  @property
  def rms_upper_bounds(self) -> tuple[float | None, ...]:
    linear = tuple(None if code == RMS_INVALID_CODE else RMS_LINEAR_UPPER_BOUNDS[code] for code in self.rms_codes[:-1])
    orientation = None if self.orientation_rms == RMS_INVALID_CODE else RMS_ORIENTATION_UPPER_BOUNDS_DEG[self.orientation_rms]
    return (*linear, orientation)

  @property
  def rms_sigmas(self) -> tuple[float | None, ...]:
    """Compatibility alias; protocol values are bin upper bounds, not exact sigmas."""
    return self.rms_upper_bounds


@dataclass(frozen=True, slots=True)
class ObjectExtended:
  raw_id: int
  a_rel: float
  object_class: int
  a_rel_lat: float = 0.0
  orientation_angle: float = 0.0
  length: float = 0.0
  width: float = 0.0

  def __post_init__(self) -> None:
    _raw_id(self.raw_id)
    _finite(self.a_rel, self.a_rel_lat, self.orientation_angle, self.length, self.width)
    if not 0 <= self.object_class <= 7:
      raise ValueError(f"invalid ARS408 object class: {self.object_class}")
    if not _in_protocol_range(self.a_rel, -10.0, 10.47):
      raise ValueError(f"invalid ARS408 longitudinal acceleration: {self.a_rel}")
    # The protocol's sensor-frame lateral range [-2.5, 2.61] is reflected into
    # the openpilot frame, yielding [-2.61, 2.5].
    if not _in_protocol_range(self.a_rel_lat, -2.61, 2.5):
      raise ValueError(f"invalid ARS408 lateral acceleration: {self.a_rel_lat}")
    if not _in_protocol_range(self.orientation_angle, -180.0, 180.0):
      raise ValueError(f"invalid ARS408 orientation angle: {self.orientation_angle}")
    if not 0.0 <= self.length <= 51.0 or not 0.0 <= self.width <= 51.0:
      raise ValueError("invalid ARS408 object dimensions")


@dataclass(frozen=True, slots=True)
class AssembledObject:
  general: ObjectGeneral
  quality: ObjectQuality
  extended: ObjectExtended | None = None

  def __post_init__(self) -> None:
    ids = {self.general.raw_id, self.quality.raw_id}
    if self.extended is not None:
      ids.add(self.extended.raw_id)
    if len(ids) != 1:
      raise ValueError("ARS408 object parts have different raw IDs")

  @property
  def raw_id(self) -> int:
    return self.general.raw_id


@dataclass(frozen=True, slots=True)
class CycleResult:
  status: ObjectStatus
  objects: tuple[AssembledObject, ...]
  exact: bool
  invalid: bool
  general_count: int
  quality_count: int
  extended_count: int
  duplicate_ids: tuple[int, ...] = ()
  measurement_mono_time_ns: int = 0
  last_component_mono_time_ns: int = 0
  assembly_close_mono_time_ns: int = 0
  # Timestamp integrity gates the shadow perception path only. It is separate
  # from protocol invalidity so malformed/missing host metadata cannot change
  # the established standard RadarData object path.
  timestamp_valid: bool = False


@dataclass(frozen=True, slots=True)
class RadarStateSnapshot:
  interference: bool
  voltage_error: bool
  temporary_error: bool
  temperature_error: bool
  persistent_error: bool
  sensor_id: int
  output_type: int
  quality_enabled: bool
  extended_enabled: bool
  motion_rx_state: int
  max_distance_m: int
  nvm_read_status: int
  nvm_write_status: int
  sort_index: int
  ctrl_relay_enabled: bool
  rcs_threshold: int

  def __post_init__(self) -> None:
    if not 0 <= self.sensor_id <= 7:
      raise ValueError(f"invalid ARS408 sensor ID: {self.sensor_id}")
    if not 0 <= self.output_type <= 3:
      raise ValueError(f"invalid ARS408 output type: {self.output_type}")
    if not 0 <= self.motion_rx_state <= 3:
      raise ValueError(f"invalid ARS408 motion RX state: {self.motion_rx_state}")
    if not 0 <= self.max_distance_m <= 2046:
      raise ValueError(f"invalid ARS408 maximum distance: {self.max_distance_m}")
    if not 0 <= self.nvm_read_status <= 1 or not 0 <= self.nvm_write_status <= 1:
      raise ValueError("invalid ARS408 NVM status")
    if not 0 <= self.sort_index <= 2 or not 0 <= self.rcs_threshold <= 7:
      raise ValueError("invalid ARS408 sort index or RCS threshold")


@dataclass(frozen=True, slots=True)
class FilterStateHeader:
  cluster_filter_count: int
  object_filter_count: int

  def __post_init__(self) -> None:
    if not 0 <= self.cluster_filter_count <= 255 or not 0 <= self.object_filter_count <= 255:
      raise ValueError("invalid ARS408 filter count")


@dataclass(frozen=True, slots=True)
class FilterStateRecord:
  index: int
  active: bool
  minimum: float
  maximum: float

  def __post_init__(self) -> None:
    _finite(self.minimum, self.maximum)
    if not 0 <= self.index <= 14:
      raise ValueError(f"invalid ARS408 object filter index: {self.index}")
    if self.minimum > self.maximum:
      raise ValueError("ARS408 filter minimum exceeds maximum")


@dataclass(frozen=True, slots=True)
class TrackedObject:
  raw_id: int
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  a_rel: float
  measured: bool
  object_class: int
  probability: int
  dynamic_property: int


@dataclass(frozen=True, slots=True)
class TrackerResult:
  tracks: tuple[TrackedObject, ...]
  accepted_raw_ids: tuple[int, ...]
  rejection_reasons: tuple[tuple[int, str], ...]
  handover_count: int
  duplicate_suppression_count: int


@dataclass(frozen=True, slots=True)
class DiagnosticErrors:
  can_error: bool = False
  radar_fault: bool = False
  radar_unavailable_temporary: bool = False
  wrong_config: bool = False


@dataclass(frozen=True, slots=True)
class DiagnosticsSnapshot:
  errors: DiagnosticErrors
  radar_state_ready: bool
  radar_state_count: int
  interference_count: int
  exact_cycles: int
  partial_cycles: int
  invalid_cycles: int
  raw_object_count: int
  class_counts: tuple[tuple[int, int], ...]
  probability_counts: tuple[tuple[int, int], ...]
  grace_held_tracks: int
  filter_header: FilterStateHeader | None
  filter_records: tuple[FilterStateRecord, ...]
  # Strict source-timestamp state for the shadow perception publisher. The
  # legacy fields above remain arrival-time based for standard RadarData.
  trusted_errors: DiagnosticErrors
  trusted_radar_state_ready: bool
  trusted_radar_state_count: int
  trusted_interference_count: int


ParsedFrame = ObjectStatus | ObjectGeneral | ObjectQuality | ObjectExtended | RadarStateSnapshot | FilterStateHeader | FilterStateRecord
