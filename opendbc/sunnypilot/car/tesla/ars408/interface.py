import time
from collections.abc import Callable

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.sunnypilot.car.tesla.ars408.constants import ARS408_MAX_DISTANCE_M
from opendbc.sunnypilot.car.tesla.ars408.cycle import ARS408CycleAssembler
from opendbc.sunnypilot.car.tesla.ars408.diagnostics import ARS408Diagnostics
from opendbc.sunnypilot.car.tesla.ars408.models import (
  DiagnosticErrors, FilterStateHeader, FilterStateRecord, ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus,
  RadarStateSnapshot, SoftwareVersion, TrackerResult,
)
from opendbc.sunnypilot.car.tesla.ars408.parser import ARS408Parser
from opendbc.sunnypilot.car.tesla.ars408.tracker import ARS408Tracker


class ARS408RadarInterface(RadarInterfaceBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
               clock: Callable[[], int] = time.monotonic_ns) -> None:
    super().__init__(CP, CP_SP)
    self._clock = clock
    self.parser = ARS408Parser(diagnostic_fields=False)
    self.rcp = self.parser.can_parser
    self.assembler = ARS408CycleAssembler()
    self.tracker = ARS408Tracker()
    self.diagnostics = ARS408Diagnostics(self._clock())
    self.radar_off_can = bool(CP.radarUnavailable)

  def update(self, can_packets: list[tuple[int, list[CanData]]]) -> structs.RadarDataT | None:
    if self.radar_off_can:
      return super().update(None)
    self.frame += 1

    now_ns = self._clock()
    latest_result: TrackerResult | None = None
    for timestamp, frames in can_packets:
      for frame in frames:
        parsed = self.parser.parse(timestamp, frame)
        if parsed is None:
          continue
        if isinstance(parsed, RadarStateSnapshot):
          self.diagnostics.observe_radar_state(parsed, now_ns)
          continue
        if isinstance(parsed, SoftwareVersion):
          self.diagnostics.observe_software_version(parsed)
          continue
        if isinstance(parsed, FilterStateHeader):
          self.diagnostics.observe_filter_header(parsed)
          continue
        if isinstance(parsed, FilterStateRecord):
          self.diagnostics.observe_filter_record(parsed)
          continue
        if isinstance(parsed, ObjectStatus):
          self.diagnostics.observe_status(parsed, now_ns)
        if not isinstance(parsed, (ObjectStatus, ObjectGeneral, ObjectQuality, ObjectExtended)):
          continue
        cycle = self.assembler.push(parsed)
        if cycle is None:
          continue
        max_distance_m = self._max_distance_m()
        latest_result = self.tracker.update(cycle, max_distance_m=max_distance_m)
        self.diagnostics.observe_cycle(cycle, latest_result)

    snapshot = self.diagnostics.snapshot(now_ns, parser_valid=self.parser.can_valid)
    if latest_result is not None:
      ret = self._build_radar_data(latest_result if snapshot.radar_state_ready else None)
      self._apply_errors(ret, snapshot.errors)
      return ret
    if self.frame % 5 == 0 and any((
      snapshot.errors.can_error, snapshot.errors.radar_fault,
      snapshot.errors.radar_unavailable_temporary, snapshot.errors.wrong_config,
    )):
      ret = structs.RadarData()
      self._apply_errors(ret, snapshot.errors)
      return ret
    return None

  def _max_distance_m(self) -> float:
    state = self.diagnostics.radar_state
    if state is not None and 0 < state.max_distance_m <= ARS408_MAX_DISTANCE_M:
      return float(state.max_distance_m)
    return ARS408_MAX_DISTANCE_M

  @staticmethod
  def _build_radar_data(result: TrackerResult | None) -> structs.RadarData:
    ret = structs.RadarData()
    if result is None:
      return ret
    points: list[structs.RadarData.RadarPoint] = []
    for track in result.tracks:
      point = structs.RadarData.RadarPoint()
      point.trackId = track.track_id
      point.dRel = track.d_rel
      point.yRel = track.y_rel
      point.vRel = track.v_rel
      point.deprecated.yvRel = track.yv_rel
      point.deprecated.aRel = track.a_rel
      point.deprecated.measured = track.measured
      points.append(point)
    ret.points = points
    return ret

  @staticmethod
  def _apply_errors(ret: structs.RadarData, errors: DiagnosticErrors) -> None:
    ret.errors.canError = bool(errors.can_error)
    ret.errors.radarFault = bool(errors.radar_fault)
    ret.errors.radarUnavailableTemporary = bool(errors.radar_unavailable_temporary)
    ret.errors.wrongConfig = bool(errors.wrong_config)
