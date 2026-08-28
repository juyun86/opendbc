from collections import deque
from dataclasses import replace

import pytest

from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.ars408.interface import ARS408RadarInterface
from opendbc.sunnypilot.car.tesla.ars408.models import (
  ObjectGeneral, ObjectQuality, ObjectStatus, ParsedFrame, RadarStateSnapshot,
)


def radar_state() -> RadarStateSnapshot:
  return RadarStateSnapshot(False, False, False, False, False, 0, 1, True, True, 0, 250, 0, 0, 1, False, 0)


class ScriptedParser:
  def __init__(self, frames: list[ParsedFrame]) -> None:
    self.frames = deque(frames)
    self.can_valid = True

  def parse(self, _timestamp: int, _frame: tuple[int, bytes, int]) -> ParsedFrame | None:
    return self.frames.popleft()


def interface(frames: list[ParsedFrame], *, now_ns: int = 0) -> ARS408RadarInterface:
  cp = structs.CarParams()
  cp.radarUnavailable = False
  radar = ARS408RadarInterface(cp, structs.CarParamsSP(), clock=lambda: now_ns)
  radar.parser = ScriptedParser(frames)  # type: ignore[assignment]
  return radar


def packets(count: int) -> list[tuple[int, list[tuple[int, bytes, int]]]]:
  return [(0, [(0x60A, b"", 1) for _ in range(count)])]


def test_waits_for_radar_state_before_publishing_standard_points() -> None:
  object_frames: list[ParsedFrame] = [
    ObjectStatus(1, 1, 1), ObjectGeneral(7, 40.0, -1.5, -2.0, 0.2, 4.0, 0),
    ObjectQuality(7, 4, 2), ObjectStatus(0, 2, 1),
  ]
  no_state = interface(object_frames)
  result = no_state.update(packets(len(object_frames)))
  assert result is not None and list(result.points) == []

  with_state = interface([radar_state(), *object_frames])
  result = with_state.update(packets(len(object_frames) + 1))
  assert result is not None and len(result.points) == 1
  point = result.points[0]
  assert point.trackId == 7
  assert (point.dRel, point.yRel, point.vRel, point.deprecated.yvRel, point.deprecated.aRel) == pytest.approx(
    (40.0, -1.5, -2.0, 0.2, 0.0),
  )
  assert point.deprecated.measured
  assert point.objectClass == 7
  assert point.existenceProbability == 4
  assert point.dynamicProperty == 0


def test_internal_metadata_is_published_as_optional_radar_point_fields() -> None:
  frames: list[ParsedFrame] = [
    radar_state(), ObjectStatus(1, 1, 1), ObjectGeneral(9, 20.0, 0.0, 0.0, 0.0, 1.0, 0),
    ObjectQuality(9, 6, 2), ObjectStatus(0, 2, 1),
  ]
  radar = interface(frames)
  result = radar.update(packets(len(frames)))
  assert result is not None and result.points[0].trackId == 9
  track = radar.tracker._tracks[9]
  assert track.obj.quality.probability == 6
  assert result.points[0].objectClass == 7
  assert result.points[0].existenceProbability == 6
  assert result.points[0].dynamicProperty == 0


def test_radar_state_faults_map_to_standard_errors() -> None:
  faulty = replace(radar_state(), persistent_error=True)
  frames: list[ParsedFrame] = [faulty, ObjectStatus(0, 1, 1), ObjectStatus(0, 2, 1)]
  radar = interface(frames)
  result = radar.update(packets(len(frames)))
  assert result is not None and result.errors.radarFault
