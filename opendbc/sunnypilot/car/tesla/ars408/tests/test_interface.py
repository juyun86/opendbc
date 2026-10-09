from collections import deque
from dataclasses import replace

import pytest

from opendbc.car import structs
from opendbc.sunnypilot.car.tesla.ars408.interface import ARS408RadarInterface
from opendbc.sunnypilot.car.tesla.ars408.models import (
  ObjectGeneral, ObjectQuality, ObjectStatus, ParsedFrame, RadarStateSnapshot, SoftwareVersion,
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


def test_internal_metadata_does_not_require_schema_fields() -> None:
  frames: list[ParsedFrame] = [
    radar_state(), ObjectStatus(1, 1, 1), ObjectGeneral(9, 20.0, 0.0, 0.0, 0.0, 1.0, 0),
    ObjectQuality(9, 6, 2), ObjectStatus(0, 2, 1),
  ]
  radar = interface(frames)
  result = radar.update(packets(len(frames)))
  assert result is not None and result.points[0].trackId == 9
  track = radar.tracker._tracks[9]
  assert track.obj.quality.probability == 6
  assert not hasattr(result.points[0], "objectClass")


def test_radar_state_faults_map_to_standard_errors() -> None:
  faulty = replace(radar_state(), persistent_error=True)
  frames: list[ParsedFrame] = [faulty, ObjectStatus(0, 1, 1), ObjectStatus(0, 2, 1)]
  radar = interface(frames)
  result = radar.update(packets(len(frames)))
  assert result is not None and result.errors.radarFault


@pytest.mark.parametrize("invalid_status", [ObjectStatus(0, 1, 2), ObjectStatus(101, 1, 1)])
def test_invalid_object_protocol_publishes_error_and_recovers(invalid_status: ObjectStatus) -> None:
  second_invalid = replace(invalid_status, measurement_counter=2)
  radar = interface([radar_state(), invalid_status, second_invalid, ObjectStatus(0, 3, 1), ObjectStatus(0, 4, 1)])
  result = radar.update(packets(4))
  assert result is not None and result.errors.wrongConfig and not result.points
  recovered = radar.update(packets(1))
  assert recovered is not None and not recovered.errors.wrongConfig


def test_version_is_diagnostic_only_and_does_not_close_object_cycle() -> None:
  version = SoftwareVersion(4, 34, 1, False, 0)
  frames = [radar_state(), ObjectStatus(0, 1, 1), version, ObjectStatus(0, 2, 1)]
  radar = interface(frames)
  result = radar.update(packets(len(frames)))
  assert result is not None and not result.errors.wrongConfig
  snapshot = radar.diagnostics.snapshot(0, parser_valid=True)
  assert snapshot.software_version == version and snapshot.exact_cycles == 1


def test_batched_cycles_use_can_timestamps_for_high_speed_id_handover() -> None:
  frames: list[ParsedFrame] = [
    radar_state(), ObjectStatus(1, 1, 1), ObjectGeneral(10, 40.0, 0.0, -31.25, 0.0, 8.0, 1),
    ObjectQuality(10, 4, 2), ObjectStatus(1, 2, 1), ObjectGeneral(20, 37.6, 0.0, -31.25, 0.0, 8.0, 1),
    ObjectQuality(20, 4, 1), ObjectStatus(0, 3, 1),
  ]
  radar = interface(frames)
  # Both cycles arrive in one update; its fixed clock cannot provide their dt.
  first = (1_000_000_000, [(0x60A, b"", 1)] * 4)
  second = (1_076_800_000, [(0x60A, b"", 1)] * 3)
  third = (1_153_600_000, [(0x60A, b"", 1)])
  result = radar.update([first, second, third])
  assert result is not None and len(result.points) == 1
  assert result.points[0].trackId == 10 and result.points[0].dRel == pytest.approx(37.6)
  assert result.points[0].deprecated.measured
