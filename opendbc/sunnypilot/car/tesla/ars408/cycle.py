from opendbc.sunnypilot.car.tesla.ars408.models import (
  AssembledObject, CycleResult, ObjectExtended, ObjectGeneral, ObjectQuality, ObjectStatus,
)


class ARS408CycleAssembler:
  def __init__(self) -> None:
    self._status: ObjectStatus | None = None
    self._status_time_ns = 0
    self._last_component_time_ns = 0
    self._timestamp_order_valid = False
    self._general: dict[int, ObjectGeneral] = {}
    self._quality: dict[int, ObjectQuality] = {}
    self._extended: dict[int, ObjectExtended] = {}
    self._duplicate_ids: set[int] = set()

  def push(self, frame: ObjectStatus | ObjectGeneral | ObjectQuality | ObjectExtended,
           timestamp_ns: int = 0) -> CycleResult | None:
    if isinstance(frame, ObjectStatus):
      result = self._close_cycle(timestamp_ns)
      self._status = frame
      self._status_time_ns = timestamp_ns
      self._last_component_time_ns = timestamp_ns
      self._timestamp_order_valid = timestamp_ns > 0
      self._general.clear()
      self._quality.clear()
      self._extended.clear()
      self._duplicate_ids.clear()
      return result

    if self._status is None:
      return None
    self._timestamp_order_valid = self._timestamp_order_valid and timestamp_ns > 0 and \
      timestamp_ns >= self._last_component_time_ns
    self._last_component_time_ns = timestamp_ns
    if isinstance(frame, ObjectGeneral):
      self._store(self._general, frame.raw_id, frame)
    elif isinstance(frame, ObjectQuality):
      self._store(self._quality, frame.raw_id, frame)
    else:
      self._store(self._extended, frame.raw_id, frame)
    return None

  def _store(self, collection: dict, raw_id: int, frame: object) -> None:
    if raw_id in collection:
      self._duplicate_ids.add(raw_id)
      return
    collection[raw_id] = frame

  def _close_cycle(self, assembly_close_time_ns: int) -> CycleResult | None:
    if self._status is None:
      return None

    paired_ids = (set(self._general) & set(self._quality)) - self._duplicate_ids
    objects = tuple(
      AssembledObject(self._general[raw_id], self._quality[raw_id], self._extended.get(raw_id))
      for raw_id in sorted(paired_ids)
    )
    expected = self._status.object_count
    timestamp_order_valid = self._timestamp_order_valid and assembly_close_time_ns > 0 and \
      assembly_close_time_ns >= self._last_component_time_ns
    exact = self._status.protocol_valid and not self._duplicate_ids and len(self._general) == expected and \
      len(self._quality) == expected and set(self._general) == set(self._quality)
    return CycleResult(
      status=self._status, objects=objects, exact=exact, invalid=not self._status.protocol_valid,
      general_count=len(self._general), quality_count=len(self._quality), extended_count=len(self._extended),
      duplicate_ids=tuple(sorted(self._duplicate_ids)),
      measurement_mono_time_ns=self._status_time_ns,
      last_component_mono_time_ns=self._last_component_time_ns,
      assembly_close_mono_time_ns=assembly_close_time_ns,
      timestamp_valid=timestamp_order_valid,
    )
