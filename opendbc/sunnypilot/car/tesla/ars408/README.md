# ARS408 shadow perception safety boundary

This module is **shadow/offline only**. Its perception output is diagnostic evidence for replay, logging, and evaluation. It is not authorized to command steering, change a path, release a lane-change gate, or otherwise affect vehicle control.

The compact high-rate output is capped at 64 risk-prioritized targets. Parsing
and cycle-integrity checks still cover every received object. When more than 64
eligible targets exist, `targetsTruncated` and the full count remain explicit;
truncation is never evidence of clearance.

## What the output does not prove

- A forward target's presence is not a lane-occupancy decision. Its absence is never evidence that a lane or avoidance corridor is clear.
- One forward-facing radar cannot establish full side or rear coverage. Occlusion, mounting yaw, lateral/longitudinal bias, radar-to-vehicle alignment, and curved-road geometry remain installation-specific and unverified.
- Tesla blind-spot monitoring, road-edge, lane-line, and lane-model hard blocks remain independent requirements. ARS408 output must never override or relax them.
- `quality_score` and `clutter_score` are uncalibrated diagnostics. They are not probabilities, safety thresholds, free-space confidence, or control release criteria.

## Unverified protocol and installation assumptions

- `LATERAL_KINEMATICS_VERIFIED` is currently `False`. Lateral position/velocity, lateral acceleration, and orientation may be retained for diagnosis, but must not be used for association, cut-in inference, avoidance direction, or control until device/replay evidence verifies the complete sensor-to-openpilot transform.
- Cadence is valid only when the expected counter step comes from a device capture and is explicitly marked `deviceCapture`. An external value or inferred step never becomes verified.
- The inference heuristic of five consistent deltas in a 50-110 ms window is an uncalibrated diagnostic assumption, not an ARS408 protocol guarantee.
- Shadow parser health does not use `CANParser.can_valid`, because the ARS408 DBC messages have ignored/NaN liveness frequencies. It requires successfully decoded required frames in the current epoch, no required-frame semantic/DLC rejection, a general required-frame source age of at most 500 ms, and an independently maintained ObjectStatus source age of at most 500 ms. RadarState or object component traffic cannot refresh the Status TTL. These parser TTLs are independent of RadarState freshness and object measurement freshness.
- Every decoded ObjectStatus is checked immediately for protocol version/object-count validity. Counter continuity is checked at Status arrival only when a device-capture cadence step is verified; an unverified cadence must not claim that a counter gap was detected or absent.

## Standard RadarData compatibility

The standard `RadarData` path remains observable independently of the shadow gate. Parser changes to the `yvRel` sign and reserved measurement-state handling can therefore change standard diagnostic output even while shadow perception remains invalid or hidden. That observability is useful for replay comparison; it is not proof that the lateral sign, timing, or reserved-state semantics are correct on the installed radar.

## Deployment blocker

The current candidate removes direct shadow tracking, serialization, and publication from both `RadarInterface.update()` and the `card` process. A separate observational `ars408shadowd` subscribes to non-conflated raw `can` and owns its parser, cycle assembler, standard acceptance/dedup tracker, perception tracker, and both shadow publishers. There is no synchronous shadow IPC, serialization, wait, or publisher call in the `card`/standard `RadarData` path.

This process boundary is not real-time or resource isolation. `ars408shadowd` still shares C3 CPU, memory, thermal capacity, raw-CAN msgq queues, and logger bandwidth with safety-relevant processes. Before Cap'n Proto decoding it rejects empty raw events, events over 256 KiB, and batches over 2 MiB; after processing it applies a 25 ms wall-time check. These are fail-closed input caps/detectors, not preemption, scheduler reservations, device measurements, or proof that contention stayed within a deadline. Raw queue overflow/drop behavior, restart storms, two independent non-conflated CAN subscribers, and worst-case C3XL CPU/RSS/temperature/card-loop/sendcan effects remain unverified.

The production worker runs its standard acceptance/dedup tracker for every completed cycle, including when cadence is unverified and `LATERAL_KINEMATICS_VERIFIED` is `False`; duplicate suppression is already worst-case `O(n^2)`. An unverified or inferred cadence still prevents perception target processing/promotion. Once an explicitly verified `deviceCapture` cadence is configured, perception association, handover, and structural-clutter work add further worst-case `O(n^2)` cost. Neither the cadence gate nor the false lateral-transform gate provides a resource boundary.

On a detected reset or live configuration loss, the daemon makes a best-effort attempt to publish a new-epoch empty `INVALID` barrier and discard pre-barrier queued CAN. A gate pause retains the same process-lifetime worker, its physical 14 Hz/4 Hz limit clocks, pending failure evidence, and uncommitted diagnostics transaction/window. Absent an independent barrier build/send/publication failure, the gate edge itself faults once; sustained invalid polling alone retries the same pending barrier and does not add gate-induced epoch churn. Independent publication failures remain producer faults and may create additional epochs. Recovery must complete the pending barrier and drain before accepting a baseline. These guarantees are limited to one daemon process lifetime. The manager keeps the process alive across normal offroad/onroad transitions, but that does not make the producer or its data valid: the daemon's nonblocking live Tesla/ARS408 flag+safety gate controls all processing and publication. A non-Tesla or never-enabled boot need not create a CAN socket. After the first valid configuration, the process keeps one process-lifetime, non-conflated CAN subscriber across any later gate toggles; rotating or dropping that subscriber is forbidden because msgq reader slots are finite and are not reclaimed by ordinary subscriber destruction, so repeated rotation could disturb other CAN readers. Automatic crash restart is disabled for the same reason. An unrecoverable crash is silent, and a hung process may still be reported as running; only a future consumer's independent service-liveness check and local TTL may interpret missing/stale output as `UNKNOWN`. There is currently no consumer. `selfdrived` ignores this observational process only for engagement faults—it does not turn a dead service into valid radar data.

In live mode the worker, publisher rate limits, processing budget, pipeline live freshness checks, and core-send completion fence all require Linux `CLOCK_BOOTTIME`, matching CAN `Event.logMonoTime`; only Windows development tests may fall back to `monotonic_ns`. Future source timestamps and transport age over 250 ms fail closed, while live shadow-now is the actual host `CLOCK_BOOTTIME` so queue lag cannot be projected away. These thresholds are engineering safety boundaries, not device measurements. The fence combines successful core-send completion time/source high-water mark with a best-effort drain, but the real cross-socket ordering property remains a C3 msgq/HIL blocker. Production `REPLAY`/source mode is disabled until a non-disruptive transport watermark is proven; fixed-count batch dropping is not accepted as a fence.

Process crash, SIGTERM, manager stop, socket failure, or publication failure can prevent the barrier. Any future consumer must therefore treat service loss or local TTL expiry as `UNKNOWN`, require monotonic Event/source time, and discard all cached state on an epoch change. A last healthy message or manager state must never keep presence data valid after producer loss.

No C3XL worst-case replay, real-msgq dual-subscriber test, kill/restart campaign, HIL, or fault-injection run has validated these resource and failure boundaries. Status is therefore **REVIEW_BLOCKED / NO DEPLOYMENT AUTHORIZATION**. This module must not be enabled for production control until worst-case target-device evidence closes those blockers.

## Logging contract

- `ars408StateSP` is produced at up to 14 Hz with `should_log=False`; it is a bounded live health/compact channel and is not written to rlog or qlog by the service logger.
- `ars408DiagnosticsSP` is nominally emitted at 2 Hz. One immediate health/fault transition may replace the next nominal window, giving a declared ceiling of 4 Hz. It is logged with `should_log=True` and decimation 1, so every physical diagnostics event is selected for logging.
- Raw CAN completeness can only be audited from rlog containing the CAN stream. A qlog sequence of derived `ars408DiagnosticsSP` events cannot prove that every raw CAN frame, component, counter transition, or rejected object was present.

## Diagnostics window and field semantics

`targets`, `fullCount`, `targetCount`, and the four `*FullCount` fields form a bounded incremental audit collection accumulated since the previous physical `ars408DiagnosticsSP` event. They are not a current-scene inventory. Selection is deduplicated by `(observedProducerEpoch, logicalId)`: possible-VRU/class-conflict records have highest safety priority, then expired, clutter, uncertain, and stableAudit; for the same `(observedProducerEpoch, logicalId)` and priority, the newer observation replaces the older one. `rawCount` and `acceptedCount` describe only the latest snapshot header; they are not window totals.

A `targetCount` of zero is not a current target-absence claim and must never authorize free space, lane use, or motion. It can simply mean that no new audit target entered the bounded collection during that diagnostics window.

Field units are:

- `dRel`, `yRel`, `rawDRel`, `rawYRel`, `length`, and `width`: metres (m).
- `vRel`, `yvRel`, `rawVRel`, and `rawYvRel`: metres per second (m/s).
- `aRelLong` and `aRelLat`: metres per second squared (m/s²).
- `orientation`: degrees.
- `rcs`: dBm² (the DBC spelling is `dBm2`).
- `lastMeasuredAgeMs`: milliseconds; all `*MonoTime` fields: nanoseconds in their documented monotonic source domain.

All lateral fields `yRel`, `rawYRel`, `yvRel`, `rawYvRel`, `aRelLat`, and `orientation` have passed through the current sensor-to-openpilot sign transform, which remains unverified on the installed vehicle. `observedProducerEpoch`, `observedSequence`, and `observedMonoTime` identify the selected source observation, not the diagnostics publication header; they must not be replaced with the later event header values. The bounded audit collection is deduplicated by `(observedProducerEpoch, logicalId)`, so an unsent target from an older producer instance remains attributable across a later epoch until a physical diagnostics send commits the window.
