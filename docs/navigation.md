# Navigation control API

`pcvmf_mavlink.navigation` supplies schema-version-1 control, setpoint, stream
status, and navigation telemetry payloads. Register their codecs as shown in
`examples/fake-navigation.yaml`. Existing telemetry, actions, missions, and
parameters keep their original schemas.

## Enable and route

`setpoints_enabled: true` enables leased streams; `mission_execution_enabled:
true` enables mission execution. These flags are independent of
`commands_enabled` (arm/takeoff/land/RTL/disarm) and `missions_enabled` (transfer).
All are false by default. A stored mission can be controlled without enabling
transfer. Arming and takeoff are separate explicit operations.

| Payload and codec prefix | Default topic | Delivery |
| --- | --- | --- |
| `ControlRequest` | `flight/command` | ordered |
| `ControlResult` | `flight/result` | ordered |
| `Setpoint` | `flight/setpoint` | latest |
| `StreamStatus` | `flight/stream_status` | latest |
| `NavigationTelemetry` | `flight/navigation` | latest |

Append `Codec` to the payload names for codec import paths. Topic names are
configurable through the worker options. Declare navigation publications when
either navigation feature is enabled and stream status when streams are enabled.
The custom worker coalesces valid stream updates itself; PCVMF PUB/SUB can still
drop packets. Only the worker thread accesses injected PCVMF transports.

## Start, update and stop a stream

`Setpoint(session_id, stream_id, sequence, expires_at, yaw_rad,
position_ned_m=None, velocity_ned_m_s=None)` contains exactly one vector. Both
vectors have three elements in north, east, down order. Position is metres from
the autopilot's local estimator origin, **not home or current position**; velocity
is metres per second. Down is positive. Yaw is radians in [-π, π], with zero
toward north. These map to `MAV_FRAME_LOCAL_NED`; no body-frame conversion occurs.

`sequence` must increase within one stream (0–2^53−1). `expires_at` is an absolute
host wall-clock deadline. The effective lifetime also has a monotonic bound of
`setpoint_timeout_s` from receipt, so a long deadline cannot create an indefinite
setpoint. Duplicate, reordered, expired, wrong-owner, wrong-stream, old-session,
or out-of-limit updates are ignored and do not renew the lease. Speed and position
limits apply to the vector norm. They constrain targets; they are not a geofence,
acceleration limit, collision avoidance, or an achieved-speed guarantee.

For an already armed vehicle with a fresh local estimate:

```python
import time
import uuid
from pcvmf_mavlink.navigation import ControlRequest, Setpoint

stream_id = str(uuid.uuid4())
sequence = 0
initial = Setpoint(
    status.session_id, stream_id, 0, time.time() + 0.5, 0.0,
    velocity_ned_m_s=[0.0, 0.0, 0.0],
)
start = ControlRequest(
    str(uuid.uuid4()), status.session_id, "stream_start", time.time() + 5,
    stream_id=stream_id, initial_setpoint=initial,
)
context.publisher.publish("flight/command", start)

# In subsequent bounded worker steps, publish fresh updates at e.g. 20 Hz.
# Start immediately, including while the start result is still processing.
sequence += 1
context.publisher.publish("flight/setpoint", Setpoint(
    status.session_id, stream_id, sequence, time.time() + 0.5, 0.0,
    velocity_ned_m_s=[0.5, 0.0, 0.0],
))
```

Position and velocity targets can be switched by publishing the other vector type. The
producer owns the stream by its PCVMF source name. Another source cannot update
or stop that stream. Use a new stream ID for every start; IDs are not reusable
within a connection session. The ID history is bounded by `dedup_size`, after
which starts return busy until a new session.

PX4 requires a continuous stream before switching to OFFBOARD; this package
primes for 1.1 seconds. Continue fresh updates during priming. An initial point
alone will expire with the default 0.5-second timeout. The action timeout must
exceed 1.2 seconds; the default is 5. ArduCopter switches to GUIDED. The adapter
waits for the expected heartbeat mode before accepting activation. It checks
fresh arming and finite local-position telemetry, while the autopilot enforces
its own mode prerequisites. Receiving a local position is not proof of estimator
quality. ArduCopter normally needs an explicit takeoff before guided movement.
See [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard) and
[ArduCopter Guided commands](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html).

Stop with a new `ControlRequest` using `operation="stream_stop"` and the same
`stream_id`, without an initial setpoint. Transmission stops first, then the
adapter requests PX4 Hold or ArduCopter Loiter. Lease expiry performs the same
transition within `stream_stop_timeout_s`; rejection or timeout invalidates the
connection. On an observed mode departure, the package stops transmitting and
resets the session without sending a mode override. Loss of arming/local telemetry
also invalidates the session. These are best-effort scheduling/transport bounds,
not hard real-time guarantees; an already transmitted packet cannot be recalled.

Land, RTL, and disarm from an authorized action source supersede an active stream
without an intermediate Hold command. Other transactions return busy until the
stream stops. An in-progress activation/control transaction still occupies the
single transaction slot. Stream updates use a separate bounded latest-value
mailbox and continue during activation.

The pymavlink backend sends `SET_POSITION_TARGET_LOCAL_NED` directly. The MAVSDK
backend uses `MavlinkDirect` for one-shot setpoints and acknowledged mode changes;
its native Offboard plugin's indefinite resend cache is not used. Transmission
is scheduled by this package at `setpoint_hz` (5–50, default 20). Process exit,
disconnect, and cleanup cease emission; cleanup does not issue a flight action.
Onboard loss-of-control policy remains the autopilot's responsibility. A retained
position target may still be pursued if a Hold request cannot reach the vehicle.
The package does not change onboard failsafe parameters or resume a stream after
reconnect. External MAVSDK servers must be dedicated to the configured vehicle;
other clients' streams are outside this adapter's ownership.

## Mission execution

`ControlRequest` supports `mission_start`, `mission_pause`, and
`mission_set_current`. Only `mission_set_current` accepts `mission_index`, a
zero-based index in the firmware's stored mission; account for ArduCopter's home
entry. Mission operations do not accept `stream_id` or `initial_setpoint`.

Start requires fresh telemetry confirming arming, then asks the autopilot to
start/resume its stored mission. PX4 uses a mission mode change that does not arm
the vehicle; ArduCopter uses `MAV_CMD_MISSION_START`. Firmware settings determine
restart/resume behavior. Pause asks the autopilot to suspend execution (PX4 Hold
on both backends, or ArduCopter `MAV_CMD_DO_PAUSE_CONTINUE`). Select-current uses
acknowledged `MAV_CMD_DO_SET_MISSION_CURRENT` for ArduCopter. PX4 uses one
`MISSION_SET_CURRENT` message and waits for a subsequent `MISSION_CURRENT`
report with the requested index, because the tested firmware lacks command 224.
It does not depend on MAVSDK having cached an upload. Selection can redirect a
running mission; telemetry confirmation has no application request ID and is
subject to the same concurrent-client attribution limits as other MAVLink state.
Unsupported firmware commands return unsupported without a speculative fallback.
The [MAVLink command definitions](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MISSION_CURRENT)
describe index/reset semantics.
An upload ACK can precede the navigator's mission-state update. Observe mission
telemetry and firmware readiness before requesting execution; transfer acceptance
alone does not establish that the mission is feasible.

`ControlResult` contains request ID, session ID, requester, operation, outcome,
and detail. Processing means locally queued; acceptance means the mode/control
transaction succeeded, not that navigation or a mission completed. Existing
deduplication, request expiry, request conflicts, and unknown-outcome session
reset rules apply. The shared request-ID namespace also includes flight actions,
mission transfers, and parameters. Repeat the exact control request to recover a
cached result; never mutate its initial setpoint or expiry during retransmission.

## Observe state

`StreamStatus` includes session, state, stream ID, requester, last consumed
sequence, and reason. States are idle, priming, active, stopping, stopped, expired,
and lost. Status reports lifecycle and local consumption, not individual packet
ACKs or physical setpoint completion. It is published on transitions and refreshed
periodically. A consumer must match the current session and stream.

`NavigationTelemetry` uses the same `TelemetrySample(received_at, age_s, stale,
values)` structure as vehicle telemetry, with separate groups:

| Group | Values |
| --- | --- |
| `local` | `north_m`, `east_m`, `down_m`, `north_m_s`, `east_m_s`, `down_m_s` |
| `mission` | `current_index`, `total_items`, numeric MAVLink `mission_state` |
| `mission_reached` | `reached_index` |

Local data comes from `LOCAL_POSITION_NED`; mission groups come from
`MISSION_CURRENT` and `MISSION_ITEM_REACHED`. Missing/unsupported values are None
and unreceived groups are absent. A reported total of zero means no mission;
an unavailable total is None. Firmware may exclude the home entry from the
reported total. Receipt freshness is local host freshness, not synchronized
measurement time. Samples clear on disconnect. The reached group is an event's
last observed index and becomes stale without another event; stale does not undo
the fact it was observed. Use mode, position, landed state, and mission progress
together to establish the completion relevant to your application.
