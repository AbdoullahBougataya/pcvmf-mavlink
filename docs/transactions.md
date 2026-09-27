# Mission and parameter API

Import the frozen payloads and schema-version-1 codecs from
`pcvmf_mavlink.transactions`. Existing action and telemetry schemas are unchanged.
Register the matching codecs in every application's top-level `codecs` list.
Use `flight/command` for requests and `flight/result` for results (or the existing
configured topic overrides). A consumer must inspect the payload type and match
`requester`, `session_id`, and `request_id` before acting on a result.

## Mission transfer

Enable `missions_enabled: true`. `MissionRequest` fields are `request_id`,
`session_id`, `operation`, `expires_at`, and `items` (defaults to an empty list).
Operations are `upload`, `download`, and `clear`. Only upload takes items; to
delete a mission, use an explicit clear request. Only ordinary missions
(`MAV_MISSION_TYPE_MISSION=0`) are transferred. Upload replaces that mission;
clear removes it. Neither operation issues arm, takeoff, start-mission, or mode
commands. Changes to a mission already executing are subject to firmware policy.

For example, inside a producer with an injected publisher and connected status:

```python
import time
import uuid
from pcvmf_mavlink.transactions import MissionItem, MissionRequest

items = [
    MissionItem(
        frame=6,                 # GLOBAL_RELATIVE_ALT_INT
        command=16,              # NAV_WAYPOINT
        x=473977420, y=85455940,  # degrees * 10^7
        z=10.0,                 # metres above home in this frame
        current=1,
    ),
]
request = MissionRequest(
    str(uuid.uuid4()), status.session_id, "upload", time.time() + 30, items
)
context.publisher.publish("flight/command", request)
```

`MissionItem` mirrors `MISSION_ITEM_INT`: `frame`, `command`, `x`, `y`, `z`,
`param1`–`param4`, `current`, and `autocontinue`. List order supplies zero-based
sequence numbers. The first three parameters default to zero; `param4` defaults
to `None`, and `autocontinue` defaults to 1. Exactly one uploaded item must have
`current=1`; this identifies the current item without starting the mission.
Raw command validity remains the autopilot's responsibility.

For global navigation use frame 5 (AMSL), 6 (above home), or 11 (above terrain),
with integer latitude/longitude in degrees × 10^7 and altitude in metres. Terrain
support depends on the firmware. Frame 2 is for commands whose x/y/z fields are
non-positional parameters. Other raw uint8 frame values are preserved, including
firmware-specific legacy values returned on download. Do not infer units for an
arbitrary frame/command. Optional float parameters and z use `None` to represent
MAVLink's NaN/unspecified value; JSON NaN and infinity are rejected. Values must
fit float32 or the MAVLink integer field widths.

The package preserves firmware item ordering. ArduCopter includes a home entry
at index zero; account for it when constructing or comparing its raw mission.
This API does not convert QGroundControl or Mission Planner files or insert a
takeoff, landing, or home item on your behalf. The protocol and frame conventions
are documented in the [MAVLink mission specification](https://mavlink.io/en/services/mission.html).

`MissionResult` contains request/session/requester IDs, operation, outcome,
detail, and `items`. An accepted download returns the complete ordered list,
including an empty list when the autopilot reports zero items. ArduCopter may
still return its home entry after clearing. Other results have `items=None`.
Upload/clear acceptance is a protocol acknowledgement, not evidence of mission
execution. No partial download is published as a complete result.

`max_mission_items` bounds uploads and returned downloads (default 500, maximum
10000). Pymavlink checks the incoming count before requesting items. MAVSDK's
generated download RPC returns the full list before the adapter applies this
limit; its native gRPC receive limit still applies. The pymavlink adapter retries
mission protocol exchanges up to five times, with 1.5-second initial/clear
timeouts and 0.25-second item timeouts. MAVSDK manages native retries. The whole
transaction is bounded by `transfer_timeout_s` and the request expiry.

## Named parameters

Set `parameters_enabled: true` to read parameters. Add
`parameter_writes_enabled: true` to write; configuration rejects the write flag
without the read flag. `ParameterRequest` fields are `request_id`, `session_id`,
`operation` (`get` or `set`), `expires_at`, `name`, `parameter_type` (`int` or
`float`), and `value` (required for set, `None` for get).

```python
from pcvmf_mavlink.transactions import ParameterRequest

request = ParameterRequest(
    str(uuid.uuid4()), status.session_id, "get", time.time() + 5,
    name="MPC_TKO_SPEED", parameter_type="float",
)
context.publisher.publish("flight/command", request)
```

Names contain 1–16 printable ASCII characters without spaces. Integers are
signed 32-bit values; floats must be finite and representable as float32.
The returned `ParameterResult` includes the same identifiers and operation,
outcome, detail, name, parameter type, and value. Only an accepted result has a
value. Float writes are rounded to float32 for transmission and comparison.
MAVSDK parameter access is restricted to autopilot component 1: the pinned
server's component-selection RPC returns `UNKNOWN`. Pymavlink uses the configured
target component. No generic peripheral-component API is provided here.

The adapter reads the existing parameter and verifies its type before writing.
Pymavlink supports the smaller integer wire types used by ArduCopter while
checking their actual ranges. PX4 uses bytewise integer encoding; ArduCopter
uses numeric float encoding. Integer writes that cannot be represented exactly
in ArduCopter's encoding are rejected before transmission. See the
[MAVLink parameter specification](https://mavlink.io/en/services/parameter.html).

Pymavlink transmits each parameter write once, then requests readback; read
requests can retry five times. This also works when ArduCopter does not emit
an immediate write acknowledgement. MAVSDK handles its own retries. An accepted
write means the requested value was observed through the protocol. It does not
confirm persistence through reboot or that reboot-dependent behavior is active.
The API does not invent parameters, validate firmware-specific operational
ranges, fetch metadata, enumerate all parameters, or support string/64-bit types.

## Deadlines and recovery

All requests use the current connection session and an unchanged absolute
host-wall-clock expiry when retransmitted. `processing` indicates local queuing;
other outcomes follow the action contract. Expiry before transmission prevents
execution. A timeout after a transaction starts returns `outcome_unknown`, even
for reads, and resets the session to isolate late replies. For writes/transfers,
an unknown outcome can mean the change reached the autopilot. Do not interpret
it as rollback: reconnect and read the actual state before deciding what to do.

Mission/parameter transactions use the same bounded queue and deduplication as
actions. The request ID namespace spans all three payload types. Retransmitting
the identical request can recover a cached result; changing its type or contents
is a conflict. Mission lists are copied before submission. Cached download lists
consume memory until expiry/session eviction, so choose item limits, request
expiries, and deduplication size together. PUB/SUB delivery and exactly-once
execution are not guaranteed.

Cancellation sends a best-effort mission cancellation where available. Owned
MAVSDK servers are stopped on disconnect; an external server remains alive and
may have native retries in flight. MAVLink mission acknowledgements and parameter
updates have no PCVMF request ID, and parameter updates are broadcast. Use a
dedicated command connection; concurrent external clients can prevent reliable
attribution. Reconnect never replays an application request automatically.
