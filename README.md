# pcvmf-mavlink

An external PCVMF 0.3 worker package for PX4 multicopters and ArduCopter.
One worker owns one vehicle connection and one explicitly selected backend.
No framework modifications are required. Linux x86-64 and ARM64 and Python
3.10–3.12 are the intended platform matrix; see `docs/validation.md` for what
has actually been verified.

| Firmware | pymavlink | MAVSDK-Python |
| --- | --- | --- |
| PX4 multicopter | Implemented | Implemented, preferred |
| ArduCopter | Implemented, preferred | Rejected during validation |

## Installation and first run

From this package directory, `uv sync --extra all --extra dev` uses the local
PCVMF checkout through `[tool.uv.sources]`. The package's wheel instead declares
the normal dependency `PCVMF>=0.3.0,<0.4`. Install a PCVMF wheel alongside it when
that version is unavailable on your package index.

To add this package to the framework's existing development environment, run
from the framework root:

```bash
uv pip install --python .venv/bin/python -e 'packages/pcvmf-mavlink[all]'
uv run --no-sync pcvmf config validate packages/pcvmf-mavlink/examples/fake.yaml
uv run --no-sync pcvmf run --config packages/pcvmf-mavlink/examples/fake.yaml
```

Use `--no-sync` after the separate editable install. Select `[pymavlink]` or
`[mavsdk]` to install only one backend. The `fake` backend needs neither extra.
The finite fake example proves typed telemetry and action-result receipt and
observes the requested state changes before ending the application.

MAVSDK is pinned to the gRPC-based `mavsdk==3.10.2` distribution. Its matching
bundled `mavsdk_server` is started and explicitly stopped by the adapter.
This package does not use the newer direct Python binding. The adapter owns
the generated plugins' gRPC channel, including during partial initialization.

## Configuration

See `examples/px4-mavsdk.yaml`, `px4-pymavlink.yaml`, and
`arducopter-pymavlink.yaml`. These continuously publish telemetry and leave
flight actions disabled. Select `pcvmf_mavlink.worker:FlightControllerWorker`.
Register all four codecs shown in the examples in the application's `codecs`.
Declare `flight/telemetry`, `flight/status`, and `flight/result` publications.
An action producer declares `flight/command`; the flight worker subscribes to
that producer using `delivery: ordered`. Consumers use `latest` telemetry/status
and `ordered` results. Commands are read in order by the custom worker itself.

Required options are `backend` (`pymavlink`, `mavsdk`, or `fake`), `firmware`
(`px4` or `arducopter`), and `connection`. Examples of connection mappings:

```yaml
connection: {transport: udpin, host: 0.0.0.0, port: 14540}
connection: {transport: udpout, host: 127.0.0.1, port: 14550}
connection: {transport: tcp, host: 127.0.0.1, port: 5760}
connection: {transport: serial, device: /dev/ttyACM0, baud: 115200}
```

TCP is client-only. Use a dedicated MAVSDK link: that SDK selects a single
vehicle, and this adapter verifies the selected autopilot's heartbeat identity.
The flight controller must send MAVLink 2. The pymavlink adapter uses an explicit
v2 ardupilotmega dialect without changing process-global environment variables.
Unknown options and MAVSDK/ArduCopter are rejected before acquiring resources.
Structural validation does not establish vehicle reachability.

| Option | Default | Meaning |
| --- | --- | --- |
| `commands_enabled` | `false` | Explicitly enable incoming flight actions |
| `missions_enabled` | `false` | Enable mission upload, download and clearing |
| `parameters_enabled` | `false` | Enable named parameter reads |
| `parameter_writes_enabled` | `false` | Also enable parameter writes; requires `parameters_enabled` |
| `transfer_timeout_s`, `parameter_timeout_s` | `30`, `5` | Whole mission / parameter transaction deadlines |
| `max_mission_items` | `500` | Upload/download item limit (1–10000) |
| `setpoints_enabled` | `false` | Enable leased local NED position/velocity streams |
| `mission_execution_enabled` | `false` | Enable mission start, pause and current-item selection |
| `setpoint_hz`, `setpoint_timeout_s` | `20`, `0.5` | Send frequency and maximum silence before stream expiry |
| `stream_stop_timeout_s` | `2` | Budget for requesting Hold/Loiter on stream expiry |
| `max_setpoint_speed_m_s`, `max_setpoint_distance_m` | `5`, `100` | Vector speed and position norm limits (position measured from EKF origin) |
| `setpoint_topic`, `stream_status_topic`, `navigation_topic` | `flight/setpoint`, `flight/stream_status`, `flight/navigation` | Stream input, stream lifecycle, and local/mission telemetry |
| `target_system`, `target_component` | `1`, `1` | Expected autopilot IDs |
| `source_system`, `source_component` | `245`, `190` | This connection's MAVLink identity |
| `connect_timeout_s` | `15` | Initial connection and each reconnect attempt |
| `link_timeout_s` | `3` | Silence from the expected heartbeat |
| `action_timeout_s` | `5` | Entire action transaction, including prerequisites |
| `telemetry_stale_s`, `telemetry_hz` | `3`, `10` | Local sample freshness and publication rate |
| `reconnect_initial_s`, `reconnect_max_s` | `1`, `10` | Exponential reconnect delay |
| `cleanup_timeout_s` | `2` | Adapter cleanup budget; worker shutdown also allows thread join |
| `max_messages`, `receive_budget_ms` | `100`, `5` | Bounded worker input processing |
| `event_queue_size`, `dedup_size` | `256`, `1024` | Result/status queue and deduplication cache limits |
| `telemetry_topic`, `status_topic`, `command_topic`, `result_topic` | `flight/…` | Distinct declared routing topics |

Worker `rate_hz`, `startup_timeout`, `progress_timeout`, and `shutdown_timeout`
belong in PCVMF YAML, not options. Examples use 50 Hz and 30/10/5-second
timeouts. Increase the shutdown allowance when changing cleanup budgets or
enabling recording. Onboard failsafe policy remains a flight-controller setting;
the package does not change it or issue a flight action during cleanup.

MAVSDK's optional `server` mapping accepts `executable` and/or `port` for an
owned local server (otherwise bundled executable and an ephemeral port), or
`host` plus `port` for an externally managed server. Configure the external
server's MAVLink connection and source IDs yourself to match the options. It
must use the SDK's matching server version and serve only the expected vehicle.
The adapter closes its client channel but never terminates an external server.

## Message contract

Import frozen dataclasses and codecs from `pcvmf_mavlink.messages`. Every codec
uses schema version 1 and validates encoding as well as decoding. Prefixes are
`pcvmf_mavlink.vehicle_telemetry`, `.connection_status`, `.action_request`, and
`.action_result`; these are separate from configurable routing topics.

`VehicleTelemetry(session_id, system_id, component_id, samples)` contains
`TelemetrySample(received_at, age_s, stale, values)` groups: state, landed,
position, velocity, attitude, battery, GPS, and health. Sample timestamps are
host receipt wall time; ages use the local monotonic clock. They are not
synchronized vehicle measurement timestamps. Unreceived groups are absent and
unavailable values are `None`. Stale values stay marked stale until refreshed;
all samples are cleared on disconnect. No validity is inferred from zero values.
The MAVSDK server can cache state internally; receipt freshness does not prove
sensor measurement freshness. Health fields differ by backend; missing fields
are unknown, not a passed arming check.

Units appear in field names: degrees, metres AMSL or above home, NED metres per
second, Euler radians, volts, and a battery fraction between zero and one.
Custom flight modes retain a `CUSTOM_…` representation when no common label is
available. Firmware identity is published in `ConnectionStatus`, alongside the
session UUID, capabilities, enabled-command flag and connection reason.

`ActionRequest(request_id, session_id, action, expires_at, takeoff_altitude_m)`
supports arm, disarm, takeoff, land, and return_to_launch. `expires_at` is host
wall time and must remain unchanged when retransmitting the same request.
Only takeoff accepts altitude (0.1–1000 metres above home). Takeoff requires
fresh telemetry confirming explicit arming. ArduCopter first acknowledges and
enters Guided mode; PX4/pymavlink requires HOME_POSITION to convert altitude
to AMSL. Other preconditions and action acceptance are enforced by the autopilot.

`ActionResult` includes request/session IDs, `requester` (PCVMF source worker),
action, outcome and detail. `processing` means queued locally; `accepted` means
autopilot acceptance, not physical completion. Observe telemetry to determine
completion. Other outcomes include disabled, expired, busy, disconnected,
session_mismatch, request_conflict, unsupported, rejected, and outcome_unknown.
The finite `SmokeWorker` demonstrates request correlation and state observation.

PCVMF PUB/SUB can lose requests or results, including at startup. The sender may
retransmit an identical request to recover a result while its dedup entry remains
cached. Unexpired entries are retained; cache exhaustion rejects new work as busy.
Expired or old-session requests cannot run even after cache eviction.
Deduplication is bounded, keyed by source/session/request ID, and does not
survive process restart. It does not promise exactly-once execution. MAVLink ACKs
also carry no application request ID. Only one action transaction runs at a time.
The pymavlink adapter sends each action command once; MAVSDK may retry internally.
Do not run another command client with the same source IDs.

An ACK timeout or disconnect during a transaction produces outcome_unknown and
resets the link. It is not proof that the vehicle did nothing. Pending commands
are invalidated, old sessions rejected, telemetry discarded, and no actions are
replayed after reconnect. New commands require the new status session ID. Result
queue overflow and unexpected backend errors fail the worker rather than silently
discarding results. Initial connection failure fails application startup.

## Mission transfer and parameters

All three supported backend/firmware pairs implement mission upload, download,
and clearing, and named integer/float parameter reads and writes. These features
are opt-in through the options above, independent of `commands_enabled`.
Register the codecs for each feature you use, alongside the telemetry/status
codecs and any action codecs your application uses:

```yaml
codecs:
- pcvmf_mavlink.transactions:MissionRequestCodec
- pcvmf_mavlink.transactions:MissionResultCodec
- pcvmf_mavlink.transactions:ParameterRequestCodec
- pcvmf_mavlink.transactions:ParameterResultCodec
```

Publish these requests on the existing `flight/command` topic and receive their
typed results on `flight/result`, both with ordered delivery. All request kinds
share the same source/session/request-ID namespace, deduplication cache, and
single transaction slot. A mission transfer or parameter read can therefore
produce `busy` for a concurrent flight action. `ConnectionStatus.supported_actions`
and `commands_enabled` still describe flight actions; configure the new feature
flags explicitly rather than inferring them from that status.

The finite device-free example exercises upload/download/clear and typed
parameter read/write/readback across spawned workers:

```bash
uv run --no-sync pcvmf config validate examples/fake-transactions.yaml
uv run --no-sync pcvmf run --config examples/fake-transactions.yaml
```

`TransactionSmokeWorker` requires the fake backend. See
[mission and parameter API](docs/transactions.md) for payload examples, coordinate
frames, firmware differences, limits, and transfer outcome semantics.

## Streamed setpoints and mission execution

All three supported backend/firmware pairs implement local NED position/yaw and
velocity/yaw streams, plus mission start, pause, and current-item selection.
Enable `setpoints_enabled` and/or `mission_execution_enabled` explicitly. Starting
a stream or mission requires fresh armed telemetry; stream start also requires
finite, fresh local position. Arming and takeoff remain explicit actions.

Navigation uses separate schema-version-1 codecs from `pcvmf_mavlink.navigation`:
`ControlRequestCodec`, `ControlResultCodec`, `SetpointCodec`, `StreamStatusCodec`,
and `NavigationTelemetryCodec`. Existing schemas are unchanged. Control requests
and results use the existing command/result topics. Declare `flight/navigation`
when either feature is enabled, and `flight/stream_status` for streams. A setpoint
producer publishes `flight/setpoint`; subscribe with `delivery: latest`. The
worker keeps only the newest valid update for the current owner and stream.

Each stream starts with an initial setpoint and a new stream ID. Keep publishing
fresh, increasing-sequence updates during activation and execution. PX4 activation
includes more than one second of priming before changing to OFFBOARD. ArduCopter
uses GUIDED. On stale input, transmission stops and the adapter requests PX4 Hold
or ArduCopter Loiter. Reconnect and observed mode departures invalidate the stream;
old updates cannot restart it. Land, RTL and disarm can supersede an active stream.

```bash
uv run --no-sync pcvmf config validate examples/fake-navigation.yaml
uv run --no-sync pcvmf run --config examples/fake-navigation.yaml
```

This finite fake-only example observes state changes through streaming and
mission controls. See [navigation API](docs/navigation.md) for ownership, expiry,
coordinate frames, mission progress, backend details, and failure behavior.

## Verification and distribution

```bash
uv run --no-sync pytest
uv run --no-sync ruff check .
uv run --no-sync black --check .
uv build
```

Run these from the package directory in its configured development environment.
Tests cover direct codecs, worker policy, reconnect, real MAVLink UDP/TCP/serial
framing, MAVSDK's actual bundled server, and spawned PCVMF workers. Simulator
tests are opt-in; instructions and current evidence are in `docs/validation.md`.
Install the built PCVMF and plugin wheels in a clean environment, change outside
the source checkout, validate the supplied configuration and run the finite fake
scenario. Editable installs and `PYTHONPATH` are not distribution verification.

Enable PCVMF's native recording with `logging: {mcap: {directory: recordings}}`;
`examples/fake.yaml` demonstrates this. The plugin needs no special recorder.

Raw forwarding, signing, fleet coordination, and automatic route planning remain
future work. Geofence/rally transfer, parameter enumeration/metadata, string/64-bit
parameters, and body-frame/attitude/acceleration setpoints are also outside the
current API.
