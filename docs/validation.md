# Validation and reproducible simulation checks

This package implements three combinations: pymavlink/PX4, pymavlink/ArduCopter,
and MAVSDK/PX4. MAVSDK/ArduCopter is rejected at configuration validation. A test
against a MAVLink message peer checks framing and transactions; it does not
establish flight behavior. Flight behavior is checked separately in SITL.

## Local evidence

The development environment is Linux x86-64. The dependency lock selects
pymavlink 2.4.50, pyserial 3.5, and mavsdk 3.10.2 with its bundled server.
The firmware sources used for simulator acceptance are:

| Stack | Exact local revision | Vehicle / simulator |
| --- | --- | --- |
| ArduCopter | `Copter-4.5.7-3-gb72075518f` | Quad, built-in SITL dynamics |
| PX4 | `v1.16.0-rc1-745-g3a734bc846` | `10040_sihsim_quadx`, SIH |

Both are development revisions, not a claim of certification across all stable
firmware releases. No physical Pixhawk or ARM64 machine was connected during
development. ARM64 is included in the added CI matrix; a configured job is not
evidence that it has run. Serial verification uses Linux pseudo-terminals and
the real library transports, not USB hardware or radio links.

Initial action/telemetry baseline validation completed on 2026-09-26
(before mission/parameter additions):

| Check | Result |
| --- | --- |
| Python 3.10.20 package suite | 72 passed; one opt-in SITL test skipped |
| Python 3.11.15 package suite | 72 passed; one opt-in SITL test skipped |
| Python 3.12.3 package suite | 72 passed; one opt-in SITL test skipped |
| pymavlink / ArduCopter SITL | Takeoff/land and separate RTL scenarios passed |
| pymavlink / PX4 SIH | Takeoff/land and separate RTL scenarios passed |
| MAVSDK / PX4 SIH | Takeoff/land and separate RTL scenarios passed |
| Ruff and Black checks | Passed |
| Source distribution and wheel builds | Passed |
| Clean installed-wheel smoke outside checkout | Validation, typed-message/action exchange, MCAP recording and shutdown passed |

Flight tests confirmed arm/disarm and physical simulator state in addition to
acknowledgements. Controlled wire tests passed before/after-transmission link
loss and recovery for all three supported pairs. The final polling change was
exercised by the complete Python matrix and the subsequent PX4 RTL runs.
ARM64, real Pixhawk serial/USB, and radio-loss flight behavior remain unverified.

The package suite exercises codec validation, resource-free configuration,
command correlation and expiration, bounded deduplication, command conflicts,
initialization failure, stale telemetry, link recovery without replay, error
propagation, queue overflow, and cleanup. It also runs all supported actions
against a protocol peer through UDP, TCP, and serial, using the actual bundled
MAVSDK server. Spawned PCVMF tests assert received typed payloads, correlated
results, observed state, successful application completion and MCAP output.

## Run the automated suite

From the package directory:

```bash
uv sync --frozen --extra all --extra dev --python 3.12
uv run --no-sync pytest -m 'not sitl'
uv run --no-sync ruff check .
uv run --no-sync black --check .
```

Repeat with Python 3.10 and 3.11. If unrelated ROS pytest plugins are injected
by your shell, prefix the pytest command with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`
and remove that shell's `PYTHONPATH`. The suite does not require an asyncio
pytest plugin. It never starts a physical flight controller automatically.

## Flight acceptance against a running simulator

Start a separate simulator instance using a temporary working directory and
the firmware revision listed above. ArduCopter can use its built-in quad model:

```bash
mkdir -p /tmp/pcvmf-copter
cd /tmp/pcvmf-copter
/path/to/ardupilot/build/sitl/bin/arducopter \
  --model quad --speedup 3 \
  --defaults /path/to/ardupilot/Tools/autotest/default_params/copter.parm \
  --serial0 tcp:15760 --home 47.397742,8.545594,488,0
```

Copy `examples/arducopter-pymavlink.yaml` and set its TCP port to 15760. PX4's
headless SIH model requires no graphical simulator:

```bash
cmake -S /path/to/PX4-Autopilot -B /tmp/pcvmf-px4-build \
  -GNinja -DCONFIG=px4_sitl_default
cmake --build /tmp/pcvmf-px4-build --target px4 -j 4
mkdir -p /tmp/pcvmf-px4-state
PX4_SYS_AUTOSTART=10040 PX4_SIM_MODEL=sihsim_quadx \
  /tmp/pcvmf-px4-build/bin/px4 -d /tmp/pcvmf-px4-build/etc \
  -w /tmp/pcvmf-px4-state
```

The standard PX4 examples receive on UDP 14540. Run only one backend on that
link at a time. Install the firmware's documented build prerequisites first.
Use a fresh simulator state for each acceptance scenario; ArduCopter can reject
arming in the LAND mode left by a preceding scenario. Do not bypass arming
checks to make a test pass.

From the package directory, explicitly opt into the running simulator:

```bash
PCVMF_SITL_CONFIG=/absolute/path/to/simulator.yaml \
  uv run --no-sync pytest tests/test_sitl.py -q -s
PCVMF_SITL_CONFIG=/absolute/path/to/simulator.yaml PCVMF_SITL_RTL=1 \
  uv run --no-sync pytest tests/test_sitl.py -q -s
```

The supplied YAML must contain one flight worker. The test adds a command
producer and enables actions, waits 15 seconds for estimator initialization,
then performs either arm/disarm/arm/takeoff/land/disarm or
arm/takeoff/return-to-launch/disarm. It confirms the corresponding telemetry
state before advancing. Each scenario has a 180-second deadline and records
MCAP plus a JSON result in pytest's temporary directory. Acceptance requires
both scenarios for every supported pair; an ACK alone is insufficient.

Wire fault tests disconnect before a request and after the peer receives an
arm command. They verify cleared telemetry, an unknown in-flight outcome,
new session identity, old-session rejection and no application-level replay.
These controlled transport tests are not a claim that radio failsafe behavior
has been verified on aircraft. Native MAVSDK retries may already be in flight
when a request is cancelled, especially with an externally managed server.

## Mission and parameter acceptance

The complete package suite after these additions passed on Python 3.10.20,
3.11.15, and 3.12.3: **109 passed, 2 skipped** on each interpreter. The two
skips are explicit simulator opt-ins; mission/parameter simulator checks were
run separately as described below. Ruff, Black, and `pcvmf config validate
examples/fake-transactions.yaml` passed. The action/telemetry SITL results above
remain earlier baseline evidence; those flight scenarios were not rerun here.

On 2026-09-26 the new mission/parameter tests passed against the same local
firmware revisions above, separately for pymavlink/PX4, MAVSDK/PX4, and
pymavlink/ArduCopter. Each used a dedicated simulator with fresh temporary state.
The checks uploaded a mission, verified downloaded commands and coordinates,
cleared it and checked the remaining items. They also read integer and float
parameters and wrote their existing values back with confirmation. These checks
did not arm, take off, start a mission, or tune a parameter.

To repeat, start a dedicated simulator using the instructions above and point
the corresponding single-flight-worker example YAML at it. This test replaces
and clears the simulator's ordinary mission; use only disposable simulator state.

```bash
PCVMF_SITL_TRANSACTIONS_CONFIG=/absolute/path/to/simulator.yaml \
  uv run --no-sync pytest tests/test_sitl_transactions.py -q
```

The synthetic protocol tests additionally exercise lost packets, duplicate
requests/late ACKs, typed parameter writes with changed values, PX4 integer
bit-pattern preservation, ArduCopter cast/range rejection, missing parameter
write acknowledgements, wrong mission recipients, mission refusal, oversized
downloads, cancellation, and timeout/session invalidation without replay.
They use the actual bundled MAVSDK server where applicable. The finite
`examples/fake-transactions.yaml` asserts typed request/result exchange and
download/readback values across spawned PCVMF workers, with MCAP and cleanup.

The installed-wheel CI check now also exercises the new fake transaction
example. The original wheel evidence above predates these features; no new
local distribution build is claimed for this implementation pass. Physical
Pixhawk and ARM64 verification remain outstanding.

## Clean wheel verification

Build the framework and plugin wheels, install them together in a clean virtual
environment, and run from outside the checkout with no `PYTHONPATH`:

```bash
uv build ../.. --out-dir /tmp/pcvmf-framework-dist
uv build --out-dir /tmp/pcvmf-mavlink-dist
uv venv /tmp/pcvmf-wheel
uv pip install --python /tmp/pcvmf-wheel/bin/python \
  /tmp/pcvmf-framework-dist/*.whl /tmp/pcvmf-mavlink-dist/*.whl
cp examples/fake.yaml /tmp/pcvmf-wheel/fake.yaml
cd /tmp/pcvmf-wheel
env -u PYTHONPATH bin/pcvmf config validate fake.yaml
env -u PYTHONPATH bin/pcvmf run --config fake.yaml
```

Hardware acceptance remains a separate step: verify telemetry identity and
units over USB/UART, unplug/reconnect behavior, absence of orphaned owned
servers, and compatibility with the actual deployed flight-controller
firmware. Record the board, firmware, OS, architecture and connection settings
alongside the test result before describing a hardware combination as verified.
