import subprocess
import sys
from dataclasses import asdict, replace

import pytest

from pcvmf_mavlink.messages import (
    ActionRequest,
    ActionRequestCodec,
    ActionResult,
    ActionResultCodec,
    ConnectionStatus,
    ConnectionStatusCodec,
    TelemetrySample,
    VehicleTelemetry,
    VehicleTelemetryCodec,
)
from pcvmf_mavlink.options import Options


@pytest.mark.parametrize(
    "codec,payload",
    [
        (ActionRequestCodec(), ActionRequest("req", "session", "takeoff", 100, 2)),
        (ActionResultCodec(), ActionResult("req", "session", "producer", "arm", "accepted", "ACK")),
        (
            ConnectionStatusCodec(),
            ConnectionStatus("connected", "pymavlink", "px4", 1, 1, "session", ["arm"], True, "connected"),
        ),
        (
            VehicleTelemetryCodec(),
            VehicleTelemetry(
                "session", 1, 1, {"battery": TelemetrySample(100, 0.5, False, {"remaining_fraction": None})}
            ),
        ),
    ],
)
def test_roundtrip_and_strict_fields(codec, payload):
    assert codec.decode(codec.encode(payload)) == payload
    raw = asdict(payload)
    with pytest.raises(ValueError):
        codec.decode({**raw, "extra": 1})
    raw.pop(next(iter(raw)))
    with pytest.raises(ValueError):
        codec.decode(raw)


@pytest.mark.parametrize(
    "changes",
    [
        {"expires_at": float("nan")},
        {"expires_at": True},
        {"request_id": ""},
        {"action": "kill"},
        {"takeoff_altitude_m": 2},
        {"session_id": 1},
    ],
)
def test_invalid_requests_rejected_on_both_paths(changes):
    payload = replace(ActionRequest("req", "session", "arm", 100), **changes)
    codec = ActionRequestCodec()
    with pytest.raises(ValueError):
        codec.encode(payload)
    with pytest.raises(ValueError):
        codec.decode(asdict(payload))


@pytest.mark.parametrize(
    "group,values",
    [
        ("position", {"latitude_deg": 91}),
        ("battery", {"remaining_fraction": 80}),
        ("attitude", {"roll_rad": float("inf")}),
        ("state", {"armed": 1}),
        ("gps", {"satellites_visible": True}),
        ("health", {"armable": "yes"}),
    ],
)
def test_invalid_telemetry(group, values):
    with pytest.raises(ValueError):
        VehicleTelemetryCodec().encode(VehicleTelemetry("s", 1, 1, {group: TelemetrySample(1, 0, False, values)}))


@pytest.mark.parametrize(
    "changes",
    [
        {"backend": "mavsdk", "firmware": "arducopter"},
        {"commands_enabled": 1},
        {"target_system": True},
        {"source_system": 1},
        {"connect_timeout_s": float("nan")},
        {"receive_budget_ms": 0},
        {"foo": 1},
        {"server": {"port": 50051}},
        {"result_topic": "flight/status"},
        {"connection": {"transport": "http"}},
    ],
)
def test_invalid_options(changes):
    with pytest.raises(ValueError):
        Options.parse({"backend": "fake", "firmware": "px4", "connection": {}, **changes})


@pytest.mark.parametrize(
    "backend,transport,address",
    [
        ("pymavlink", "udpin", "udpin:127.0.0.1:14540"),
        ("mavsdk", "udpin", "udpin://127.0.0.1:14540"),
        ("mavsdk", "tcp", "tcpout://127.0.0.1:14540"),
    ],
)
def test_connection_conversion(backend, transport, address):
    o = Options.parse(
        {
            "backend": backend,
            "firmware": "px4",
            "connection": {"transport": transport, "host": "127.0.0.1", "port": 14540},
        }
    )
    assert o.address() == address


def test_import_and_validation_do_not_import_optional_libraries():
    code = """
import sys
from pcvmf_mavlink.worker import FlightControllerWorker
FlightControllerWorker.validate_options({'backend':'mavsdk','firmware':'px4',
    'connection':{'transport':'udpin','host':'127.0.0.1','port':14540}})
FlightControllerWorker({'backend':'fake','firmware':'px4','connection':{}})
assert 'mavsdk' not in sys.modules and 'pymavlink' not in sys.modules
"""
    subprocess.run([sys.executable, "-c", code], check=True, timeout=10)
