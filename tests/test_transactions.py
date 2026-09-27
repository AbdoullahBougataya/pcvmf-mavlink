import asyncio
import json
import time
from dataclasses import replace

import pytest
from pcvmf.api import Message
from test_service import event, options

from pcvmf_mavlink.messages import ActionRequest, ConnectionStatus
from pcvmf_mavlink.options import Options
from pcvmf_mavlink.service import FlightService
from pcvmf_mavlink.transactions import (
    MissionItem,
    MissionRequest,
    MissionRequestCodec,
    MissionResult,
    MissionResultCodec,
    ParameterRequest,
    ParameterRequestCodec,
    ParameterResult,
    ParameterResultCodec,
)

pytest_plugins = ["test_worker"]

ITEMS = [MissionItem(6, 16, 473977420, 85455940, 10, current=1), MissionItem(6, 21, 473977420, 85455940, 0)]


@pytest.mark.parametrize(
    "payload,codec",
    [
        (MissionRequest("r", "s", "upload", 123, ITEMS), MissionRequestCodec),
        (MissionResult("r", "s", "p", "download", "accepted", "ok", ITEMS), MissionResultCodec),
        (ParameterRequest("r", "s", "set", 123, "ABCDEFGHIJKLMNOP", "int", -(2**31)), ParameterRequestCodec),
        (ParameterResult("r", "s", "p", "get", "accepted", "ok", "TEST_FLOAT", "float", 1.25), ParameterResultCodec),
    ],
)
def test_new_codec_json_roundtrip(payload, codec):
    raw = json.loads(json.dumps(codec().encode(payload), allow_nan=False))
    assert codec().decode(raw) == payload
    with pytest.raises(ValueError):
        codec().decode({**raw, "extra": True})


@pytest.mark.parametrize(
    "change",
    [
        {"name": "a" * 17},
        {"name": "é"},
        {"name": "A\0B"},
        {"name": "a b"},
        {"value": True},
        {"value": 2**31},
        {"value": None},
        {"value": 1.5},
        {"parameter_type": "float", "value": float("nan")},
        {"parameter_type": "float", "value": 1e39},
        {"operation": "get"},
    ],
)
def test_parameter_codec_rejects_invalid_values(change):
    payload = replace(ParameterRequest("r", "s", "set", 123, "TEST_INT", "int", 2), **change)
    with pytest.raises(ValueError):
        ParameterRequestCodec().encode(payload)


@pytest.mark.parametrize(
    "change", [{"x": 2**31}, {"frame": True}, {"current": 2}, {"z": float("inf")}, {"param4": float("nan")}]
)
def test_mission_codec_rejects_invalid_fields(change):
    with pytest.raises(ValueError):
        MissionRequestCodec().encode(MissionRequest("r", "s", "upload", 123, [replace(ITEMS[0], **change)]))


@pytest.mark.parametrize(
    "change",
    [
        {"missions_enabled": 1},
        {"parameter_writes_enabled": True},
        {"transfer_timeout_s": 0},
        {"max_mission_items": 10001},
    ],
)
def test_new_option_validation(change):
    with pytest.raises(ValueError):
        Options.parse({"backend": "fake", "firmware": "px4", "connection": {}, **change})


def msg(request):
    return Message("flight/command", "unused", 1, "producer", 0, 100, request)


def test_transaction_gates_deduplication_and_cross_type_conflicts(worker):
    mission = MissionRequest("r", "session", "upload", 120, ITEMS)
    worker._command(msg(mission))
    assert worker.outputs[-1].outcome == "disabled"
    worker.config = replace(worker.config, missions_enabled=True, parameters_enabled=True)
    mission = replace(mission, request_id="enabled")
    worker._command(msg(mission))
    worker._command(msg(mission))
    assert len(worker.service.sent) == 1
    worker._command(msg(ActionRequest("enabled", "session", "arm", 120)))
    assert worker.outputs[-1].outcome == "request_conflict"
    worker._command(msg(ParameterRequest("get", "session", "get", 120, "TEST_INT", "int")))
    assert worker.outputs[-1].outcome == "busy"
    worker._command(msg(ParameterRequest("set", "session", "set", 120, "TEST_INT", "int", 4)))
    assert worker.outputs[-1].outcome == "disabled"
    result = MissionResult("enabled", "session", "producer", "upload", "accepted", "ok")
    worker.service.events.put(result)
    worker.step()
    worker._command(msg(mission))
    assert worker.outputs[-1] == result
    worker._command(msg(ParameterRequest("old", "old-session", "get", 120, "TEST_INT", "int")))
    assert worker.outputs[-1].outcome == "session_mismatch"
    worker._command(msg(MissionRequest("expired", "session", "clear", 99)))
    assert worker.outputs[-1].outcome == "expired"


@pytest.mark.parametrize("kind", ["mission", "parameter"])
def test_new_transaction_timeout_resets_session_without_replay(kind):
    from pcvmf_mavlink.backends.fake import FakeBackend

    calls = []

    class Slow(FakeBackend):
        async def transfer_mission(self, request):
            calls.append(request)
            await asyncio.sleep(10)

        parameter = transfer_mission

    service = FlightService(
        options(missions_enabled=True, parameters_enabled=True, transfer_timeout_s=0.04, parameter_timeout_s=0.04), Slow
    )
    try:
        service.start()
        session = service.snapshot()[0].session_id
        request = (
            MissionRequest("r", session, "clear", time.time() + 5)
            if kind == "mission"
            else ParameterRequest("r", session, "get", time.time() + 5, "TEST_INT", "int")
        )
        service.submit(request, "producer")
        assert event(service, lambda e: isinstance(e, (MissionResult, ParameterResult))).outcome == "outcome_unknown"
        event(service, lambda e: isinstance(e, ConnectionStatus) and e.state == "connected" and e.session_id != session)
        assert len(calls) == 1
    finally:
        service.close()
    assert not service.thread.is_alive()


@pytest.mark.parametrize("backend,firmware", [("pymavlink", "px4"), ("pymavlink", "arducopter"), ("mavsdk", "px4")])
def test_wire_mission_and_parameter_roundtrips(backend, firmware):
    pytest.importorskip("pymavlink")
    if backend == "mavsdk":
        pytest.importorskip("mavsdk")
    from transaction_peer import TransactionPeer

    from pcvmf_mavlink.backends import create

    async def scenario():
        peer = TransactionPeer(firmware)
        adapter = create(Options.parse({"backend": backend, "firmware": firmware, "connection": await peer.open()}))
        polling = None
        try:
            await asyncio.wait_for(adapter.connect(), 12)

            async def poll():
                while True:
                    await adapter.poll()

            polling = asyncio.create_task(poll())
            peer.drop_once = {
                "MISSION_COUNT",
                "MISSION_ITEM_INT",
                "MISSION_REQUEST_LIST",
                "MISSION_REQUEST_INT",
                "MISSION_CLEAR_ALL",
                "PARAM_REQUEST_READ",
            }
            peer.duplicate_request = True
            for operation in ("upload", "download", "clear", "download"):
                request = MissionRequest("r", "s", operation, time.time() + 15, ITEMS if operation == "upload" else [])
                result = await asyncio.wait_for(adapter.transfer_mission(request), 15)
                assert result[0] == "accepted", (
                    operation,
                    result,
                    [(m.get_type(), getattr(m, "seq", None)) for m in peer.transactions],
                )
                if operation == "download":
                    assert result[2] == (ITEMS if peer.items else [])
            peer.drop_parameter_echo = firmware == "arducopter"
            for name, typ, value in [("TEST_INT", "int", -12345), ("TEST_FLOAT", "float", 3.5)]:
                for operation in ("get", "set", "get"):
                    request = ParameterRequest(
                        "r", "s", operation, time.time() + 10, name, typ, value if operation == "set" else None
                    )
                    result = await asyncio.wait_for(adapter.parameter(request), 10)
                    assert result[0] == "accepted", result
                    if operation == "set":
                        assert result[2] == value
                assert peer.params[name][1] == value
            if firmware == "px4":
                # Includes a signalling-NaN bit pattern in the wire float field.
                for value in (0x7F800001, -(2**31), 2**31 - 1):
                    result = await asyncio.wait_for(
                        adapter.parameter(
                            ParameterRequest("bits", "s", "set", time.time() + 10, "TEST_INT", "int", value)
                        ),
                        10,
                    )
                    assert result == ("accepted", "parameter value observed on the autopilot", value)
                    assert peer.params["TEST_INT"][1] == value
            before = len([m for m in peer.transactions if m.get_type() == "PARAM_SET"])
            service = FlightService(replace(adapter.options, parameters_enabled=True, parameter_writes_enabled=True))
            result = await service._execute(
                adapter, ParameterRequest("wrong", "s", "set", time.time() + 10, "TEST_FLOAT", "int", 2), "producer"
            )
            assert result.outcome == "rejected"
            if firmware == "arducopter":
                for name, value in (("TEST_INT", 2**24 + 1), ("TEST_SMALL", 128)):
                    result = await service._execute(
                        adapter, ParameterRequest("range", "s", "set", time.time() + 10, name, "int", value), "producer"
                    )
                    assert result.outcome == "rejected"
            assert len([m for m in peer.transactions if m.get_type() == "PARAM_SET"]) == before
            peer.reject_mission = True
            result = await asyncio.wait_for(
                adapter.transfer_mission(MissionRequest("deny", "s", "clear", time.time() + 5)), 5
            )
            assert result[0] == "rejected"
            assert not any(m.command in (20, 21, 22, 176, 400, 300) for m in peer.commands)
            if polling.done():
                await polling
        finally:
            if polling:
                polling.cancel()
                await asyncio.gather(polling, return_exceptions=True)
            await adapter.close()
            await peer.close()
        if backend == "mavsdk":
            assert adapter.process.returncode is not None

    asyncio.run(scenario())
