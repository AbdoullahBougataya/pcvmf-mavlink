import asyncio
import json
import math
import time
from dataclasses import replace

import pytest
from test_service import event, options

from pcvmf_mavlink.backends.fake import FakeBackend
from pcvmf_mavlink.navigation import (
    ControlRequest,
    ControlRequestCodec,
    ControlResult,
    ControlResultCodec,
    NavigationTelemetry,
    NavigationTelemetryCodec,
    Setpoint,
    SetpointCodec,
    StreamStatus,
    StreamStatusCodec,
)
from pcvmf_mavlink.options import Options
from pcvmf_mavlink.service import FlightService


def point(session="s", stream="stream", sequence=0, **kwargs):
    return Setpoint(session, stream, sequence, time.time() + 0.5, 0, velocity_ned_m_s=[0, 0, 0], **kwargs)


@pytest.mark.parametrize(
    "payload,codec",
    [
        (point(), SetpointCodec),
        (ControlRequest("r", "s", "stream_start", time.time() + 3, "stream", point()), ControlRequestCodec),
        (ControlResult("r", "s", "p", "mission_pause", "accepted", "ok"), ControlResultCodec),
        (StreamStatus("s", "idle", None, None, None, "ready"), StreamStatusCodec),
        (NavigationTelemetry("s"), NavigationTelemetryCodec),
    ],
)
def test_navigation_codecs_roundtrip(payload, codec):
    assert codec().decode(json.loads(json.dumps(codec().encode(payload), allow_nan=False))) == payload


@pytest.mark.parametrize(
    "change",
    [
        {"yaw_rad": math.inf},
        {"yaw_rad": 4},
        {"sequence": True},
        {"sequence": -1},
        {"velocity_ned_m_s": [1, 2]},
        {"velocity_ned_m_s": [True, 0, 0]},
        {"position_ned_m": [0, 0, 0]},
        {"velocity_ned_m_s": None},
    ],
)
def test_reject_invalid_setpoint(change):
    with pytest.raises(ValueError):
        SetpointCodec().encode(replace(point(), **change))


@pytest.mark.parametrize("change", [{"initial_setpoint": point("old")}, {"stream_id": None}, {"mission_index": 1}])
def test_reject_invalid_stream_start(change):
    with pytest.raises(ValueError):
        ControlRequestCodec().encode(
            replace(ControlRequest("r", "s", "stream_start", time.time() + 3, "stream", point()), **change)
        )


@pytest.mark.parametrize(
    "change",
    [
        {"setpoint_hz": 2},
        {"setpoint_timeout_s": 0},
        {"setpoints_enabled": 1},
        {"setpoints_enabled": True, "action_timeout_s": 1},
        {"setpoints_enabled": True, "action_timeout_s": "bad"},
        {"setpoints_enabled": True, "action_timeout_s": None},
        {"setpoint_topic": "flight/result"},
    ],
)
def test_navigation_option_rejection(change):
    with pytest.raises(ValueError):
        options(**change)


def test_stream_owner_sequence_expiry_and_reuse():
    class ArmedFake(FakeBackend):
        async def connect(self):
            await super().connect()
            self.armed = True
            await self.poll()

    service = FlightService(options(setpoints_enabled=True, firmware="arducopter"), ArmedFake)
    try:
        service.start()
        session = service.snapshot()[0].session_id
        initial = point(session)
        request = ControlRequest("r", session, "stream_start", time.time() + 3, "stream", initial)
        service.submit(request, "owner")
        assert event(service, lambda e: isinstance(e, ControlResult)).outcome == "accepted"
        fresh = replace(initial, sequence=1, expires_at=time.time() + 0.5)
        assert not service.submit_setpoint(fresh, "other")
        assert not service.submit_setpoint(replace(fresh, session_id="old"), "owner")
        assert not service.submit_setpoint(replace(fresh, velocity_ned_m_s=[6, 0, 0]), "owner")
        assert service.submit_setpoint(fresh, "owner")
        assert not service.submit_setpoint(fresh, "owner")
        assert not service.submit_setpoint(initial, "owner")
        status = event(service, lambda e: isinstance(e, StreamStatus) and e.state == "expired")
        assert status.last_sequence == 1
        assert not service.submit_setpoint(replace(fresh, sequence=2, expires_at=time.time() + 1), "owner")
        service.submit(
            replace(
                request,
                request_id="new",
                expires_at=time.time() + 3,
                initial_setpoint=replace(initial, expires_at=time.time() + 1),
            ),
            "owner",
        )
        assert event(service, lambda e: isinstance(e, ControlResult)).outcome == "rejected"
    finally:
        service.close()
    assert not service.thread.is_alive()


@pytest.mark.parametrize("backend,firmware", [("pymavlink", "px4"), ("pymavlink", "arducopter"), ("mavsdk", "px4")])
def test_wire_stream_and_mission_controls(backend, firmware):
    pytest.importorskip("pymavlink")
    if backend == "mavsdk":
        pytest.importorskip("mavsdk")
    from navigation_peer import NavigationPeer

    async def scenario():
        peer = NavigationPeer(firmware)
        config = Options.parse(
            dict(
                backend=backend,
                firmware=firmware,
                connection=await peer.open(),
                setpoints_enabled=True,
                mission_execution_enabled=True,
                commands_enabled=True,
                connect_timeout_s=12,
                telemetry_stale_s=2,
            )
        )
        service = FlightService(config)
        try:
            await asyncio.to_thread(service.start)
            session = service.snapshot()[0].session_id
            end = time.monotonic() + 3
            while service.navigation_snapshot()[1] is None or "local" not in service.navigation_snapshot()[1].samples:
                assert time.monotonic() < end
                await asyncio.sleep(0.02)
            initial = point(session)
            service.submit(
                ControlRequest("start", session, "stream_start", time.time() + 6, "stream", initial), "owner"
            )

            async def refresh():
                for seq in range(1, 70):
                    await asyncio.sleep(0.04)
                    service.submit_setpoint(
                        replace(
                            initial,
                            sequence=seq,
                            expires_at=time.time() + 0.5,
                            velocity_ned_m_s=[0.5, 0, 0],
                            yaw_rad=0.25,
                        ),
                        "owner",
                    )

            updating = asyncio.create_task(refresh())
            result = await asyncio.to_thread(event, service, lambda e: isinstance(e, ControlResult), 7)
            assert result.outcome == "accepted", result
            await updating
            transitions = []

            def expired_event(value):
                if isinstance(value, StreamStatus):
                    transitions.append(value)
                return isinstance(value, StreamStatus) and value.state == "expired"

            try:
                expired = await asyncio.to_thread(event, service, expired_event, 4)
            except AssertionError as exc:
                raise AssertionError(f"stream expiry transitions: {transitions}") from exc
            assert expired.last_sequence > 1
            count = len(peer.setpoints)
            await asyncio.sleep(0.2)
            assert len(peer.setpoints) == count
            packet = peer.setpoints[-1][1]
            assert (packet.target_system, packet.target_component, packet.coordinate_frame, packet.type_mask) == (
                1,
                1,
                1,
                2503,
            )
            assert packet.vx == 0.5 and packet.yaw == pytest.approx(0.25)
            peer.items = [{}, {}, {}]
            for index, operation in enumerate(("mission_start", "mission_pause", "mission_set_current")):
                request = ControlRequest(
                    str(index),
                    session,
                    operation,
                    time.time() + 5,
                    mission_index=2 if operation == "mission_set_current" else None,
                )
                service.submit(request, "owner")
                result = await asyncio.to_thread(event, service, lambda e: isinstance(e, ControlResult), 6)
                assert result.outcome == "accepted", result
            end = time.monotonic() + 2
            while service.navigation_snapshot()[1].samples["mission"].values["current_index"] != 2:
                assert time.monotonic() < end
                await asyncio.sleep(0.02)
            assert not any(m.command == 400 for m in peer.commands), "navigation must never implicitly arm"
        finally:
            await asyncio.to_thread(service.close)
            await peer.close()

    asyncio.run(scenario())
