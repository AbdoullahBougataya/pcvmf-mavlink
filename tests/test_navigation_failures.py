import asyncio
import time
from dataclasses import replace

import pytest
from test_service import event, options

from pcvmf_mavlink.backends.fake import FakeBackend
from pcvmf_mavlink.messages import ActionRequest, ActionResult, ConnectionStatus
from pcvmf_mavlink.navigation import ControlRequest, ControlResult, Setpoint, StreamStatus
from pcvmf_mavlink.service import FlightService


class ArmedRecorder(FakeBackend):
    async def connect(self):
        await super().connect()
        self.armed = True
        self.sent = []
        self.modes = []
        self.closed = False
        await self.poll()

    async def send_setpoint(self, point):
        self.sent.append(point)
        await super().send_setpoint(point)

    async def stream_mode(self, enabled):
        self.modes.append(enabled)
        return await super().stream_mode(enabled)

    async def close(self):
        self.closed = True


def start(service, expires=0.5):
    session = service.snapshot()[0].session_id
    point = Setpoint(session, "stream", 0, time.time() + expires, 0, velocity_ned_m_s=[0, 0, 0])
    request = ControlRequest("start", session, "stream_start", time.time() + 5, "stream", point)
    service.submit(request, "owner")
    return request


def test_expiry_during_px4_priming_never_changes_mode():
    adapters = []

    def factory(o):
        adapter = ArmedRecorder(o)
        adapters.append(adapter)
        return adapter

    service = FlightService(options(setpoints_enabled=True, setpoint_timeout_s=0.15), factory)
    try:
        service.start()
        start(service, 0.15)
        result = event(service, lambda e: isinstance(e, ControlResult))
        assert result.outcome == "rejected"
        assert adapters[0].modes == []
        assert service.navigation_snapshot()[0].state == "expired"
    finally:
        service.close()
    assert all(a.closed for a in adapters)


def test_pilot_mode_change_invalidates_stream_without_overriding_mode():
    adapters = []

    def factory(o):
        adapter = ArmedRecorder(o)
        adapters.append(adapter)
        return adapter

    service = FlightService(options(setpoints_enabled=True, firmware="arducopter"), factory)
    try:
        service.start()
        request = start(service)
        assert event(service, lambda e: isinstance(e, ControlResult)).outcome == "accepted"
        adapters[0].mode = "LOITER"
        event(service, lambda e: isinstance(e, StreamStatus) and e.state == "lost")
        event(
            service,
            lambda e: isinstance(e, ConnectionStatus) and e.state == "connected" and e.session_id != request.session_id,
        )
        assert adapters[0].modes == [True]
        assert adapters[0].closed
        assert not service.submit_setpoint(
            replace(request.initial_setpoint, sequence=1, expires_at=time.time() + 1), "owner"
        )
        assert adapters[1].sent == [] and adapters[1].modes == []
    finally:
        service.close()


@pytest.mark.parametrize("action", ["land", "return_to_launch", "disarm"])
def test_terminal_action_supersedes_stream_without_intermediate_mode_command(action):
    adapters = []

    def factory(o):
        adapter = ArmedRecorder(o)
        adapters.append(adapter)
        return adapter

    service = FlightService(options(setpoints_enabled=True, firmware="arducopter"), factory)
    try:
        service.start()
        request = start(service)
        assert event(service, lambda e: isinstance(e, ControlResult)).outcome == "accepted"
        service.submit(ActionRequest("action", request.session_id, action, time.time() + 5), "other-authorized-worker")
        assert event(service, lambda e: isinstance(e, ActionResult)).outcome == "accepted"
        assert service.navigation_snapshot()[0].state == "stopped"
        assert adapters[0].modes == [True]
        assert not service.submit_setpoint(
            replace(request.initial_setpoint, sequence=1, expires_at=time.time() + 1), "owner"
        )
    finally:
        service.close()


def test_hold_rejection_on_expiry_resets_connection_and_stops_emission():
    class RefuseHold(ArmedRecorder):
        async def stream_mode(self, enabled):
            if not enabled:
                return "rejected", "hold refused"
            return await super().stream_mode(enabled)

    service = FlightService(options(setpoints_enabled=True, firmware="arducopter", setpoint_timeout_s=0.15), RefuseHold)
    try:
        service.start()
        request = start(service, 0.15)
        assert event(service, lambda e: isinstance(e, ControlResult)).outcome == "accepted"
        event(service, lambda e: isinstance(e, StreamStatus) and e.state == "lost")
        event(
            service,
            lambda e: isinstance(e, ConnectionStatus) and e.state == "connected" and e.session_id != request.session_id,
        )
        assert service.navigation_snapshot()[0].state == "idle"
    finally:
        service.close()


@pytest.mark.parametrize("lease,expected_state", [(0.15, "expired"), (0.8, "lost")])
def test_pending_send_distinguishes_lease_expiry_from_transport_timeout(lease, expected_state):
    adapters = []

    class BlockedSend(ArmedRecorder):
        async def send_setpoint(self, point):
            if self.mode == "GUIDED":
                await asyncio.sleep(10)
            await super().send_setpoint(point)

    def factory(o):
        adapter = BlockedSend(o)
        adapters.append(adapter)
        return adapter

    service = FlightService(options(setpoints_enabled=True, firmware="arducopter", setpoint_timeout_s=lease), factory)
    try:
        service.start()
        request = start(service, lease)
        assert event(service, lambda e: isinstance(e, ControlResult)).outcome == "accepted"
        status = event(service, lambda e: isinstance(e, StreamStatus) and e.state in ("expired", "lost"))
        assert status.state == expected_state, status
        if expected_state == "expired":
            assert adapters[0].modes == [True, False]
            assert service.snapshot()[0].session_id == request.session_id
        else:
            event(
                service,
                lambda e: isinstance(e, ConnectionStatus)
                and e.state == "connected"
                and e.session_id != request.session_id,
            )
            assert adapters[0].closed
        count = len(adapters[0].sent)
        time.sleep(0.1)
        assert len(adapters[0].sent) == count
    finally:
        service.close()
