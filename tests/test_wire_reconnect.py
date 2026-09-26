import asyncio
import queue
import time

import pytest

pytest.importorskip("pymavlink")

from vehicle_peer import VehiclePeer

from pcvmf_mavlink.messages import ActionRequest, ActionResult, ConnectionStatus
from pcvmf_mavlink.options import Options
from pcvmf_mavlink.service import FlightService


async def next_event(service, predicate):
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        service.check()
        try:
            event = service.events.get_nowait()
        except queue.Empty:
            await asyncio.sleep(0.01)
            continue
        if predicate(event):
            return event
    raise AssertionError("expected wire-level recovery event did not arrive")


@pytest.mark.parametrize("backend,firmware", [("pymavlink", "px4"), ("pymavlink", "arducopter"), ("mavsdk", "px4")])
@pytest.mark.parametrize("after_transmission", [False, True])
def test_link_loss_changes_session_and_does_not_replay(backend, firmware, after_transmission):
    if backend == "mavsdk":
        pytest.importorskip("mavsdk")

    async def scenario():
        peer = VehiclePeer(firmware)
        connection = await peer.open()
        options = Options.parse(
            {
                "backend": backend,
                "firmware": firmware,
                "connection": connection,
                "commands_enabled": True,
                "connect_timeout_s": 10,
                "link_timeout_s": 0.3,
                "reconnect_initial_s": 0.05,
                "reconnect_max_s": 0.1,
            }
        )
        service = FlightService(options)
        try:
            await asyncio.to_thread(service.start)
            status = await next_event(service, lambda e: isinstance(e, ConnectionStatus) and e.state == "connected")
            request = ActionRequest("r", status.session_id, "arm", time.time() + 10)
            if after_transmission:
                peer.drop_after_action = True
                service.submit(request, "producer")
            else:
                peer.transmit = False
            await next_event(service, lambda e: isinstance(e, ConnectionStatus) and e.state == "disconnected")
            assert service.snapshot()[1] is None
            if after_transmission:
                result = await next_event(service, lambda e: isinstance(e, ActionResult))
                assert result.outcome == "outcome_unknown"
                assert peer.armed  # Proves timeout cannot be equated with rejection.
            peer.drop_after_action = False
            peer.transmit = True
            recovered = await next_event(service, lambda e: isinstance(e, ConnectionStatus) and e.state == "connected")
            assert recovered.session_id != status.session_id
            command_count = sum(m.command == 400 for m in peer.commands)
            service.submit(request, "producer")  # Delayed request from the old session.
            result = await next_event(service, lambda e: isinstance(e, ActionResult))
            assert result.outcome == "session_mismatch"
            await asyncio.sleep(0.1)
            assert sum(m.command == 400 for m in peer.commands) == command_count
            assert command_count >= 1 if after_transmission else command_count == 0
        finally:
            try:
                await asyncio.to_thread(service.close)
            finally:
                await peer.close()
        assert not service.thread.is_alive()

    asyncio.run(scenario())
