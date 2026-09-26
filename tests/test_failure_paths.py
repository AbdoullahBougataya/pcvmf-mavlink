import asyncio
import queue
import time
from types import SimpleNamespace

import pytest

from pcvmf_mavlink.backends.fake import FakeBackend
from pcvmf_mavlink.messages import ActionRequest, ActionResult
from pcvmf_mavlink.options import Options
from pcvmf_mavlink.service import FlightService


def test_overflow_fails_visibly_and_still_closes_adapter():
    instances = []

    class Adapter(FakeBackend):
        async def connect(self):
            instances.append(self)
            self.closed = False
            await super().connect()

        async def close(self):
            self.closed = True

    o = Options.parse({"backend": "fake", "firmware": "px4", "connection": {}, "event_queue_size": 1})
    s = FlightService(o, Adapter)
    s.start()  # The one queue slot now contains the connected status.
    s.submit(ActionRequest("r", s.snapshot()[0].session_id, "arm", time.time() + 3), "producer")
    s.thread.join(2)
    assert isinstance(s.error, queue.Full)
    assert instances[0].closed
    with pytest.raises(RuntimeError):
        s.close()


def test_ack_filters_target_and_command_then_waits_for_final():
    pytest.importorskip("pymavlink")
    from pymavlink.dialects.v20 import ardupilotmega as mav

    from pcvmf_mavlink.backends.pymavlink import PymavlinkBackend

    async def scenario():
        o = Options.parse(
            {
                "backend": "pymavlink",
                "firmware": "px4",
                "connection": {"transport": "udpin", "host": "127.0.0.1", "port": 12345},
            }
        )
        adapter = PymavlinkBackend(o)
        adapter.command_id = 400
        adapter.ack = asyncio.get_running_loop().create_future()
        adapter._message(mav.MAVLink_command_ack_message(22, 0, 0, 0, 245, 190))
        adapter._message(mav.MAVLink_command_ack_message(400, 0, 0, 0, 99, 190))
        adapter._message(mav.MAVLink_command_ack_message(400, 0, 0, 0, 245, 99))
        adapter._message(mav.MAVLink_command_ack_message(400, 5, 50, 0, 245, 190))
        assert not adapter.ack.done()
        adapter._message(mav.MAVLink_command_ack_message(400, 2, 0, 0, 245, 190))
        assert adapter.ack.result() == 2

    asyncio.run(scenario())


def test_mavsdk_external_server_cleanup_never_owns_a_process():
    pytest.importorskip("mavsdk")
    from pcvmf_mavlink.backends.mavsdk import MavsdkBackend

    async def scenario():
        o = Options.parse(
            {
                "backend": "mavsdk",
                "firmware": "px4",
                "connection": {"transport": "udpin", "host": "127.0.0.1", "port": 14540},
                "server": {"host": "127.0.0.1", "port": 1},
            }
        )
        adapter = MavsdkBackend(o)
        try:
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(adapter.connect(), 0.1)
        finally:
            await adapter.close()
        assert adapter.process is None
        assert adapter.channel is not None

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "sdk_result,outcome",
    [("TIMEOUT", "outcome_unknown"), ("COMMAND_DENIED", "rejected"), ("BUSY", "busy"), ("UNSUPPORTED", "unsupported")],
)
def test_mavsdk_error_normalization(sdk_result, outcome):
    pytest.importorskip("mavsdk")
    from mavsdk.action import ActionError
    from mavsdk.action import ActionResult as SdkResult

    from pcvmf_mavlink.backends.mavsdk import MavsdkBackend

    async def scenario():
        o = Options.parse(
            {
                "backend": "mavsdk",
                "firmware": "px4",
                "connection": {"transport": "udpin", "host": "127.0.0.1", "port": 14540},
            }
        )
        adapter = MavsdkBackend(o)

        async def arm():
            raise ActionError(SdkResult(getattr(SdkResult.Result, sdk_result), sdk_result), "arm")

        adapter.action = SimpleNamespace(arm=arm)
        result = await adapter.execute(ActionRequest("r", "s", "arm", time.time() + 5))
        assert result[0] == outcome

    asyncio.run(scenario())


def test_stale_telemetry_is_explicit_and_cleared_on_loss():
    o = Options.parse({"backend": "fake", "firmware": "px4", "connection": {}, "telemetry_stale_s": 0.1})
    adapter = FakeBackend(o)
    adapter.update("state", {"armed": True, "flight_mode": "HOLD"})
    adapter.sample_times["state"] -= 1
    s = FlightService(o)
    s._telemetry(adapter, "session")
    assert s.snapshot()[1].samples["state"].stale
    assert s.snapshot()[1].samples["state"].age_s >= 1
    s._status("disconnected", "session", "timeout")
    assert s.snapshot()[1] is None


def test_old_session_command_rejected_in_io_thread():
    o = Options.parse({"backend": "fake", "firmware": "px4", "connection": {}})
    s = FlightService(o)
    try:
        s.start()
        s.submit(ActionRequest("r", "old-session", "arm", time.time() + 5), "producer")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            item = s.events.get(timeout=1)
            if isinstance(item, ActionResult):
                assert item.outcome == "session_mismatch"
                break
        else:
            pytest.fail("no session rejection")
    finally:
        s.close()
