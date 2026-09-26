import asyncio
import queue
import time

import pytest

from pcvmf_mavlink.backends.base import LinkLost
from pcvmf_mavlink.backends.fake import FakeBackend
from pcvmf_mavlink.messages import ActionRequest, ActionResult, ConnectionStatus
from pcvmf_mavlink.options import Options
from pcvmf_mavlink.service import FlightService


def options(**kwargs):
    return Options.parse(
        {
            "backend": "fake",
            "firmware": "px4",
            "connection": {},
            "commands_enabled": True,
            "reconnect_initial_s": 0.02,
            "reconnect_max_s": 0.04,
            "connect_timeout_s": 0.3,
            **kwargs,
        }
    )


def event(service, predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        service.check()
        try:
            item = service.events.get(timeout=0.02)
        except queue.Empty:
            continue
        if predicate(item):
            return item
    raise AssertionError("expected service event did not arrive")


def test_disconnect_during_action_reconnects_without_replay():
    instances = []

    class Drops(FakeBackend):
        async def execute(self, request):
            self.running = True
            self.executions += 1
            await asyncio.sleep(10)

        async def poll(self):
            await super().poll()
            if self.running:
                raise LinkLost("simulated cable removal")

        async def close(self):
            self.closed = True

    def factory(o):
        adapter = Drops(o)
        adapter.running, adapter.closed, adapter.executions = False, False, 0
        instances.append(adapter)
        return adapter

    s = FlightService(options(), factory)
    try:
        s.start()
        original = s.snapshot()[0].session_id
        s.submit(ActionRequest("req", original, "arm", time.time() + 5), "producer")
        result = event(s, lambda e: isinstance(e, ActionResult))
        assert result.outcome == "outcome_unknown"
        status = event(
            s, lambda e: isinstance(e, ConnectionStatus) and e.state == "connected" and e.session_id != original
        )
        assert status.session_id != original
        assert sum(a.executions for a in instances) == 1
        assert instances[0].closed
    finally:
        s.close()
    assert not s.thread.is_alive()
    assert all(a.closed for a in instances)


def test_timeout_invalidates_session():
    class Slow(FakeBackend):
        async def execute(self, request):
            await asyncio.sleep(5)

    s = FlightService(options(action_timeout_s=0.04), Slow)
    try:
        s.start()
        session = s.snapshot()[0].session_id
        s.submit(ActionRequest("req", session, "arm", time.time() + 3), "producer")
        assert event(s, lambda e: isinstance(e, ActionResult)).outcome == "outcome_unknown"
        event(s, lambda e: isinstance(e, ConnectionStatus) and e.state == "connected" and e.session_id != session)
    finally:
        s.close()


def test_initial_timeout_releases_partially_initialized_adapter():
    adapters = []

    class Hangs(FakeBackend):
        async def connect(self):
            adapters.append(self)
            self.closed = False
            await asyncio.sleep(10)

        async def close(self):
            self.closed = True

    s = FlightService(options(connect_timeout_s=0.03), Hangs)
    with pytest.raises(RuntimeError):
        s.start()
    with pytest.raises(RuntimeError):
        s.close()
    assert adapters[0].closed and not s.thread.is_alive()


def test_expired_before_io_is_not_executed():
    s = FlightService(options())
    try:
        s.start()
        s.submit(ActionRequest("req", s.snapshot()[0].session_id, "arm", time.time() - 1), "producer")
        assert event(s, lambda e: isinstance(e, ActionResult)).outcome == "expired"
    finally:
        s.close()
