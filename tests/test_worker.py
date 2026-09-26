import queue
from dataclasses import replace
from types import SimpleNamespace

import pytest
from pcvmf.api import Message, WorkerContext

from pcvmf_mavlink.messages import ActionRequest, ActionResult, ConnectionStatus
from pcvmf_mavlink.worker import FlightControllerWorker


class MemoryService:
    def __init__(self, options):
        self.events = queue.Queue()
        self.sent = []
        self.status = ConnectionStatus("connected", "fake", "px4", 1, 1, "session", ["arm"], True, "connected")

    def start(self):
        pass

    def close(self):
        pass

    def check(self):
        pass

    def snapshot(self):
        return self.status, None

    def submit(self, request, source):
        self.sent.append((request, source))


@pytest.fixture
def worker(monkeypatch):
    monkeypatch.setattr("pcvmf_mavlink.worker.FlightService", MemoryService)
    outputs = []
    w = FlightControllerWorker({"backend": "fake", "firmware": "px4", "connection": {}, "commands_enabled": True})
    w.initialize(
        WorkerContext(
            "flight",
            50,
            SimpleNamespace(publish=lambda topic, payload: outputs.append(payload)),
            wall_clock=lambda: 100,
        )
    )
    w.outputs = outputs
    yield w
    w.cleanup()


def message(request=None, source="producer"):
    return Message(
        "flight/command",
        "pcvmf_mavlink.action_request",
        1,
        source,
        0,
        100,
        request or ActionRequest("req", "session", "arm", 110),
    )


def test_duplicate_requests_replay_result_without_resending(worker):
    m = message()
    worker._command(m)
    worker._command(m)
    assert len(worker.service.sent) == 1
    assert [x.outcome for x in worker.outputs] == ["processing", "processing"]
    worker.service.events.put(ActionResult("req", "session", "producer", "arm", "accepted", "ACK"))
    worker.step()
    worker._command(m)
    assert worker.outputs[-1].outcome == "accepted"
    assert worker.pending is None and len(worker.service.sent) == 1


@pytest.mark.parametrize(
    "command,outcome",
    [
        (ActionRequest("req", "old", "arm", 110), "session_mismatch"),
        (ActionRequest("req", "session", "arm", 99), "expired"),
    ],
)
def test_invalid_command_never_submitted(worker, command, outcome):
    worker._command(message(command))
    assert worker.outputs[-1].outcome == outcome
    assert worker.service.sent == []


def test_request_conflict_busy_and_source_correlation(worker):
    worker._command(message())
    worker._command(message(replace(message().payload, action="disarm")))
    assert worker.outputs[-1].outcome == "request_conflict"
    worker._command(message(source="another_producer"))
    assert worker.outputs[-1].outcome == "busy"
    assert len(worker.service.sent) == 1


def test_disconnected_and_disabled(worker):
    worker.service.status = replace(worker.service.status, state="disconnected")
    worker._command(message())
    assert worker.outputs[-1].outcome == "disconnected"
    worker.config = replace(worker.config, commands_enabled=False)
    worker._command(message(replace(message().payload, request_id="other")))
    assert worker.outputs[-1].outcome == "disabled"


def test_bounded_receive_even_when_subscriber_returns_none(worker):
    calls = []
    worker.context = replace(worker.context, subscriber=SimpleNamespace(receive=lambda timeout_ms: calls.append(1)))
    worker.step()
    assert len(calls) <= worker.config.max_messages


def test_cleanup_before_and_after_partial_initialization():
    w = FlightControllerWorker({"backend": "fake", "firmware": "px4", "connection": {}})
    w.cleanup()
    with pytest.raises(ValueError):
        w.initialize(WorkerContext("flight", 50))
    w.cleanup()


def test_cache_pressure_retains_live_request_and_rejects_new_work(worker):
    worker.config = replace(worker.config, dedup_size=1)
    worker._command(message())
    worker.service.events.put(ActionResult("req", "session", "producer", "arm", "accepted", "ACK"))
    worker.step()
    worker._command(message(replace(message().payload, request_id="new")))
    assert worker.outputs[-1].outcome == "busy"
    worker._command(message())
    assert worker.outputs[-1].outcome == "accepted"
    assert len(worker.service.sent) == 1
    worker.context = replace(worker.context, wall_clock=lambda: 111)
    worker._command(message())
    assert worker.outputs[-1].outcome == "expired"
    assert len(worker.service.sent) == 1
