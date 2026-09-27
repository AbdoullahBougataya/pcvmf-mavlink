"""Finite mission/parameter example restricted to the device-free backend."""

import json
import logging
import uuid
from pathlib import Path

from pcvmf.api import Worker

from .messages import ConnectionStatus, number, text
from .transactions import MissionItem, MissionRequest, MissionResult, ParameterRequest, ParameterResult


class TransactionSmokeWorker(Worker):
    @classmethod
    def validate_options(cls, options):
        if set(options) - {"timeout_s", "output"}:
            raise ValueError("unknown transaction smoke option")
        number(options.get("timeout_s", 20), "timeout_s", 1)
        if "output" in options:
            text(options["output"], "output")

    def initialize(self, context):
        if context.publisher is None or context.subscriber is None:
            raise ValueError("transaction smoke worker needs a publisher and subscriber")
        self.context = context
        self.started = context.monotonic()
        self.status = self.request = None
        self.last_sent = float("-inf")
        self.finished = []
        self.items = [MissionItem(6, 16, 473977420, 85455940, 10, current=1)]
        self.operations = [
            "upload",
            "download",
            "clear",
            "download_empty",
            "get_int",
            "set_int",
            "get_int_again",
            "get_float",
            "set_float",
            "get_float_again",
        ]

    def step(self):
        now = self.context.monotonic()
        if now - self.started > self.options.get("timeout_s", 20):
            raise TimeoutError("transaction example exceeded its deadline")
        for _ in range(100):
            message = self.context.subscriber.receive(0)
            if message is None or message.source != "flight":
                continue
            payload = message.payload
            if isinstance(payload, ConnectionStatus):
                if payload.backend != "fake":
                    raise ValueError("TransactionSmokeWorker only supports the fake backend")
                if self.status and (payload.session_id != self.status.session_id or payload.state != "connected"):
                    raise RuntimeError("connection changed during transaction example")
                self.status = payload
            elif isinstance(payload, (MissionResult, ParameterResult)) and self.request is not None:
                if (payload.requester, payload.request_id, payload.session_id) != (
                    self.context.name,
                    self.request.request_id,
                    self.request.session_id,
                ):
                    continue
                if payload.outcome == "processing":
                    continue
                if payload.outcome != "accepted":
                    raise RuntimeError(f"{payload.operation}: {payload.outcome}: {payload.detail}")
                operation = self.operations[len(self.finished)]
                if isinstance(payload, MissionResult) and payload.operation == "download":
                    if payload.items != (self.items if operation == "download" else []):
                        raise ValueError("downloaded mission does not match")
                if isinstance(payload, ParameterResult):
                    expected = {
                        "get_int": 42,
                        "set_int": 43,
                        "get_int_again": 43,
                        "get_float": 1.25,
                        "set_float": 3.5,
                        "get_float_again": 3.5,
                    }[operation]
                    if payload.value != expected:
                        raise ValueError("parameter value does not match")
                self.finished.append(operation)
                self.request = None
                self.last_sent = float("-inf")
        if len(self.finished) == len(self.operations):
            if "output" in self.options:
                Path(self.options["output"]).write_text(json.dumps({"transactions_observed": self.finished}))
            logging.getLogger(__name__).info("Observed mission and parameter round trips: %s", self.finished)
            return False
        if self.status is None or self.status.state != "connected":
            return
        if self.request is None:
            operation = self.operations[len(self.finished)]
            identity = (str(uuid.uuid4()), self.status.session_id)
            expires = self.context.wall_clock() + 10
            if "int" in operation or "float" in operation:
                kind = "int" if "int" in operation else "float"
                setting = operation.startswith("set_")
                self.request = ParameterRequest(
                    *identity,
                    "set" if setting else "get",
                    expires,
                    f"TEST_{kind.upper()}",
                    kind,
                    (43 if kind == "int" else 3.5) if setting else None,
                )
            else:
                self.request = MissionRequest(
                    *identity, operation.split("_")[0], expires, self.items if operation == "upload" else []
                )
        if now - self.last_sent >= 0.25:
            self.context.publisher.publish("flight/command", self.request)
            self.last_sent = now

    def cleanup(self):
        pass
