"""Finite, observable smoke worker. Action sequences require explicit configuration."""

import json
import logging
import uuid
from pathlib import Path

from pcvmf.api import Worker

from .messages import ACTIONS, ActionRequest, ActionResult, ConnectionStatus, VehicleTelemetry, choice, number, text

logger = logging.getLogger(__name__)


class SmokeWorker(Worker):
    @classmethod
    def validate_options(cls, options):
        if set(options) - {"actions", "timeout_s", "settle_s", "takeoff_altitude_m", "output", "source"}:
            raise ValueError("unknown smoke worker option")
        actions = options.get("actions", [])
        if not isinstance(actions, list):
            raise ValueError("actions must be a list")
        for action in actions:
            choice(action, ACTIONS, "action")
        number(options.get("timeout_s", 20), "timeout_s", 1)
        number(options.get("settle_s", 0), "settle_s", 0)
        number(options.get("takeoff_altitude_m", 2), "takeoff_altitude_m", 0.1, 1000)
        for key in ("output", "source"):
            if key in options:
                text(options[key], key)

    def initialize(self, context):
        self.context = context
        self.started = context.monotonic()
        self.status = self.telemetry = self.request = None
        self.accepted = False
        self.finished = []
        self.actions = list(self.options.get("actions", []))
        self.last_sent = float("-inf")
        if context.subscriber is None or (self.actions and context.publisher is None):
            raise ValueError("smoke worker needs subscriptions and a publisher when actions are enabled")

    def _observed(self, action):
        samples = self.telemetry.samples
        state = samples.get("state")
        landed = samples.get("landed")
        position = samples.get("position")
        if action in ("arm", "disarm"):
            return state and not state.stale and state.values.get("armed") is (action == "arm")
        if action == "takeoff":
            return (
                position
                and not position.stale
                and position.values.get("altitude_relative_home_m") is not None
                and position.values["altitude_relative_home_m"] >= self.options.get("takeoff_altitude_m", 2) * 0.9
            )
        return landed and not landed.stale and landed.values.get("landed_state") == "ON_GROUND"

    def step(self):
        now = self.context.monotonic()
        if now - self.started > self.options.get("timeout_s", 20):
            raise TimeoutError("smoke scenario did not complete before its deadline")
        for _ in range(100):
            message = self.context.subscriber.receive(0)
            if message is None or message.source != self.options.get("source", "flight"):
                continue
            payload = message.payload
            if isinstance(payload, ConnectionStatus):
                if self.request is not None and (
                    payload.state != "connected" or payload.session_id != self.request.session_id
                ):
                    raise RuntimeError("connection changed during smoke action; no automatic replay")
                self.status = payload
            elif isinstance(payload, VehicleTelemetry):
                self.telemetry = payload
            elif (
                isinstance(payload, ActionResult)
                and self.request is not None
                and payload.requester == self.context.name
                and payload.request_id == self.request.request_id
                and payload.session_id == self.request.session_id
            ):
                if payload.outcome == "accepted":
                    self.accepted = True
                elif payload.outcome != "processing":
                    raise RuntimeError(f"{payload.action}: {payload.outcome}: {payload.detail}")
        if (
            self.status is None
            or self.status.state != "connected"
            or self.telemetry is None
            or self.telemetry.session_id != self.status.session_id
            or not self.telemetry.samples
        ):
            return
        if now - self.started < self.options.get("settle_s", 0):
            return
        if self.request is not None and self.accepted and self._observed(self.request.action):
            self.finished.append(self.request.action)
            self.request = None
            self.accepted = False
            self.actions.pop(0)
        if not self.actions:
            summary = {
                "received": "VehicleTelemetry",
                "backend": self.status.backend,
                "firmware": self.status.firmware,
                "actions_observed": self.finished,
            }
            if "output" in self.options:
                Path(self.options["output"]).write_text(json.dumps(summary), encoding="utf-8")
            logger.info("Flight smoke completed: %s", summary)
            return False
        if self.request is None:
            action = self.actions[0]
            self.request = ActionRequest(
                str(uuid.uuid4()),
                self.status.session_id,
                action,
                self.context.wall_clock() + 10,
                self.options.get("takeoff_altitude_m", 2) if action == "takeoff" else None,
            )
            self.last_sent = float("-inf")
        if not self.accepted and now - self.last_sent >= 0.5:
            self.context.publisher.publish("flight/command", self.request)
            self.last_sent = now

    def cleanup(self):
        pass
