"""Finite, fake-only PCVMF navigation example with observed state transitions."""

import json
import uuid
from pathlib import Path

from pcvmf.api import Worker

from .messages import ActionRequest, ActionResult, ConnectionStatus, VehicleTelemetry, number, text
from .navigation import ControlRequest, ControlResult, NavigationTelemetry, Setpoint, StreamStatus
from .transactions import MissionItem, MissionRequest, MissionResult


class NavigationSmokeWorker(Worker):
    @classmethod
    def validate_options(cls, options):
        if set(options) - {"timeout_s", "output"}:
            raise ValueError("unknown navigation smoke option")
        number(options.get("timeout_s", 30), "timeout_s", 1)
        if "output" in options:
            text(options["output"], "output")

    def initialize(self, context):
        if context.publisher is None or context.subscriber is None:
            raise ValueError("navigation example needs publications and subscriptions")
        self.context = context
        self.started = context.monotonic()
        self.last_sent = self.last_point = float("-inf")
        self.status = self.telemetry = self.navigation = self.stream_status = self.request = None
        self.stream_id = str(uuid.uuid4())
        self.sequence = 0
        self.streaming = self.accepted = False
        self.finished = []
        self.stages = [
            "upload",
            "arm",
            "takeoff",
            "stream_start",
            "stream_stop",
            "mission_start",
            "mission_pause",
            "mission_set_current",
            "land",
            "disarm",
        ]

    def _sample(self, payload, group):
        if payload is None or payload.session_id != self.status.session_id:
            return {}
        sample = payload.samples.get(group)
        return sample.values if sample is not None and not sample.stale else {}

    def _observed(self, stage):
        state = self._sample(self.telemetry, "state")
        if stage == "arm":
            return state.get("armed") is True
        if stage == "disarm":
            return state.get("armed") is False
        if stage == "takeoff":
            return self._sample(self.telemetry, "position").get("altitude_relative_home_m", 0) >= 1.9
        if stage == "land":
            return self._sample(self.telemetry, "landed").get("landed_state") == "ON_GROUND"
        if stage == "stream_start":
            return (
                self.stream_status is not None
                and self.stream_status.state == "active"
                and self._sample(self.navigation, "local").get("north_m") == 1
            )
        if stage in ("stream_stop", "mission_pause"):
            return state.get("flight_mode") == "HOLD"
        if stage == "mission_start":
            return state.get("flight_mode") in ("MISSION", "AUTO")
        if stage == "mission_set_current":
            return self._sample(self.navigation, "mission").get("current_index") == 1
        return True

    def _point(self):
        point = Setpoint(
            self.status.session_id,
            self.stream_id,
            self.sequence,
            self.context.wall_clock() + 0.5,
            0,
            position_ned_m=[1, 0, -2],
        )
        self.sequence += 1
        return point

    def step(self):
        now = self.context.monotonic()
        if now - self.started > self.options.get("timeout_s", 30):
            raise TimeoutError("navigation example did not complete")
        for _ in range(100):
            message = self.context.subscriber.receive(0)
            if message is None or message.source != "flight":
                continue
            value = message.payload
            if isinstance(value, ConnectionStatus):
                if value.backend != "fake":
                    raise ValueError("NavigationSmokeWorker only supports the fake backend")
                if self.status and (value.session_id != self.status.session_id or value.state != "connected"):
                    raise RuntimeError("connection changed; navigation example will not replay")
                self.status = value
            elif isinstance(value, VehicleTelemetry):
                self.telemetry = value
            elif isinstance(value, NavigationTelemetry):
                self.navigation = value
            elif isinstance(value, StreamStatus):
                self.stream_status = value
            elif isinstance(value, (ActionResult, MissionResult, ControlResult)) and self.request is not None:
                if (value.requester, value.session_id, value.request_id) != (
                    self.context.name,
                    self.request.session_id,
                    self.request.request_id,
                ):
                    continue
                if value.outcome == "accepted":
                    self.accepted = True
                    if getattr(self.request, "operation", None) == "stream_stop":
                        self.streaming = False
                elif value.outcome != "processing":
                    raise RuntimeError(f"navigation request failed: {value.outcome}: {value.detail}")
        if self.status is None or self.status.state != "connected" or self.navigation is None:
            return
        if self.streaming and now - self.last_point >= 0.04:
            self.context.publisher.publish("flight/setpoint", self._point())
            self.last_point = now
        stage = self.stages[len(self.finished)]
        if self.request is not None and self.accepted and self._observed(stage):
            self.finished.append(stage)
            self.request = None
            self.accepted = False
            self.last_sent = float("-inf")
            if len(self.finished) == len(self.stages):
                if "output" in self.options:
                    Path(self.options["output"]).write_text(json.dumps({"navigation_observed": self.finished}))
                return False
            stage = self.stages[len(self.finished)]
        if self.request is None:
            identity = (str(uuid.uuid4()), self.status.session_id)
            expires = self.context.wall_clock() + 8
            if stage == "upload":
                self.request = MissionRequest(
                    *identity,
                    stage,
                    expires,
                    [MissionItem(6, 16, 473977420, 85455940, 2, current=1), MissionItem(6, 21, 473977420, 85455940, 0)],
                )
            elif stage in ("arm", "takeoff", "land", "disarm"):
                self.request = ActionRequest(*identity, stage, expires, 2 if stage == "takeoff" else None)
            else:
                self.request = ControlRequest(
                    *identity,
                    stage,
                    expires,
                    self.stream_id if stage.startswith("stream_") else None,
                    self._point() if stage == "stream_start" else None,
                    1 if stage == "mission_set_current" else None,
                )
                if stage == "stream_start":
                    self.streaming = True
        if now - self.last_sent >= 0.25:
            self.context.publisher.publish("flight/command", self.request)
            self.last_sent = now

    def cleanup(self):
        pass
