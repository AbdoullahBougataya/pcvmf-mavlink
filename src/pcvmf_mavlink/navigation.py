"""Versioned navigation control, leased setpoints, and navigation telemetry."""

import math
from dataclasses import dataclass, field

from .messages import OUTCOMES, TelemetrySample, _Codec, choice, number, text

OPERATIONS = ("stream_start", "stream_stop", "mission_start", "mission_pause", "mission_set_current")
STREAM_STATES = ("idle", "priming", "active", "stopping", "stopped", "expired", "lost")


@dataclass(frozen=True)
class Setpoint:
    session_id: str
    stream_id: str
    sequence: int
    expires_at: float
    yaw_rad: float
    position_ned_m: list[float] | None = None
    velocity_ned_m_s: list[float] | None = None


@dataclass(frozen=True)
class ControlRequest:
    request_id: str
    session_id: str
    operation: str
    expires_at: float
    stream_id: str | None = None
    initial_setpoint: Setpoint | None = None
    mission_index: int | None = None


@dataclass(frozen=True)
class ControlResult:
    request_id: str
    session_id: str
    requester: str
    operation: str
    outcome: str
    detail: str


@dataclass(frozen=True)
class StreamStatus:
    session_id: str
    state: str
    stream_id: str | None
    requester: str | None
    last_sequence: int | None
    reason: str


@dataclass(frozen=True)
class NavigationTelemetry:
    session_id: str
    samples: dict[str, TelemetrySample] = field(default_factory=dict)


class SetpointCodec(_Codec):
    message_type = "pcvmf_mavlink.setpoint"
    payload_type = Setpoint

    def decode(self, raw):
        d = self.checked(raw)
        for key in ("session_id", "stream_id"):
            text(d[key], key)
        number(d["sequence"], "sequence", 0, 2**53 - 1, True)
        number(d["expires_at"], "expires_at", 0)
        number(d["yaw_rad"], "yaw_rad", -math.pi, math.pi)
        if (d["position_ned_m"] is None) == (d["velocity_ned_m_s"] is None):
            raise ValueError("provide exactly one NED position or velocity vector")
        for key, limit in (("position_ned_m", 1000000), ("velocity_ned_m_s", 1000)):
            vector = d[key]
            if vector is not None:
                if not isinstance(vector, list) or len(vector) != 3:
                    raise ValueError(f"{key} must contain north, east, down")
                for value in vector:
                    number(value, key, -limit, limit)
        return Setpoint(**d)


class ControlRequestCodec(_Codec):
    message_type = "pcvmf_mavlink.control_request"
    payload_type = ControlRequest

    def decode(self, raw):
        d = self.checked(raw)
        for key in ("request_id", "session_id"):
            text(d[key], key)
        number(d["expires_at"], "expires_at", 0)
        choice(d["operation"], OPERATIONS, "operation")
        initial = None
        if d["operation"].startswith("stream_"):
            text(d["stream_id"], "stream_id")
        elif d["stream_id"] is not None:
            raise ValueError("only stream controls accept stream_id")
        if d["operation"] == "stream_start":
            initial = SetpointCodec().decode(d["initial_setpoint"])
            if (initial.session_id, initial.stream_id) != (d["session_id"], d["stream_id"]):
                raise ValueError("initial setpoint must belong to this session and stream")
        elif d["initial_setpoint"] is not None:
            raise ValueError("only stream_start accepts an initial setpoint")
        if d["operation"] == "mission_set_current":
            number(d["mission_index"], "mission_index", 0, 65534, True)
        elif d["mission_index"] is not None:
            raise ValueError("only mission_set_current accepts mission_index")
        return ControlRequest(**{**d, "initial_setpoint": initial})


class ControlResultCodec(_Codec):
    message_type = "pcvmf_mavlink.control_result"
    payload_type = ControlResult

    def decode(self, raw):
        d = self.checked(raw)
        for key in ("request_id", "session_id", "requester", "detail"):
            text(d[key], key)
        choice(d["operation"], OPERATIONS, "operation")
        choice(d["outcome"], OUTCOMES, "outcome")
        return ControlResult(**d)


class StreamStatusCodec(_Codec):
    message_type = "pcvmf_mavlink.stream_status"
    payload_type = StreamStatus

    def decode(self, raw):
        d = self.checked(raw)
        text(d["session_id"], "session_id")
        text(d["reason"], "reason")
        choice(d["state"], STREAM_STATES, "state")
        for key in ("stream_id", "requester"):
            if d[key] is not None:
                text(d[key], key)
        if d["last_sequence"] is not None:
            number(d["last_sequence"], "last_sequence", 0, 2**53 - 1, True)
        if d["state"] != "idle" and any(d[k] is None for k in ("stream_id", "requester", "last_sequence")):
            raise ValueError("non-idle stream status needs owner, ID and sequence")
        return StreamStatus(**d)


NAVIGATION_FIELDS = {
    "local": {"north_m", "east_m", "down_m", "north_m_s", "east_m_s", "down_m_s"},
    "mission": {"current_index", "total_items", "mission_state"},
    "mission_reached": {"reached_index"},
}


class NavigationTelemetryCodec(_Codec):
    message_type = "pcvmf_mavlink.navigation_telemetry"
    payload_type = NavigationTelemetry

    def decode(self, raw):
        d = self.checked(raw)
        text(d["session_id"], "session_id")
        if not isinstance(d["samples"], dict) or set(d["samples"]) - set(NAVIGATION_FIELDS):
            raise ValueError("invalid navigation groups")
        samples = {}
        for group, s in d["samples"].items():
            if not isinstance(s, dict) or set(s) != {"received_at", "age_s", "stale", "values"}:
                raise ValueError("invalid navigation sample")
            number(s["received_at"], "received_at", 0)
            number(s["age_s"], "age_s", 0)
            if type(s["stale"]) is not bool:
                raise ValueError("stale must be boolean")
            if not isinstance(s["values"], dict) or not s["values"] or set(s["values"]) - NAVIGATION_FIELDS[group]:
                raise ValueError("invalid navigation values")
            for key, value in s["values"].items():
                if value is not None:
                    if group == "local":
                        number(value, key)
                    else:
                        number(value, key, 0, 255 if key == "mission_state" else 65535, True)
            samples[group] = TelemetrySample(**s)
        return NavigationTelemetry(d["session_id"], samples)


def validate_setpoint_limits(point, options):
    if point.velocity_ned_m_s is not None:
        if math.hypot(*point.velocity_ned_m_s) > options.max_setpoint_speed_m_s:
            raise ValueError("setpoint exceeds max_setpoint_speed_m_s")
    elif math.hypot(*point.position_ned_m) > options.max_setpoint_distance_m:
        raise ValueError("setpoint exceeds max_setpoint_distance_m from the estimator origin")


def setpoint_fields(point, options):
    position = point.position_ned_m or [0, 0, 0]
    velocity = point.velocity_ned_m_s or [0, 0, 0]
    return dict(
        time_boot_ms=0,
        target_system=options.target_system,
        target_component=options.target_component,
        coordinate_frame=1,
        type_mask=2552 if point.position_ned_m is not None else 2503,
        x=position[0],
        y=position[1],
        z=position[2],
        vx=velocity[0],
        vy=velocity[1],
        vz=velocity[2],
        afx=0,
        afy=0,
        afz=0,
        yaw=point.yaw_rad,
        yaw_rate=0,
    )
