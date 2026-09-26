"""Versioned, backend-independent public payloads and strict JSON codecs."""

import math
from dataclasses import asdict, dataclass, field, fields
from typing import Any

from pcvmf.api import MessageCodec

ACTIONS = ("arm", "disarm", "takeoff", "land", "return_to_launch")
OUTCOMES = (
    "processing",
    "accepted",
    "rejected",
    "unsupported",
    "busy",
    "expired",
    "disconnected",
    "session_mismatch",
    "disabled",
    "outcome_unknown",
    "request_conflict",
)
GROUP_FIELDS = {
    "state": {"armed", "flight_mode"},
    "landed": {"landed_state"},
    "position": {"latitude_deg", "longitude_deg", "altitude_msl_m", "altitude_relative_home_m"},
    "velocity": {"north_m_s", "east_m_s", "down_m_s"},
    "attitude": {"roll_rad", "pitch_rad", "yaw_rad"},
    "battery": {"voltage_v", "remaining_fraction"},
    "gps": {"fix_type", "satellites_visible"},
    "health": {"armable", "global_position_ok", "local_position_ok", "home_position_ok", "sensors_ok"},
}
BOOL_FIELDS = {"armed", "armable", "global_position_ok", "local_position_ok", "home_position_ok", "sensors_ok"}
TEXT_FIELDS = {"flight_mode", "landed_state"}
INT_FIELDS = {"fix_type", "satellites_visible"}


def text(value, name):
    if not isinstance(value, str) or not value or len(value) > 512 or "\x00" in value:
        raise ValueError(f"{name} must be a nonempty string (at most 512 characters)")


def number(value, name, minimum=None, maximum=None, integer=False):
    if isinstance(value, bool) or not isinstance(value, int if integer else (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite {'integer' if integer else 'number'}")
    if (minimum is not None and value < minimum) or (maximum is not None and value > maximum):
        raise ValueError(f"{name} is outside its allowed range")


def choice(value, choices, name):
    if not isinstance(value, str) or value not in choices:
        raise ValueError(f"{name} must be one of {choices}")


@dataclass(frozen=True)
class TelemetrySample:
    """Receipt time and local silence age, not a synchronized vehicle measurement time."""

    received_at: float
    age_s: float
    stale: bool
    values: dict[str, Any]


@dataclass(frozen=True)
class VehicleTelemetry:
    session_id: str
    system_id: int
    component_id: int
    samples: dict[str, TelemetrySample] = field(default_factory=dict)


@dataclass(frozen=True)
class ConnectionStatus:
    state: str
    backend: str
    firmware: str
    system_id: int
    component_id: int
    session_id: str
    supported_actions: list[str]
    commands_enabled: bool
    reason: str


@dataclass(frozen=True)
class ActionRequest:
    request_id: str
    session_id: str
    action: str
    expires_at: float
    takeoff_altitude_m: float | None = None


@dataclass(frozen=True)
class ActionResult:
    request_id: str
    session_id: str
    requester: str
    action: str
    outcome: str
    detail: str


class _Codec(MessageCodec):
    schema_version = 1

    def encode(self, payload):
        if type(payload) is not self.payload_type:
            raise ValueError(f"expected {self.payload_type.__name__}")
        raw = asdict(payload)
        self.decode(raw)
        return raw

    def checked(self, raw):
        if not isinstance(raw, dict) or set(raw) != {f.name for f in fields(self.payload_type)}:
            raise ValueError(f"incorrect fields for {self.payload_type.__name__}")
        return raw


class ActionRequestCodec(_Codec):
    message_type = "pcvmf_mavlink.action_request"
    payload_type = ActionRequest

    def decode(self, raw):
        d = self.checked(raw)
        for k in ("request_id", "session_id"):
            text(d[k], k)
        choice(d["action"], ACTIONS, "action")
        number(d["expires_at"], "expires_at", 0)
        if d["action"] == "takeoff":
            number(d["takeoff_altitude_m"], "takeoff_altitude_m", 0.1, 1000)
        elif d["takeoff_altitude_m"] is not None:
            raise ValueError("takeoff_altitude_m is only valid for takeoff")
        return ActionRequest(**d)


class ActionResultCodec(_Codec):
    message_type = "pcvmf_mavlink.action_result"
    payload_type = ActionResult

    def decode(self, raw):
        d = self.checked(raw)
        for k in ("request_id", "session_id", "requester", "detail"):
            text(d[k], k)
        choice(d["action"], ACTIONS, "action")
        choice(d["outcome"], OUTCOMES, "outcome")
        return ActionResult(**d)


class ConnectionStatusCodec(_Codec):
    message_type = "pcvmf_mavlink.connection_status"
    payload_type = ConnectionStatus

    def decode(self, raw):
        d = self.checked(raw)
        choice(d["state"], ("connected", "disconnected"), "state")
        choice(d["backend"], ("pymavlink", "mavsdk", "fake"), "backend")
        choice(d["firmware"], ("px4", "arducopter"), "firmware")
        for k in ("system_id", "component_id"):
            number(d[k], k, 1, 255, integer=True)
        for k in ("session_id", "reason"):
            text(d[k], k)
        if type(d["commands_enabled"]) is not bool:
            raise ValueError("commands_enabled must be boolean")
        if not isinstance(d["supported_actions"], list):
            raise ValueError("supported_actions must be a list")
        for action in d["supported_actions"]:
            choice(action, ACTIONS, "supported_actions")
        if len(set(d["supported_actions"])) != len(d["supported_actions"]):
            raise ValueError("duplicate supported action")
        return ConnectionStatus(**d)


class VehicleTelemetryCodec(_Codec):
    message_type = "pcvmf_mavlink.vehicle_telemetry"
    payload_type = VehicleTelemetry

    def decode(self, raw):
        d = self.checked(raw)
        text(d["session_id"], "session_id")
        for k in ("system_id", "component_id"):
            number(d[k], k, 1, 255, integer=True)
        if not isinstance(d["samples"], dict) or set(d["samples"]) - set(GROUP_FIELDS):
            raise ValueError("invalid telemetry sample groups")
        samples = {}
        for group, sample in d["samples"].items():
            if not isinstance(sample, dict) or set(sample) != {"received_at", "age_s", "stale", "values"}:
                raise ValueError("invalid sample fields")
            number(sample["received_at"], "received_at", 0)
            number(sample["age_s"], "age_s", 0)
            if type(sample["stale"]) is not bool:
                raise ValueError("stale must be boolean")
            values = sample["values"]
            if not isinstance(values, dict) or not values or set(values) - GROUP_FIELDS[group]:
                raise ValueError(f"invalid fields in {group}")
            for k, v in values.items():
                if v is None:
                    continue
                if k in BOOL_FIELDS:
                    if type(v) is not bool:
                        raise ValueError(f"{k} must be boolean")
                elif k in TEXT_FIELDS:
                    text(v, k)
                else:
                    limits = {
                        "latitude_deg": (-90, 90),
                        "longitude_deg": (-180, 180),
                        "remaining_fraction": (0, 1),
                        "voltage_v": (0, None),
                        "fix_type": (0, 8),
                        "satellites_visible": (0, 254),
                    }
                    low, high = limits.get(k, (None, None))
                    number(v, k, low, high, k in INT_FIELDS)
            samples[group] = TelemetrySample(**sample)
        return VehicleTelemetry(d["session_id"], d["system_id"], d["component_id"], samples)
