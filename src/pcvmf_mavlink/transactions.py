"""Mission/parameter payloads and shared transaction policy (schema version 1)."""

import math
import struct
from dataclasses import dataclass, field, fields

from .messages import OUTCOMES, ActionRequest, ActionRequestCodec, ActionResult, _Codec, choice, number, text

MAX_MISSION_ITEMS = 10000


def float32(value):
    number(value, "float32", -3.4028234663852886e38, 3.4028234663852886e38)
    return struct.unpack("<f", struct.pack("<f", value))[0]


@dataclass(frozen=True)
class MissionItem:
    """Raw MISSION_ITEM_INT fields; list position supplies the sequence number.

    Global x/y use degrees * 1e7, z uses frame-specific metres. None in a
    floating-point parameter represents the MAVLink NaN/unspecified value.
    """

    frame: int
    command: int
    x: int
    y: int
    z: float | None
    param1: float | None = 0.0
    param2: float | None = 0.0
    param3: float | None = 0.0
    param4: float | None = None
    current: int = 0
    autocontinue: int = 1


def decode_items(raw):
    if not isinstance(raw, list) or len(raw) > MAX_MISSION_ITEMS:
        raise ValueError(f"items must be a list of at most {MAX_MISSION_ITEMS} mission items")
    items = []
    for item in raw:
        if not isinstance(item, dict) or set(item) != {f.name for f in fields(MissionItem)}:
            raise ValueError("incorrect MissionItem fields")
        for key, low, high in (("frame", 0, 255), ("command", 0, 65535), ("current", 0, 1), ("autocontinue", 0, 1)):
            number(item[key], key, low, high, True)
        for key in ("x", "y"):
            number(item[key], key, -(2**31), 2**31 - 1, True)
        for key in ("param1", "param2", "param3", "param4", "z"):
            if item[key] is not None:
                float32(item[key])
        items.append(MissionItem(**item))
    return items


def item_from_wire(raw):
    result = {f.name: raw[f.name] for f in fields(MissionItem)}
    for key in ("param1", "param2", "param3", "param4", "z"):
        if math.isnan(result[key]):
            result[key] = None
    return decode_items([result])[0]


def item_to_wire(item):
    return {f.name: float("nan") if getattr(item, f.name) is None else getattr(item, f.name) for f in fields(item)}


@dataclass(frozen=True)
class MissionRequest:
    request_id: str
    session_id: str
    operation: str
    expires_at: float
    items: list[MissionItem] = field(default_factory=list)


@dataclass(frozen=True)
class MissionResult:
    request_id: str
    session_id: str
    requester: str
    operation: str
    outcome: str
    detail: str
    items: list[MissionItem] | None = None


@dataclass(frozen=True)
class ParameterRequest:
    request_id: str
    session_id: str
    operation: str
    expires_at: float
    name: str
    parameter_type: str
    value: int | float | None = None


@dataclass(frozen=True)
class ParameterResult:
    request_id: str
    session_id: str
    requester: str
    operation: str
    outcome: str
    detail: str
    name: str
    parameter_type: str
    value: int | float | None = None


def _identity(d, result=False):
    for key in ("request_id", "session_id"):
        text(d[key], key)
    if result:
        for key in ("requester", "detail"):
            text(d[key], key)
        choice(d["outcome"], OUTCOMES, "outcome")
    else:
        number(d["expires_at"], "expires_at", 0)


def _parameter(d):
    name = d["name"]
    if not isinstance(name, str) or not 1 <= len(name) <= 16 or any(not 33 <= ord(c) <= 126 for c in name):
        raise ValueError("parameter name must contain 1–16 printable ASCII characters without spaces")
    choice(d["parameter_type"], ("int", "float"), "parameter_type")
    if d["value"] is not None:
        if d["parameter_type"] == "int":
            number(d["value"], "value", -(2**31), 2**31 - 1, True)
        else:
            float32(d["value"])


class MissionRequestCodec(_Codec):
    message_type = "pcvmf_mavlink.mission_request"
    payload_type = MissionRequest

    def decode(self, raw):
        d = self.checked(raw)
        _identity(d)
        choice(d["operation"], ("upload", "download", "clear"), "operation")
        items = decode_items(d["items"])
        if d["operation"] == "upload":
            if not items:
                raise ValueError("upload needs at least one item; use clear to remove a mission")
            if sum(item.current for item in items) != 1:
                raise ValueError("exactly one uploaded mission item must be current")
        elif items:
            raise ValueError("only upload accepts items")
        return MissionRequest(**{**d, "items": items})


class MissionResultCodec(_Codec):
    message_type = "pcvmf_mavlink.mission_result"
    payload_type = MissionResult

    def decode(self, raw):
        d = self.checked(raw)
        _identity(d, True)
        choice(d["operation"], ("upload", "download", "clear"), "operation")
        if d["operation"] == "download" and d["outcome"] == "accepted":
            return MissionResult(**{**d, "items": decode_items(d["items"])})
        if d["items"] is not None:
            raise ValueError("items are only returned for an accepted download")
        return MissionResult(**d)


class ParameterRequestCodec(_Codec):
    message_type = "pcvmf_mavlink.parameter_request"
    payload_type = ParameterRequest

    def decode(self, raw):
        d = self.checked(raw)
        _identity(d)
        choice(d["operation"], ("get", "set"), "operation")
        _parameter(d)
        if (d["value"] is not None) != (d["operation"] == "set"):
            raise ValueError("set requires value; get does not accept value")
        return ParameterRequest(**d)


class ParameterResultCodec(_Codec):
    message_type = "pcvmf_mavlink.parameter_result"
    payload_type = ParameterResult

    def decode(self, raw):
        d = self.checked(raw)
        _identity(d, True)
        choice(d["operation"], ("get", "set"), "operation")
        _parameter(d)
        if (d["value"] is not None) != (d["outcome"] == "accepted"):
            raise ValueError("only accepted parameter results require a value")
        return ParameterResult(**d)


REQUEST_CODECS = {
    ActionRequest: ActionRequestCodec,
    MissionRequest: MissionRequestCodec,
    ParameterRequest: ParameterRequestCodec,
}


def result_for(request, requester, outcome, detail, data=None):
    identity = (request.request_id, request.session_id, requester)
    tail = (outcome, detail[:512] or outcome)
    if type(request) is ActionRequest:
        return ActionResult(*identity, request.action, *tail)
    if type(request) is MissionRequest:
        return MissionResult(*identity, request.operation, *tail, data)
    return ParameterResult(*identity, request.operation, *tail, request.name, request.parameter_type, data)


def disabled_reason(request, options):
    if type(request) is ActionRequest:
        return None if options.commands_enabled else "commands_enabled is false"
    if type(request) is MissionRequest:
        return None if options.missions_enabled else "missions_enabled is false"
    if not options.parameters_enabled:
        return "parameters_enabled is false"
    if request.operation == "set" and not options.parameter_writes_enabled:
        return "parameter_writes_enabled is false"
    return None
