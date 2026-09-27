"""Adapter contract; every method runs in the owning I/O thread's event loop."""

import math
import time
from abc import ABC, abstractmethod

from ..messages import TelemetrySample


class LinkLost(ConnectionError):
    pass


class Backend(ABC):
    def __init__(self, options):
        self.options = options
        self.last_heartbeat = 0.0
        self.samples = {}
        self.sample_times = {}

    def update(self, group, values):
        # MAVLink uses NaN and various sentinel values for unavailable data.
        values = {k: None if isinstance(v, float) and not math.isfinite(v) else v for k, v in values.items()}
        self.samples[group] = TelemetrySample(time.time(), 0, False, values)
        self.sample_times[group] = time.monotonic()

    @abstractmethod
    async def connect(self): ...

    @abstractmethod
    async def poll(self): ...

    @abstractmethod
    async def execute(self, request):
        """Return (normalized outcome, detail); acceptance does not mean completion."""

    @abstractmethod
    async def close(self): ...

    async def transfer_mission(self, request):
        return "unsupported", "backend does not implement mission transfer", None

    async def parameter(self, request):
        return "unsupported", "backend does not implement parameters", None

    def require_armed(self):
        sample = self.samples.get("state")
        if (
            sample is None
            or sample.values.get("armed") is not True
            or time.monotonic() - self.sample_times["state"] > self.options.telemetry_stale_s
        ):
            raise ValueError("takeoff requires fresh telemetry confirming explicit arming")


def heartbeat_values(fields, options):
    expected = 12 if options.firmware == "px4" else 3
    if fields["autopilot"] != expected:
        raise ValueError(f"expected {options.firmware}; heartbeat autopilot={fields['autopilot']}")
    if fields["type"] not in (2, 3, 4, 13, 14, 15, 29, 35):
        raise ValueError("the connected vehicle is not a supported multicopter")
    custom = fields["custom_mode"]
    if options.firmware == "arducopter":
        mode = {
            0: "STABILIZE",
            1: "ACRO",
            2: "ALT_HOLD",
            3: "AUTO",
            4: "GUIDED",
            5: "LOITER",
            6: "RTL",
            9: "LAND",
            16: "POSHOLD",
            20: "GUIDED_NOGPS",
        }.get(custom, f"CUSTOM_{custom}")
    else:
        main, sub = (custom >> 16) & 255, (custom >> 24) & 255
        mode = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED"}.get(main)
        if main == 4:
            mode = {1: "READY", 2: "TAKEOFF", 3: "HOLD", 4: "MISSION", 5: "RTL", 6: "LAND"}.get(sub)
        mode = mode or f"CUSTOM_{custom}"
    return {"armed": bool(fields["base_mode"] & 128), "flight_mode": mode}
