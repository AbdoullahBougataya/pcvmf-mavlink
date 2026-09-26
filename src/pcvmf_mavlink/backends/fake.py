"""Deterministic, device-free adapter used by examples and integration tests."""

import asyncio
import time

from .base import Backend


class FakeBackend(Backend):
    async def connect(self):
        self.armed = False
        self.altitude = 0.0
        self.last_heartbeat = time.monotonic()
        await self.poll()

    async def poll(self):
        await asyncio.sleep(0.01)
        self.last_heartbeat = time.monotonic()
        self.update("state", {"armed": self.armed, "flight_mode": "HOLD"})
        self.update("landed", {"landed_state": "ON_GROUND" if self.altitude == 0 else "IN_AIR"})
        self.update(
            "position",
            {
                "latitude_deg": 47.397742,
                "longitude_deg": 8.545594,
                "altitude_msl_m": 488 + self.altitude,
                "altitude_relative_home_m": self.altitude,
            },
        )
        self.update("battery", {"voltage_v": 16.2, "remaining_fraction": 0.9})

    async def execute(self, request):
        await asyncio.sleep(0.02)
        if request.action == "arm":
            self.armed = True
        elif request.action == "disarm":
            self.armed = False
        elif request.action == "takeoff":
            self.require_armed()
            self.altitude = request.takeoff_altitude_m
        else:
            self.altitude = 0.0
        return "accepted", "fake autopilot accepted the action"

    async def close(self):
        pass
