"""Deterministic, device-free adapter used by examples and integration tests."""

import asyncio
import time

from ..transactions import float32
from .base import Backend


class FakeBackend(Backend):
    async def connect(self):
        self.armed = False
        self.altitude = 0.0
        self.mode = "HOLD"
        self.local = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.mission_index = 0
        self.mission = []
        self.parameters = {"TEST_INT": ("int", 42), "TEST_FLOAT": ("float", 1.25)}
        self.last_heartbeat = time.monotonic()
        await self.poll()

    async def poll(self):
        await asyncio.sleep(0.01)
        self.last_heartbeat = time.monotonic()
        self.update("state", {"armed": self.armed, "flight_mode": self.mode})
        self.update_navigation(
            "local",
            dict(
                north_m=self.local[0],
                east_m=self.local[1],
                down_m=-self.altitude,
                north_m_s=self.velocity[0],
                east_m_s=self.velocity[1],
                down_m_s=self.velocity[2],
            ),
        )
        self.update_navigation(
            "mission",
            dict(
                current_index=self.mission_index if self.mission else None,
                total_items=len(self.mission),
                mission_state=3 if self.mode in ("AUTO", "MISSION") else 4,
            ),
        )
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

    async def transfer_mission(self, request):
        await asyncio.sleep(0.02)
        if len(request.items) > self.options.max_mission_items:
            raise ValueError("mission exceeds max_mission_items")
        if request.operation == "upload":
            self.mission = list(request.items)
        elif request.operation == "clear":
            self.mission.clear()
        return (
            "accepted",
            "fake mission transaction completed",
            list(self.mission) if request.operation == "download" else None,
        )

    async def parameter(self, request):
        await asyncio.sleep(0.02)
        if request.name not in self.parameters:
            raise ValueError("unknown fake parameter")
        kind, value = self.parameters[request.name]
        if kind != request.parameter_type:
            raise ValueError("parameter type does not match")
        if request.operation == "set":
            value = float32(request.value) if kind == "float" else request.value
            self.parameters[request.name] = (kind, value)
        return "accepted", "fake parameter value observed", value

    async def send_setpoint(self, point):
        if self.mode not in ("GUIDED", "OFFBOARD"):
            await asyncio.sleep(0)
            return
        if point.position_ned_m is not None:
            self.local = list(point.position_ned_m)
            self.altitude = -self.local[2]
            self.velocity = [0, 0, 0]
        else:
            self.velocity = list(point.velocity_ned_m_s)
        await asyncio.sleep(0)

    async def stream_mode(self, enabled):
        self.mode = ("OFFBOARD" if self.options.firmware == "px4" else "GUIDED") if enabled else "HOLD"
        if not enabled:
            self.velocity = [0, 0, 0]
        return "accepted", "fake stream mode changed"

    async def control_mission(self, request):
        if request.operation == "mission_start":
            self.require_armed("mission start")
            if not self.mission:
                raise ValueError("no mission stored")
            self.mode = "MISSION" if self.options.firmware == "px4" else "AUTO"
        elif request.operation == "mission_pause":
            self.mode = "HOLD"
        else:
            if request.mission_index >= len(self.mission):
                raise ValueError("mission index is outside the stored mission")
            self.mission_index = request.mission_index
        return "accepted", "fake mission control accepted"
