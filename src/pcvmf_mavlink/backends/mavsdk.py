"""MAVSDK-Python 3.10.2 adapter with explicitly owned gRPC channel/server.

Use generated plugin constructors with a channel provider, avoiding System's
destructor-managed server and inaccessible partial-initialization channel.
"""

import asyncio
import json
import logging
import math
import socket
import time
from importlib.resources import files
from types import SimpleNamespace

from .base import Backend, LinkLost, heartbeat_values

logger = logging.getLogger(__name__)


class MavsdkBackend(Backend):
    def __init__(self, options):
        super().__init__(options)
        self.process = self.channel = None
        self.tasks = []

    async def connect(self):
        import grpc
        from mavsdk.action import Action
        from mavsdk.mavlink_direct import MavlinkDirect
        from mavsdk.telemetry import Telemetry

        self.grpc = grpc
        config = self.options.server or {}
        host = config.get("host", "127.0.0.1")
        port = config.get("port")
        if "host" not in config:
            if port is None:
                with socket.socket() as reservation:
                    reservation.bind(("127.0.0.1", 0))
                    port = reservation.getsockname()[1]
            executable = config.get("executable", str(files("mavsdk.bin").joinpath("mavsdk_server")))
            self.process = await asyncio.create_subprocess_exec(
                executable,
                "-p",
                str(port),
                "--sysid",
                str(self.options.source_system),
                "--compid",
                str(self.options.source_component),
                self.options.address(),
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
            self.tasks.append(asyncio.create_task(self._logs()))
        self.channel = grpc.aio.insecure_channel(f"{host}:{port}")
        while True:
            if self.process is not None and self.process.returncode is not None:
                raise LinkLost(f"mavsdk_server exited with {self.process.returncode}")
            try:
                await asyncio.wait_for(self.channel.channel_ready(), 0.2)
                break
            except asyncio.TimeoutError:
                continue
        provider = SimpleNamespace(channel=self.channel)
        self.action = Action(provider)
        self.telemetry_api = Telemetry(provider)
        self.direct = MavlinkDirect(provider)
        self.tasks.append(asyncio.create_task(self._heartbeats()))
        streams = {
            "position": (
                "position",
                lambda x: {
                    "latitude_deg": x.latitude_deg,
                    "longitude_deg": x.longitude_deg,
                    "altitude_msl_m": x.absolute_altitude_m,
                    "altitude_relative_home_m": x.relative_altitude_m,
                },
            ),
            "velocity_ned": (
                "velocity",
                lambda x: {"north_m_s": x.north_m_s, "east_m_s": x.east_m_s, "down_m_s": x.down_m_s},
            ),
            "attitude_euler": (
                "attitude",
                lambda x: {
                    "roll_rad": math.radians(x.roll_deg),
                    "pitch_rad": math.radians(x.pitch_deg),
                    "yaw_rad": math.radians(x.yaw_deg),
                },
            ),
            "battery": (
                "battery",
                lambda x: {
                    "voltage_v": x.voltage_v if x.voltage_v >= 0 else None,
                    "remaining_fraction": x.remaining_percent if 0 <= x.remaining_percent <= 1 else None,
                },
            ),
            "gps_info": (
                "gps",
                lambda x: {
                    "fix_type": x.fix_type.value,
                    "satellites_visible": x.num_satellites if 0 <= x.num_satellites < 255 else None,
                },
            ),
            "landed_state": ("landed", lambda x: {"landed_state": x.name}),
            "health": (
                "health",
                lambda x: {
                    "armable": x.is_armable,
                    "global_position_ok": x.is_global_position_ok,
                    "local_position_ok": x.is_local_position_ok,
                    "home_position_ok": x.is_home_position_ok,
                    "sensors_ok": x.is_gyrometer_calibration_ok
                    and x.is_accelerometer_calibration_ok
                    and x.is_magnetometer_calibration_ok,
                },
            ),
        }
        for stream, (group, convert) in streams.items():
            self.tasks.append(asyncio.create_task(self._consume(stream, group, convert)))
        while not self.last_heartbeat:
            await self.poll()
        self.tasks.append(asyncio.create_task(self._rates()))

    async def _logs(self):
        while line := await self.process.stdout.readline():
            logger.debug("mavsdk_server: %s", line.decode(errors="replace").rstrip())

    async def _heartbeats(self):
        async for message in self.direct.message("HEARTBEAT"):
            fields = json.loads(message.fields_json)
            if fields.get("autopilot") == 8:
                continue  # GCS/camera heartbeat, not the flight controller.
            if (message.system_id, message.component_id) != (self.options.target_system, self.options.target_component):
                raise ValueError("MAVSDK requires a dedicated link to the configured vehicle")
            self.update("state", heartbeat_values(fields, self.options))
            self.last_heartbeat = time.monotonic()
        raise LinkLost("MAVSDK heartbeat stream ended")

    async def _consume(self, stream, group, convert):
        async for value in getattr(self.telemetry_api, stream)():
            self.update(group, convert(value))
        raise LinkLost(f"MAVSDK {stream} stream ended")

    async def _rates(self):
        for name in ("position", "velocity_ned", "attitude_euler", "battery", "gps_info", "landed_state"):
            try:
                await asyncio.wait_for(getattr(self.telemetry_api, f"set_rate_{name}")(self.options.telemetry_hz), 1)
            except asyncio.TimeoutError:
                logger.debug("MAVSDK rate request timed out: %s", name)
            except Exception as exc:
                from mavsdk.telemetry import TelemetryError

                if not isinstance(exc, TelemetryError):
                    raise
                logger.debug("MAVSDK rate request rejected: %s", exc)

    async def poll(self):
        await asyncio.sleep(0.01)
        if self.process is not None and self.process.returncode is not None:
            raise LinkLost(f"mavsdk_server exited with {self.process.returncode}")
        for task in self.tasks:
            if task.done() and not task.cancelled() and task.exception() is not None:
                exc = task.exception()
                if isinstance(exc, self.grpc.aio.AioRpcError):
                    raise LinkLost(f"MAVSDK stream connection failed: {exc.code()}") from exc
                raise exc

    async def execute(self, request):
        from mavsdk.action import ActionError

        try:
            if request.action == "takeoff":
                self.require_armed()
                await self.action.set_takeoff_altitude(request.takeoff_altitude_m)
            await getattr(self.action, request.action)()
            return "accepted", "MAVSDK action succeeded; observe telemetry for physical completion"
        except ActionError as exc:
            result = exc._result.result.name  # Pinned SDK exposes errors through this generated field.
            if result in ("TIMEOUT", "CONNECTION_ERROR", "UNKNOWN", "NO_SYSTEM"):
                return "outcome_unknown", f"MAVSDK {result}"
            return {"UNSUPPORTED": "unsupported", "BUSY": "busy"}.get(result, "rejected"), f"MAVSDK {result}"
        except self.grpc.aio.AioRpcError as exc:
            return "outcome_unknown", f"MAVSDK RPC {exc.code()}"

    async def close(self):
        errors = []
        for task in self.tasks:
            task.cancel()
        if self.tasks:
            await asyncio.gather(*self.tasks, return_exceptions=True)
        if self.process is not None:
            try:
                if self.process.returncode is None:
                    self.process.terminate()
                    try:
                        await asyncio.wait_for(self.process.wait(), 0.5)
                    except asyncio.TimeoutError:
                        self.process.kill()
                        await self.process.wait()
                else:
                    await self.process.wait()
            except ProcessLookupError:
                await self.process.wait()
            except Exception as exc:
                errors.append(exc)
        if self.channel is not None:
            try:
                await self.channel.close()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("MAVSDK cleanup failed") from errors[0]
