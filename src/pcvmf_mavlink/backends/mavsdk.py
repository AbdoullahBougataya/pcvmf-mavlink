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

from ..navigation import setpoint_fields
from ..transactions import float32, item_from_wire, item_to_wire
from .base import Backend, LinkLost, heartbeat_values

logger = logging.getLogger(__name__)


class MavsdkBackend(Backend):
    def __init__(self, options):
        super().__init__(options)
        self.process = self.channel = None
        self.tasks = []
        self.navigation_ack = None
        self.navigation_command_id = None

    async def connect(self):
        import grpc
        from mavsdk.action import Action
        from mavsdk.mavlink_direct import MavlinkDirect
        from mavsdk.mission_raw import MissionRaw
        from mavsdk.param import Param
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
        self.mission_api = MissionRaw(provider)
        self.param_api = Param(provider)
        self.telemetry_api = Telemetry(provider)
        self.direct = MavlinkDirect(provider)
        self.tasks.append(asyncio.create_task(self._heartbeats()))
        if self.options.setpoints_enabled or self.options.mission_execution_enabled:
            for kind in ("LOCAL_POSITION_NED", "MISSION_CURRENT", "MISSION_ITEM_REACHED", "COMMAND_ACK"):
                self.tasks.append(asyncio.create_task(self._navigation_messages(kind)))
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

    async def _navigation_messages(self, kind):
        async for message in self.direct.message(kind):
            if (message.system_id, message.component_id) != (self.options.target_system, self.options.target_component):
                continue
            d = json.loads(message.fields_json)
            self.navigation_message(kind, d)
            if kind == "COMMAND_ACK" and self.navigation_ack is not None and not self.navigation_ack.done():
                if (
                    d["command"] == self.navigation_command_id
                    and d.get("target_system", 0) in (0, self.options.source_system)
                    and d.get("target_component", 0) in (0, self.options.source_component)
                ):
                    if d["result"] != 5:
                        self.navigation_ack.set_result(d["result"])
        raise LinkLost(f"MAVSDK navigation stream ended: {kind}")

    async def _rates(self):
        names = ["position", "velocity_ned", "attitude_euler", "battery", "gps_info", "landed_state"]
        if self.options.setpoints_enabled:
            names.insert(0, "position_velocity_ned")
        for name in names:
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

    @staticmethod
    def _transaction_error(exc):
        result = exc._result.result.name
        if result in ("TIMEOUT", "CONNECTION_ERROR", "UNKNOWN", "NO_SYSTEM", "TRANSFER_CANCELLED", "PROTOCOL_ERROR"):
            return "outcome_unknown", f"MAVSDK {result}", None
        return (
            {"UNSUPPORTED": "unsupported", "INT_MESSAGES_NOT_SUPPORTED": "unsupported", "BUSY": "busy"}.get(
                result, "rejected"
            ),
            f"MAVSDK {result}",
            None,
        )

    async def transfer_mission(self, request):
        from mavsdk.mission_raw import MissionItem, MissionRawError

        if len(request.items) > self.options.max_mission_items:
            raise ValueError("mission exceeds max_mission_items")
        try:
            if request.operation == "upload":
                items = [
                    MissionItem(seq=index, mission_type=0, **item_to_wire(item))
                    for index, item in enumerate(request.items)
                ]
                await self.mission_api.upload_mission(items)
            elif request.operation == "clear":
                await self.mission_api.clear_mission()
            else:
                downloaded = await self.mission_api.download_mission()
                if len(downloaded) > self.options.max_mission_items:
                    raise ValueError("download exceeds max_mission_items")
                items = []
                for index, item in enumerate(downloaded):
                    if item.seq != index or item.mission_type != 0:
                        raise ValueError("autopilot returned an inconsistent mission sequence/type")
                    items.append(item_from_wire(vars(item)))
                return "accepted", "mission downloaded", items
            return "accepted", "MAVSDK mission transaction succeeded", None
        except asyncio.CancelledError:
            # Cancel server-side transfers too; cancelling the RPC alone is not
            # sufficient, especially when using an externally managed server.
            if request.operation != "clear":
                cancel = (
                    self.mission_api.cancel_mission_upload
                    if request.operation == "upload"
                    else self.mission_api.cancel_mission_download
                )
                try:
                    await asyncio.wait_for(cancel(), 0.25)
                except Exception:
                    logger.debug("MAVSDK transfer cancellation failed", exc_info=True)
            raise
        except MissionRawError as exc:
            if request.operation == "download" and exc._result.result.name == "NO_MISSION_AVAILABLE":
                return "accepted", "no mission stored", []
            return self._transaction_error(exc)
        except self.grpc.aio.AioRpcError as exc:
            return "outcome_unknown", f"MAVSDK RPC {exc.code()}", None

    async def parameter(self, request):
        from mavsdk.param import ParamError

        if self.options.target_component != 1:
            return "unsupported", "MAVSDK parameter API supports autopilot component 1 only", None
        written = False
        try:
            # The pinned server's SelectComponent RPC returns UNKNOWN. Use its
            # default V1/autopilot selection, checked during option validation.
            getter = getattr(self.param_api, f"get_param_{request.parameter_type}")
            value = await getter(request.name)  # Verify name/type before writing.
            if request.operation == "set":
                expected = float32(request.value) if request.parameter_type == "float" else request.value
                written = True
                await getattr(self.param_api, f"set_param_{request.parameter_type}")(request.name, expected)
                value = await getter(request.name)
                if value != expected:
                    return "outcome_unknown", "parameter readback does not match the requested value", None
            if request.parameter_type == "float":
                value = float32(value)
            return "accepted", "parameter value observed on the autopilot", value
        except ParamError as exc:
            if written:
                return "outcome_unknown", f"MAVSDK write/readback {exc._result.result.name}", None
            return self._transaction_error(exc)
        except ValueError as exc:
            if written:
                return "outcome_unknown", f"parameter readback could not be validated: {exc}", None
            raise
        except self.grpc.aio.AioRpcError as exc:
            return "outcome_unknown", f"MAVSDK RPC {exc.code()}", None

    async def _send_direct(self, name, fields):
        from mavsdk.mavlink_direct import MavlinkDirectError, MavlinkMessage

        o = self.options
        try:
            await self.direct.send_message(
                MavlinkMessage(
                    name,
                    o.source_system,
                    o.source_component,
                    o.target_system,
                    o.target_component,
                    json.dumps(fields, allow_nan=False),
                )
            )
        except (MavlinkDirectError, self.grpc.aio.AioRpcError) as exc:
            raise LinkLost(f"MAVSDK direct send failed: {exc}") from exc

    async def send_setpoint(self, point):
        await self._send_direct("SET_POSITION_TARGET_LOCAL_NED", setpoint_fields(point, self.options))

    async def stream_mode(self, enabled):
        return await self._navigation_command(176, [1, 6 if enabled else 4, 0 if enabled else 3])

    async def _navigation_command(self, command, params):
        self.navigation_command_id = command
        self.navigation_ack = asyncio.get_running_loop().create_future()
        try:
            params = params + [0] * (7 - len(params))
            await self._send_direct(
                "COMMAND_LONG",
                dict(
                    target_system=self.options.target_system,
                    target_component=self.options.target_component,
                    command=command,
                    confirmation=0,
                    **{f"param{i + 1}": value for i, value in enumerate(params)},
                ),
            )
            result = await self.navigation_ack
            return (
                "accepted" if result == 0 else "unsupported" if result == 3 else "rejected",
                f"COMMAND_ACK command={command} result={result}",
            )
        finally:
            self.navigation_ack = None
            self.navigation_command_id = None

    async def control_mission(self, request):
        from mavsdk.mission_raw import MissionRawError

        try:
            if request.operation == "mission_start":
                self.require_armed("mission start")
                await self.mission_api.start_mission()
            elif request.operation == "mission_pause":
                await self.mission_api.pause_mission()
            else:
                since = time.monotonic()
                await self._send_direct(
                    "MISSION_SET_CURRENT",
                    dict(
                        target_system=self.options.target_system,
                        target_component=self.options.target_component,
                        seq=request.mission_index,
                    ),
                )
                return await self.wait_mission_current(request.mission_index, since)
            return "accepted", "mission control accepted; observe telemetry for execution"
        except MissionRawError as exc:
            return self._transaction_error(exc)[:2]
        except self.grpc.aio.AioRpcError as exc:
            return "outcome_unknown", f"MAVSDK RPC {exc.code()}"
