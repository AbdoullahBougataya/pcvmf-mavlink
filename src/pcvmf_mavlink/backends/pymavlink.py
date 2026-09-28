"""MAVLink 2 adapter using pymavlink's generated dialect and bounded transports.

Avoid mavutil's blocking TCP connect/reconnect and process-global dialect selection.
The single receive path dispatches both telemetry and acknowledgements.
"""

import asyncio
import logging
import time

from ..navigation import setpoint_fields
from .base import Backend, LinkLost, heartbeat_values
from .transfers import Transfers

logger = logging.getLogger(__name__)


class _Datagrams(asyncio.DatagramProtocol):
    def __init__(self):
        self.packets = asyncio.Queue(128)
        self.error = None

    def datagram_received(self, data, addr):
        if len(data) > 8192 or self.packets.full():
            self.error = LinkLost("MAVLink receive buffer overflow")
        else:
            self.packets.put_nowait((data, addr))

    def error_received(self, exc):
        self.error = exc


class PymavlinkBackend(Transfers, Backend):
    def __init__(self, options):
        super().__init__(options)
        self.udp = self.protocol = self.reader = self.writer = self.serial = None
        self.peer = None
        self.ack = None
        self.command_id = None
        self.last_sent_heartbeat = 0.0
        self.home_altitude = None
        self.read_task = None
        self.transaction_inbox = None
        self.transaction_kinds = set()
        self.parameter_name = None

    async def connect(self):
        from pymavlink.dialects.v20 import ardupilotmega as mavlink

        self.constants = mavlink
        self.mav = mavlink.MAVLink(
            self, srcSystem=self.options.source_system, srcComponent=self.options.source_component
        )
        self.mav.robust_parsing = True
        c = self.options.connection
        loop = asyncio.get_running_loop()
        if c["transport"] in ("udpin", "udpout"):
            args = {"local_addr" if c["transport"] == "udpin" else "remote_addr": (c["host"], c["port"])}
            self.udp, self.protocol = await loop.create_datagram_endpoint(_Datagrams, **args)
        elif c["transport"] == "tcp":
            self.reader, self.writer = await asyncio.open_connection(c["host"], c["port"])
            self.read_task = asyncio.create_task(self.reader.read(8192))
        else:
            import serial

            self.serial = serial.Serial(c["device"], c["baud"], timeout=0, write_timeout=0.1)
        while not self.last_heartbeat:
            await self.poll()
        # Requests are best-effort; unavailable telemetry remains absent/stale.
        ids = [1, 24, 30, 33, 242, 245]
        if self.options.setpoints_enabled or self.options.mission_execution_enabled:
            ids += [32, 42, 46]
        for message_id in ids:
            self.mav.command_long_send(
                self.options.target_system, self.options.target_component, 511, 0, message_id, 100000, 0, 0, 0, 0, 0
            )

    def write(self, data):
        if self.udp is not None:
            if self.options.connection["transport"] == "udpout":
                self.udp.sendto(data)
            elif self.peer is not None:
                self.udp.sendto(data, self.peer)
        elif self.writer is not None:
            if self.writer.is_closing() or self.writer.transport.get_write_buffer_size() > 65536:
                raise LinkLost("TCP send buffer is unavailable")
            self.writer.write(data)
        elif self.serial is not None:
            if self.serial.write(data) != len(data):
                raise LinkLost("serial write was incomplete")

    async def poll(self):
        # Yield even under continuous traffic. Avoid repeatedly wrapping reads
        # in wait_for: input racing outer cancellation can swallow cancellation
        # on older supported asyncio versions.
        await asyncio.sleep(0)
        now = time.monotonic()
        if now - self.last_sent_heartbeat >= 1:
            self.mav.heartbeat_send(6, 8, 0, 0, 4)  # GCS, invalid autopilot, active
            self.last_sent_heartbeat = now
        peer = None
        if self.protocol is not None:
            if self.protocol.error:
                raise self.protocol.error
            if self.protocol.packets.empty():
                await asyncio.sleep(0.01)
                return
            data, peer = self.protocol.packets.get_nowait()
            if self.peer is not None and peer != self.peer:
                return
        elif self.reader is not None:
            if not self.read_task.done():
                await asyncio.sleep(0.01)
                return
            data = self.read_task.result()
            if not data:
                raise LinkLost("TCP peer closed the connection")
            self.read_task = asyncio.create_task(self.reader.read(8192))
        else:
            data = self.serial.read(8192)
            if not data:
                await asyncio.sleep(0.01)
                return
        for message in self.mav.parse_buffer(data) or []:
            if (message.get_srcSystem(), message.get_srcComponent()) != (
                self.options.target_system,
                self.options.target_component,
            ):
                continue
            if message.get_type() == "HEARTBEAT" and peer is not None:
                self.peer = peer
            self._message(message)

    def _message(self, message):
        self._transaction_message(message)
        kind = message.get_type()
        d = message.to_dict()
        self.navigation_message(kind, d)
        if kind == "HEARTBEAT":
            self.update("state", heartbeat_values(d, self.options))
            self.last_heartbeat = time.monotonic()
        elif kind == "ATTITUDE":
            self.update("attitude", {"roll_rad": d["roll"], "pitch_rad": d["pitch"], "yaw_rad": d["yaw"]})
        elif kind == "GLOBAL_POSITION_INT":
            self.update(
                "position",
                {
                    "latitude_deg": d["lat"] / 1e7,
                    "longitude_deg": d["lon"] / 1e7,
                    "altitude_msl_m": d["alt"] / 1000,
                    "altitude_relative_home_m": d["relative_alt"] / 1000,
                },
            )
            self.update("velocity", {"north_m_s": d["vx"] / 100, "east_m_s": d["vy"] / 100, "down_m_s": d["vz"] / 100})
        elif kind == "SYS_STATUS":
            self.update(
                "battery",
                {
                    "voltage_v": None if d["voltage_battery"] == 65535 else d["voltage_battery"] / 1000,
                    "remaining_fraction": None if d["battery_remaining"] < 0 else d["battery_remaining"] / 100,
                },
            )
            enabled = d["onboard_control_sensors_enabled"]
            self.update("health", {"sensors_ok": (d["onboard_control_sensors_health"] & enabled) == enabled})
        elif kind == "GPS_RAW_INT":
            self.update(
                "gps",
                {
                    "fix_type": d["fix_type"],
                    "satellites_visible": None if d["satellites_visible"] == 255 else d["satellites_visible"],
                },
            )
        elif kind == "EXTENDED_SYS_STATE":
            self.update(
                "landed",
                {
                    "landed_state": {0: "UNKNOWN", 1: "ON_GROUND", 2: "IN_AIR", 3: "TAKING_OFF", 4: "LANDING"}.get(
                        d["landed_state"], "UNKNOWN"
                    )
                },
            )
        elif kind == "HOME_POSITION":
            self.home_altitude = d["altitude"] / 1000
        elif kind == "STATUSTEXT":
            logger.log(logging.WARNING if d["severity"] <= 4 else logging.INFO, "autopilot: %s", d["text"])
        elif kind == "COMMAND_ACK" and self.ack is not None and not self.ack.done():
            if d["command"] != self.command_id:
                return
            if d.get("target_system", 0) not in (0, self.options.source_system):
                return
            if d.get("target_component", 0) not in (0, self.options.source_component):
                return
            if d["result"] != 5:  # IN_PROGRESS keeps the bounded transaction open.
                self.ack.set_result(d["result"])

    async def _command(self, command_id, params):
        self.command_id = command_id
        self.ack = asyncio.get_running_loop().create_future()
        try:
            self.mav.command_long_send(
                self.options.target_system,
                self.options.target_component,
                command_id,
                0,
                *(params + [0] * (7 - len(params))),
            )
            result = await self.ack
            outcome = "accepted" if result == 0 else "unsupported" if result in (3, 7, 8, 9) else "rejected"
            return outcome, f"COMMAND_ACK command={command_id} result={result}"
        finally:
            self.ack = None
            self.command_id = None

    async def execute(self, request):
        action = request.action
        if action in ("arm", "disarm"):
            return await self._command(400, [int(action == "arm")])
        if action == "takeoff":
            self.require_armed()
            altitude = request.takeoff_altitude_m
            if self.options.firmware == "arducopter":
                outcome, detail = await self._command(176, [1, 4])  # CUSTOM_MODE_ENABLED, GUIDED
                if outcome != "accepted":
                    return outcome, f"Guided mode prerequisite: {detail}"
                while self.samples["state"].values["flight_mode"] != "GUIDED":
                    await asyncio.sleep(0.02)
                self.require_armed()
            else:
                if self.home_altitude is None:
                    raise ValueError("PX4 takeoff requires HOME_POSITION to convert home-relative altitude to AMSL")
                altitude += self.home_altitude
            return await self._command(22, [0, 0, 0, float("nan"), float("nan"), float("nan"), altitude])
        if action == "land":
            return await self._command(21, [0, 0, 0, float("nan"), float("nan"), float("nan"), 0])
        return await self._command(20, [])

    async def send_setpoint(self, point):
        self.mav.set_position_target_local_ned_send(**setpoint_fields(point, self.options))

    async def stream_mode(self, enabled):
        if self.options.firmware == "px4":
            params = [1, 6] if enabled else [1, 4, 3]  # OFFBOARD / AUTO.LOITER
        else:
            params = [1, 4 if enabled else 5]  # GUIDED / LOITER
        return await self._command(176, params)

    async def control_mission(self, request):
        if request.operation == "mission_start":
            self.require_armed("mission start")
            if self.options.firmware == "px4":
                # PX4's MISSION_START command may arm the vehicle. A mode change
                # starts/resumes the stored mission without implicit arming.
                return await self._command(176, [1, 4, 4])
            return await self._command(300, [0, 0])
        if request.operation == "mission_pause":
            if self.options.firmware == "px4":
                return await self.stream_mode(False)
            return await self._command(193, [0])
        if self.options.firmware == "px4":
            since = time.monotonic()
            self.mav.mission_set_current_send(
                self.options.target_system, self.options.target_component, request.mission_index
            )
            return await self.wait_mission_current(request.mission_index, since)
        return await self._command(224, [request.mission_index, 0])

    async def close(self):
        errors = []
        if self.read_task is not None:
            self.read_task.cancel()
            await asyncio.gather(self.read_task, return_exceptions=True)
        for resource in (self.udp, self.writer, self.serial):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    errors.append(exc)
        if self.writer is not None:
            try:
                await self.writer.wait_closed()
            except Exception as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError("failed to close MAVLink transport") from errors[0]
