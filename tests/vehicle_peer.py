"""A small wire-protocol test peer, not a flight dynamics simulator."""

import asyncio
import os
import socket
import time

from pymavlink.dialects.v20 import ardupilotmega as mav


class VehiclePeer:
    def __init__(self, firmware="px4"):
        self.firmware = firmware
        self.mav = mav.MAVLink(self, srcSystem=1, srcComponent=1)
        self.mav.robust_parsing = True
        self.armed = False
        self.altitude = 0
        self.mode = 4 if firmware == "arducopter" else 4 << 16 | 3 << 24
        self.commands = []
        self.send = None
        self.task = None
        self.socket = self.server = self.writer = None
        self.master = self.slave = None
        self.drop_ack = False
        self.transmit = True
        self.takeoff_altitude = 2.0
        self.closed = False
        self.drop_after_action = False
        self.capabilities = 8192

    def write(self, data):
        if self.send and self.transmit:
            self.send(data)

    def receive(self, data):
        for msg in self.mav.parse_buffer(data) or []:
            if self.extra_message(msg):
                continue
            if msg.get_type() == "COMMAND_LONG":
                self.commands.append(msg)
                if msg.command == 400:
                    self.armed = bool(msg.param1)
                    if self.drop_after_action:
                        self.transmit = False
                elif msg.command == 176:
                    self.mode = (
                        int(msg.param2)
                        if self.firmware == "arducopter"
                        else int(msg.param2) << 16 | int(msg.param3) << 24
                    )
                    if self.firmware == "px4" and int(msg.param3) == 5:
                        self.altitude = 0
                elif msg.command == 22:
                    self.altitude = msg.param7 if self.firmware == "arducopter" else msg.param7 - 488
                    if not 0 < self.altitude < 1000:
                        self.altitude = self.takeoff_altitude
                elif msg.command in (20, 21):
                    self.altitude = 0
                if not self.drop_ack:
                    self.mav.command_ack_send(msg.command, 0, 0, 0, msg.get_srcSystem(), msg.get_srcComponent())
                if msg.command == 520:
                    self.mav.autopilot_version_send(
                        self.capabilities, 0x010F0000, 0, 0, 0, [0] * 8, [0] * 8, [0] * 8, 0, 0, 42
                    )
            elif msg.get_type() in ("PARAM_REQUEST_READ", "PARAM_SET"):
                if msg.get_type() == "PARAM_SET":
                    self.takeoff_altitude = msg.param_value
                param_id = msg.param_id.encode() if isinstance(msg.param_id, str) else msg.param_id
                self.mav.param_value_send(param_id, self.takeoff_altitude, 9, 1, 0)

    def extra_message(self, message):
        return False

    def telemetry(self):
        self.mav.heartbeat_send(2, 12 if self.firmware == "px4" else 3, 1 | (128 if self.armed else 0), self.mode, 4)
        self.mav.sys_status_send(7, 7, 7, 10, 16200, 100, 90, 0, 0, 0, 0, 0, 0)
        self.mav.attitude_send(100, 0.1, 0.2, 0.3, 0, 0, 0)
        self.mav.global_position_int_send(
            100, 473977420, 85455940, int((488 + self.altitude) * 1000), int(self.altitude * 1000), 120, -50, 20, 9000
        )
        self.mav.gps_raw_int_send(int(time.time() * 1e6), 3, 473977420, 85455940, 488000, 80, 100, 0, 0, 12)
        self.mav.extended_sys_state_send(0, 1 if self.altitude == 0 else 2)
        self.mav.home_position_send(473977420, 85455940, 488000, 0, 0, 0, [1, 0, 0, 0], 0, 0, 0)

    async def open(self, transport="udpin"):
        if transport in ("udpin", "udpout"):
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(("127.0.0.1", 0))
            self.socket.setblocking(False)
            if transport == "udpin":
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                    reserve.bind(("127.0.0.1", 0))
                    port = reserve.getsockname()[1]
                self.send = lambda data: self.socket.sendto(data, ("127.0.0.1", port))
            else:
                port = self.socket.getsockname()[1]

            async def udp_loop():
                while not self.closed:
                    await asyncio.sleep(0)
                    try:
                        data, addr = self.socket.recvfrom(8192)
                        if transport == "udpout":
                            self.send = lambda data: self.socket.sendto(data, addr)
                        self.receive(data)
                    except BlockingIOError:
                        await asyncio.sleep(0.03)
                    self.telemetry()

            self.task = asyncio.create_task(udp_loop())
            return {"transport": transport, "host": "127.0.0.1", "port": port}
        if transport == "tcp":

            async def connected(reader, writer):
                self.writer = writer
                self.send = writer.write
                while not self.closed:
                    try:
                        data = await asyncio.wait_for(reader.read(8192), 0.03)
                        if not data:
                            return
                        self.receive(data)
                    except asyncio.TimeoutError:
                        pass
                    self.telemetry()

            def start(reader, writer):
                self.task = asyncio.create_task(connected(reader, writer))

            self.server = await asyncio.start_server(start, "127.0.0.1", 0)
            return {"transport": "tcp", "host": "127.0.0.1", "port": self.server.sockets[0].getsockname()[1]}
        self.master, self.slave = os.openpty()
        os.set_blocking(self.master, False)
        self.send = lambda data: os.write(self.master, data)

        async def serial_loop():
            while not self.closed:
                await asyncio.sleep(0.03)
                try:
                    self.receive(os.read(self.master, 8192))
                except BlockingIOError:
                    pass
                self.telemetry()

        self.task = asyncio.create_task(serial_loop())
        return {"transport": "serial", "device": os.ttyname(self.slave), "baud": 115200}

    async def close(self):
        self.closed = True
        if self.task is not None:
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
        if self.writer is not None:
            self.writer.close()
            await self.writer.wait_closed()
        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()
        if self.socket is not None:
            self.socket.close()
        for fd in (self.master, self.slave):
            if fd is not None:
                os.close(fd)
