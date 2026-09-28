"""Wire peer for local navigation and mission execution (no flight dynamics)."""

import time

from transaction_peer import TransactionPeer


class NavigationPeer(TransactionPeer):
    def __init__(self, firmware="px4"):
        super().__init__(firmware)
        self.armed = True
        self.local = [0.0, 0.0, -2.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.setpoints = []
        self.current_index = 0

    def extra_message(self, message):
        kind = message.get_type()
        target = (message.get_srcSystem(), message.get_srcComponent())
        if kind == "SET_POSITION_TARGET_LOCAL_NED":
            self.setpoints.append((time.monotonic(), message))
            if self.mode in (4, 6 << 16):
                self.local = [message.x, message.y, message.z]
                self.velocity = [message.vx, message.vy, message.vz]
            return True
        if kind == "MISSION_SET_CURRENT":
            self.current_index = message.seq
            self.mav.mission_current_send(self.current_index, len(self.items), 3, 0)
            return True
        if kind == "COMMAND_LONG" and message.command in (193, 224, 300):
            self.commands.append(message)
            if message.command == 300:
                self.mode = 3 if self.firmware == "arducopter" else (4 << 16 | 4 << 24)
            elif message.command == 193:
                self.mode = 5 if self.firmware == "arducopter" else (4 << 16 | 3 << 24)
            else:
                self.current_index = int(message.param1)
            if not self.drop_ack:
                self.mav.command_ack_send(message.command, 0, 0, 0, *target)
            return True
        return super().extra_message(message)

    def telemetry(self):
        super().telemetry()
        self.mav.local_position_ned_send(0, *self.local, *self.velocity)
        self.mav.mission_current_send(self.current_index, len(self.items) or 65535, 3, 0)
