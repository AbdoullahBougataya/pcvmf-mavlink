"""Mission and parameter protocol peer with deterministic loss injection."""

import struct
import time

from pymavlink.dialects.v20 import ardupilotmega as mav
from vehicle_peer import VehiclePeer


class TransactionPeer(VehiclePeer):
    def __init__(self, firmware="px4"):
        super().__init__(firmware)
        self.capabilities |= mav.MAV_PROTOCOL_CAPABILITY_MISSION_INT | mav.MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
        self.items = []
        self.upload = {}
        self.params = {"TEST_INT": (6, 42), "TEST_FLOAT": (9, 1.25), "TEST_SMALL": (2, 12)}
        self.transactions = []
        self.drop_once = set()
        self.ignore_sets = False
        self.wrong_address = False
        self.duplicate_request = False
        self.duplicated = False
        self.drop_parameter_echo = False
        self.reject_mission = False
        self.requesting = None
        self.last_request = 0

    def request_item(self, target, seq):
        self.requesting = (target, seq)
        self.last_request = time.monotonic()
        self.mav.mission_request_int_send(*target, seq, 0)

    def telemetry(self):
        super().telemetry()
        if self.requesting and time.monotonic() - self.last_request >= 0.25:
            self.request_item(*self.requesting)

    def parameter_value(self, name):
        kind, value = self.params[name]
        if kind == 9 or self.firmware == "arducopter":
            raw = struct.pack("<f", value)
        else:
            raw = struct.pack("<" + ("i" if kind == 6 else "b"), value).ljust(4, b"\0")
        message = mav.MAVLink_param_value_message(
            name.encode(), 0, kind, len(self.params), list(self.params).index(name)
        )
        payload = raw + struct.pack("<HH16sB", len(self.params), list(self.params).index(name), name.encode(), kind)
        message.pack = lambda link, force_mavlink1=False: message._pack(
            link, message.crc_extra, payload, force_mavlink1
        )
        self.mav.send(message)

    def extra_message(self, msg):
        kind = msg.get_type()
        if kind.startswith("PARAM_") and msg.param_id in self.params:
            self.transactions.append(msg)
            if kind in self.drop_once:
                self.drop_once.remove(kind)
                return True
            if kind == "PARAM_SET" and not self.ignore_sets:
                typ, _ = self.params[msg.param_id]
                if typ == 9 or self.firmware == "arducopter":
                    value = msg.param_value if typ == 9 else int(msg.param_value)
                else:
                    fmt = "i" if typ == 6 else "b"
                    value = struct.unpack("<" + fmt, bytes(msg.get_msgbuf())[10 : 10 + struct.calcsize(fmt)])[0]
                self.params[msg.param_id] = (typ, value)
            if kind != "PARAM_SET" or not self.drop_parameter_echo:
                self.parameter_value(msg.param_id)
            return True
        if not kind.startswith("MISSION_"):
            return False
        self.transactions.append(msg)
        if kind in self.drop_once:
            self.drop_once.remove(kind)
            return True
        target = (msg.get_srcSystem(), msg.get_srcComponent())
        if self.wrong_address:
            target = (target[0] + 1, target[1])
        if self.reject_mission and kind != "MISSION_ACK":
            self.mav.mission_ack_send(*target, 14, 0)
        elif kind == "MISSION_COUNT":
            self.upload = {}
            self.count = msg.count
            self.request_item(target, 0)
        elif kind == "MISSION_ITEM_INT":
            self.upload[msg.seq] = msg.to_dict()
            if self.duplicate_request and not self.duplicated:
                self.duplicated = True
                self.request_item(target, msg.seq)
            elif len(self.upload) == self.count:
                self.items = [self.upload[i] for i in range(self.count)]
                self.requesting = None
                self.mav.mission_ack_send(*target, 0, 0)
            else:
                self.request_item(target, next(i for i in range(self.count) if i not in self.upload))
        elif kind == "MISSION_REQUEST_LIST":
            self.mav.mission_count_send(*target, len(self.items), 0)
        elif kind == "MISSION_REQUEST_INT":
            item = self.items[msg.seq]
            keys = (
                "frame",
                "command",
                "current",
                "autocontinue",
                "param1",
                "param2",
                "param3",
                "param4",
                "x",
                "y",
                "z",
            )
            self.mav.mission_item_int_send(*target, seq=msg.seq, mission_type=0, **{key: item[key] for key in keys})
        elif kind == "MISSION_CLEAR_ALL":
            self.items = []
            self.mav.mission_ack_send(*target, 0, 0)
        elif kind == "MISSION_ACK":
            self.requesting = None
        return True
