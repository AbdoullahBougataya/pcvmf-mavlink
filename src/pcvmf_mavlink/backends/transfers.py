"""MAVLink mission and classic parameter protocols for the pymavlink adapter.

Only the adapter's poll loop reads the transport. Transaction queues contain
messages from the configured autopilot and are discarded at transaction end.
"""

import asyncio
import struct
from functools import partial

from ..transactions import float32, item_from_wire, item_to_wire
from .base import LinkLost

INTEGER_TYPES = {
    1: ("B", 0, 255),
    2: ("b", -128, 127),
    3: ("H", 0, 65535),
    4: ("h", -32768, 32767),
    5: ("I", 0, 2**32 - 1),
    6: ("i", -(2**31), 2**31 - 1),
}


class Transfers:
    def _transaction_message(self, message):
        inbox = self.transaction_inbox
        if inbox is None or message.get_type() not in self.transaction_kinds:
            return
        d = message.to_dict()
        if message.get_type().startswith("MISSION_"):
            if d.get("mission_type", 0) != 0:
                return
            if d.get("target_system", 0) not in (0, self.options.source_system):
                return
            if d.get("target_component", 0) not in (0, self.options.source_component):
                return
        elif d["param_id"] != self.parameter_name:
            return
        if inbox.full():
            raise LinkLost("transaction receive buffer overflow")
        inbox.put_nowait(message)

    async def _response(self, send, accept, timeout=0.5):
        # A fixed timeout per attempt is not extended by irrelevant messages.
        loop = asyncio.get_running_loop()
        for _ in range(6):
            send()
            until = loop.time() + timeout
            while loop.time() < until:
                # No queue.get wait_for race with outer task cancellation on
                # Python 3.10/3.11; the sole receiver fills this bounded queue.
                if self.transaction_inbox.empty():
                    await asyncio.sleep(0.01)
                    continue
                message = self.transaction_inbox.get_nowait()
                if accept(message):
                    return message
                await asyncio.sleep(0)
        raise TimeoutError("transaction response retries exhausted")

    def _mission_ack(self, result):
        self.mav.mission_ack_send(self.options.target_system, self.options.target_component, result, 0)

    async def transfer_mission(self, request):
        if len(request.items) > self.options.max_mission_items:
            raise ValueError("mission exceeds max_mission_items")
        self.transaction_kinds = {
            "MISSION_REQUEST_INT",
            "MISSION_REQUEST",
            "MISSION_COUNT",
            "MISSION_ITEM_INT",
            "MISSION_ACK",
        }
        self.transaction_inbox = asyncio.Queue(128)
        target = (self.options.target_system, self.options.target_component)
        complete = False
        try:
            if request.operation == "clear":
                ack = await self._response(
                    lambda: self.mav.mission_clear_all_send(*target, 0), lambda m: m.get_type() == "MISSION_ACK", 1.5
                )
                complete = True
                return self._mission_result(ack)
            if request.operation == "upload":
                sent = set()
                send = partial(self.mav.mission_count_send, *target, len(request.items), 0)
                timeout = 1.5
                while True:
                    response = await self._response(
                        send,
                        lambda m: m.get_type() in ("MISSION_REQUEST_INT", "MISSION_REQUEST")
                        or (m.get_type() == "MISSION_ACK" and (m.type != 0 or len(sent) == len(request.items))),
                        timeout,
                    )
                    if response.get_type() == "MISSION_ACK":
                        if response.type == 0 and len(sent) != len(request.items):
                            raise LinkLost("mission accepted before every item was requested")
                        complete = True
                        return self._mission_result(response)
                    seq = response.seq
                    if not 0 <= seq < len(request.items):
                        self._mission_ack(13)  # INVALID_SEQUENCE
                        complete = True
                        return "rejected", "autopilot requested an invalid mission sequence", None
                    sent.add(seq)
                    item = item_to_wire(request.items[seq])
                    send = partial(self.mav.mission_item_int_send, *target, seq=seq, mission_type=0, **item)
                    timeout = 0.25
            count = await self._response(
                lambda: self.mav.mission_request_list_send(*target, 0),
                lambda m: m.get_type() == "MISSION_COUNT" or (m.get_type() == "MISSION_ACK" and m.type != 0),
                1.5,
            )
            if count.get_type() == "MISSION_ACK":
                if count.type == 0:
                    raise LinkLost("download received acceptance without mission count")
                complete = True
                return self._mission_result(count)
            if count.count > self.options.max_mission_items:
                raise ValueError("download exceeds max_mission_items")
            items = []
            for seq in range(count.count):
                response = await self._response(
                    lambda: self.mav.mission_request_int_send(*target, seq, 0),
                    lambda m: (m.get_type() == "MISSION_ACK" and m.type != 0)
                    or (m.get_type() == "MISSION_ITEM_INT" and m.seq == seq),
                    0.25,
                )
                if response.get_type() == "MISSION_ACK":
                    if response.type == 0:
                        raise LinkLost("download received acceptance before all items")
                    complete = True
                    return self._mission_result(response)
                items.append(item_from_wire(response.to_dict()))
            self._mission_ack(0)
            complete = True
            return "accepted", "mission downloaded", items
        finally:
            try:
                if not complete:
                    self._mission_ack(15)  # OPERATION_CANCELLED; best effort.
            finally:
                self.transaction_inbox = None
                self.transaction_kinds = set()

    @staticmethod
    def _mission_result(ack):
        return (
            "accepted" if ack.type == 0 else "unsupported" if ack.type in (2, 3) else "rejected",
            f"MISSION_ACK result={ack.type}",
            None,
        )

    def _parameter_value(self, message, expected):
        kind = message.param_type
        if kind == 9 and expected == "float":
            return float32(message.param_value)
        if kind not in INTEGER_TYPES or expected != "int":
            raise ValueError(f"parameter type mismatch/unsupported MAV_PARAM_TYPE={kind}")
        fmt, low, high = INTEGER_TYPES[kind]
        if self.options.firmware == "px4":
            # Read original payload bytes: converting an int bit-pattern through
            # Python float can quiet a signalling NaN and corrupt integer bits.
            wire = bytes(message.get_msgbuf())
            offset = 10 if wire[0] == 0xFD else 6
            # Generated pymavlink get_payload() retains four header bytes for
            # received MAVLink 2 frames in some supported releases.
            value = struct.unpack("<" + fmt, wire[offset : offset + struct.calcsize(fmt)])[0]
        else:
            value = float32(message.param_value)
            if not value.is_integer():
                raise ValueError("autopilot returned a non-integral integer parameter")
            value = int(value)
        if not max(low, -(2**31)) <= value <= min(high, 2**31 - 1):
            raise ValueError("parameter cannot be represented as a signed 32-bit integer")
        return value

    def _set_parameter(self, request, kind):
        value = request.value
        if request.parameter_type == "int":
            fmt, low, high = INTEGER_TYPES[kind]
            if not low <= value <= high:
                raise ValueError("value is outside the autopilot parameter's integer range")
            if self.options.firmware == "px4":
                value_bytes = struct.pack("<" + fmt, value).ljust(4, b"\0")
            else:
                if int(float32(value)) != value:
                    raise ValueError("integer is not exactly representable by ArduCopter's float parameter encoding")
                value_bytes = struct.pack("<f", value)
        else:
            value_bytes = struct.pack("<f", float32(value))
        o = self.options
        message = self.constants.MAVLink_param_set_message(
            o.target_system, o.target_component, request.name.encode("ascii"), 0, kind
        )
        payload = value_bytes + struct.pack(
            "<BB16sB", o.target_system, o.target_component, request.name.encode("ascii"), kind
        )
        # Use the dialect's normal header, CRC and sequence machinery while
        # preserving integer bit patterns in the nominal float field.
        message.pack = lambda mav, force_mavlink1=False: message._pack(mav, message.crc_extra, payload, force_mavlink1)
        self.mav.send(message)

    async def parameter(self, request):
        self.transaction_kinds = {"PARAM_VALUE"}
        self.transaction_inbox = asyncio.Queue(128)
        self.parameter_name = request.name
        read = partial(
            self.mav.param_request_read_send,
            self.options.target_system,
            self.options.target_component,
            request.name.encode("ascii"),
            -1,
        )
        written = False
        try:
            message = await self._response(read, lambda m: True)
            value = self._parameter_value(message, request.parameter_type)
            if request.operation == "set":
                # Discover and check the controller's actual type before writing.
                self.transaction_inbox = asyncio.Queue(128)
                self._set_parameter(request, message.param_type)
                written = True
                expected = float32(request.value) if request.parameter_type == "float" else request.value

                # One write, then read-only confirmation retries. ArduPilot may
                # not send a PARAM_VALUE acknowledgement for PARAM_SET.
                def matches(m):
                    return self._parameter_value(m, request.parameter_type) == expected

                message = await self._response(read, matches)
                value = self._parameter_value(message, request.parameter_type)
            return "accepted", "parameter value observed on the autopilot", value
        except ValueError as exc:
            if written:
                raise LinkLost("parameter was written but readback could not be validated") from exc
            raise
        finally:
            self.transaction_inbox = None
            self.transaction_kinds = set()
            self.parameter_name = None
