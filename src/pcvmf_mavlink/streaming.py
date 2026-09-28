"""One connection's leased stream; no transport or worker objects cross threads."""

import asyncio
import time

from .backends.base import LinkLost
from .navigation import StreamStatus, validate_setpoint_limits


class SetpointStream:
    def __init__(self, adapter, options, session, notify, pull):
        self.adapter, self.options, self.session = adapter, options, session
        self.notify, self.pull = notify, pull
        self.state = "idle"
        self.stream_id = self.requester = self.point = None
        self.until = 0
        self.engaged = False
        self.closed = False
        self.used = set()
        self.mode_lock = asyncio.Lock()
        self._status("idle", "no stream has been started")
        self.task = asyncio.create_task(self._run())

    @property
    def busy(self):
        return self.state in ("priming", "active", "stopping")

    def _status(self, state, reason, emit=True):
        self.state = state
        self.reason = reason
        self.notify(
            StreamStatus(
                self.session, state, self.stream_id, self.requester, self.point.sequence if self.point else None, reason
            ),
            emit,
        )

    def check(self):
        if self.task.done() and not self.task.cancelled():
            error = self.task.exception()
            if error:
                raise error

    async def control(self, request, requester):
        if request.operation == "stream_stop":
            if (request.stream_id, requester) != (self.stream_id, self.requester):
                return "rejected", "only the owner can stop this stream ID"
            return await self.stop("explicit stream stop")
        if self.busy:
            return "busy", "a stream is already in progress"
        if request.stream_id in self.used:
            return "rejected", "stream_id cannot be reused in this connection session"
        if len(self.used) >= self.options.dedup_size:
            return "busy", "stream ID history is full for this session"
        point = request.initial_setpoint
        validate_setpoint_limits(point, self.options)
        self.adapter.require_armed("stream start")
        self.adapter.require_local_position()
        remaining = min(self.options.setpoint_timeout_s, point.expires_at - time.time())
        if remaining <= 0:
            return "expired", "initial setpoint expired before stream start"
        self.used.add(request.stream_id)
        self.point, self.stream_id, self.requester = point, request.stream_id, requester
        self.until = time.monotonic() + remaining
        self.engaged = False
        self._status("priming", "sending fresh setpoints before changing mode")
        # PX4 requires a setpoint stream for more than a second before OFFBOARD.
        priming = 1.1 if self.options.firmware == "px4" else 0.05
        end = time.monotonic() + priming
        while time.monotonic() < end:
            self.check()
            if self.state != "priming":
                return "rejected", "stream expired during priming; send fresh updates during start"
            await asyncio.sleep(0.01)
        async with self.mode_lock:
            if self.state != "priming":
                return "rejected", "stream stopped before mode change"
            self.engaged = True
            outcome, detail = await self.adapter.stream_mode(True)
            if outcome != "accepted":
                self.engaged = False
                self._status("stopped", detail)
                return outcome, detail
            expected = "OFFBOARD" if self.options.firmware == "px4" else "GUIDED"
            while self.adapter.samples["state"].values["flight_mode"] != expected:
                if self.state != "priming":
                    raise LinkLost("stream expired during mode change")
                await asyncio.sleep(0.01)
            if self.state != "priming":
                raise LinkLost("stream expired during mode acknowledgement")
            self._status("active", "stream mode observed; completion still requires telemetry")
        return "accepted", "stream activated"

    async def stop(self, reason, final_state="stopped"):
        if not self.busy:
            return "accepted", "stream is already stopped"
        # Stop emission before waiting for any mode ACK, including an in-flight start.
        self._status("stopping", reason)
        async with self.mode_lock:
            if not self.busy:
                return "accepted", "stream is already stopped"
            if self.engaged:
                response = await self.adapter.stream_mode(False)
                if response[0] != "accepted":
                    raise LinkLost(f"stream stopped sending but hold was not acknowledged: {response[1]}")
            self.engaged = False
            self._status(final_state, reason)
        return "accepted", reason

    def supersede(self, action):
        # Terminal flight actions take over without an intermediate Hold command.
        self.engaged = False
        self._status("stopped", f"stream superseded by {action}")

    async def _run(self):
        next_send = 0.0
        try:
            while not self.closed:
                if self.state in ("priming", "active"):
                    update = self.pull()
                    if update is not None:
                        point, until = update
                        if point.sequence > self.point.sequence:
                            self.point, self.until = point, until
                            self._status(self.state, self.reason, False)
                    now = time.monotonic()
                    if now >= self.until:
                        await asyncio.wait_for(
                            self.stop("setpoint lease expired", "expired"), self.options.stream_stop_timeout_s
                        )
                    else:
                        self.adapter.require_armed("streaming")
                        self.adapter.require_local_position()
                        if self.state == "active":
                            expected = "OFFBOARD" if self.options.firmware == "px4" else "GUIDED"
                            if self.adapter.samples["state"].values["flight_mode"] != expected:
                                raise LinkLost("vehicle left stream mode; stream invalidated")
                        if now >= next_send:
                            remaining = self.until - now
                            try:
                                await asyncio.wait_for(self.adapter.send_setpoint(self.point), min(0.2, remaining))
                            except (TimeoutError, asyncio.TimeoutError) as exc:
                                if remaining <= 0.2 and time.monotonic() >= self.until:
                                    # This send was bounded by the point's lease. Recheck
                                    # fresh input, then expire through the normal Hold path.
                                    continue
                                raise LinkLost("setpoint send timed out") from exc
                            next_send = time.monotonic() + 1 / self.options.setpoint_hz
                await asyncio.sleep(0.01)
        except (ValueError, OSError, TimeoutError, asyncio.TimeoutError) as exc:
            self._status("lost", str(exc)[:512] or "stream failed")
            raise LinkLost(self.reason) from exc

    async def close(self):
        self.closed = True
        self.task.cancel()
        await asyncio.gather(self.task, return_exceptions=True)
        if self.busy:
            self._status("lost", "connection closed; setpoints discarded")
