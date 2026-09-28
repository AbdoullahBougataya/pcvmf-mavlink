"""Connection lifecycle and command execution, isolated from PCVMF transports."""

import asyncio
import queue
import threading
import time
import uuid
from dataclasses import replace
from functools import partial

from .backends import create
from .backends.base import LinkLost
from .messages import ACTIONS, ActionRequest, ConnectionStatus, VehicleTelemetry
from .navigation import ControlRequest, NavigationTelemetry, validate_setpoint_limits
from .streaming import SetpointStream
from .transactions import MissionRequest, ParameterRequest, disabled_reason, result_for


class FlightService:
    def __init__(self, options, factory=create):
        self.options = options
        self.factory = factory
        self.events = queue.Queue(options.event_queue_size)
        self.commands = queue.Queue(1)
        self.ready = threading.Event()
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.error = None
        self.status = None
        self.telemetry = None
        self.loop = None
        self.task = None
        self.thread = None
        self.streaming = None
        self.stream_status = self.navigation = self.setpoint_update = None

    def start(self):
        self.thread = threading.Thread(target=self._thread_main, name="flight-io", daemon=True)
        self.thread.start()
        if not self.ready.wait(self.options.connect_timeout_s + 0.5):
            raise TimeoutError("flight-controller initial connection timed out")
        self.check()

    def check(self):
        if self.error is not None:
            raise RuntimeError("flight-controller I/O failed") from self.error

    def snapshot(self):
        with self.lock:
            return self.status, self.telemetry

    def submit(self, request, requester):
        self.commands.put_nowait((request, requester))

    def navigation_snapshot(self):
        with self.lock:
            return self.stream_status, self.navigation

    def submit_setpoint(self, point, requester):
        if not self.options.setpoints_enabled:
            return False
        try:
            validate_setpoint_limits(point, self.options)
        except ValueError:
            return False
        remaining = min(self.options.setpoint_timeout_s, point.expires_at - time.time())
        if remaining <= 0:
            return False
        with self.lock:
            status = self.stream_status
            if self.status is None or self.status.state != "connected" or status is None:
                return False
            if status.state not in ("priming", "active") or (point.session_id, point.stream_id, requester) != (
                status.session_id,
                status.stream_id,
                status.requester,
            ):
                return False
            sequence = self.setpoint_update[0].sequence if self.setpoint_update else status.last_sequence
            if point.sequence <= sequence:
                return False
            self.setpoint_update = (point, time.monotonic() + remaining)
        return True

    def _pull_setpoint(self):
        with self.lock:
            point, self.setpoint_update = self.setpoint_update, None
            return point

    def _stream_status(self, status, emit):
        with self.lock:
            self.stream_status = status
            if status.state not in ("priming", "active"):
                self.setpoint_update = None
        if emit:
            self.emit(status)

    def emit(self, event):
        # Losing a result silently would conceal the outcome of an action.
        self.events.put_nowait(event)

    def _status(self, state, session, reason):
        o = self.options
        status = ConnectionStatus(
            state,
            o.backend,
            o.firmware,
            o.target_system,
            o.target_component,
            session,
            list(ACTIONS),
            o.commands_enabled,
            reason[:512] or state,
        )
        with self.lock:
            self.status = status
            if state != "connected":
                self.telemetry = None
                self.navigation = self.setpoint_update = None
        self.emit(status)

    def _telemetry(self, adapter, session):
        now = time.monotonic()
        samples = {
            key: replace(
                value,
                age_s=max(0, now - adapter.sample_times[key]),
                stale=now - adapter.sample_times[key] > self.options.telemetry_stale_s,
            )
            for key, value in adapter.samples.items()
        }
        with self.lock:
            self.telemetry = VehicleTelemetry(
                session, self.options.target_system, self.options.target_component, samples
            )
            if self.options.setpoints_enabled or self.options.mission_execution_enabled:
                self.navigation = NavigationTelemetry(
                    session,
                    {
                        key: replace(
                            value,
                            age_s=max(0, now - adapter.navigation_times[key]),
                            stale=now - adapter.navigation_times[key] > self.options.telemetry_stale_s,
                        )
                        for key, value in adapter.navigation_samples.items()
                    },
                )

    def _thread_main(self):
        try:
            asyncio.run(self._run())
        except BaseException as exc:
            if not isinstance(exc, asyncio.CancelledError):
                self.error = exc
        finally:
            self.ready.set()

    async def _execute(self, adapter, request, requester):
        timeout = self.options.action_timeout_s
        operation = adapter.execute
        if type(request) is MissionRequest:
            timeout, operation = self.options.transfer_timeout_s, adapter.transfer_mission
        elif type(request) is ParameterRequest:
            timeout, operation = self.options.parameter_timeout_s, adapter.parameter
        elif type(request) is ControlRequest:
            operation = adapter.control_mission
            if request.operation.startswith("stream_"):
                operation = partial(self.streaming.control, requester=requester) if self.streaming else operation
        deadline = min(timeout, request.expires_at - time.time())
        data = None
        disabled = disabled_reason(request, self.options)
        if disabled:
            return result_for(request, requester, "disabled", disabled)
        if deadline <= 0:
            outcome, detail = "expired", "request expired before transmission"
        else:
            try:
                if self.streaming is not None and self.streaming.busy:
                    if type(request) is ActionRequest and request.action in ("land", "return_to_launch", "disarm"):
                        self.streaming.supersede(request.action)
                    elif not (type(request) is ControlRequest and request.operation == "stream_stop"):
                        return result_for(request, requester, "busy", "stop the active stream before this transaction")
                response = await asyncio.wait_for(operation(request), deadline)
                outcome, detail = response[:2]
                if len(response) == 3:
                    data = response[2]
            except ValueError as exc:
                outcome, detail = "rejected", str(exc)
            except (TimeoutError, asyncio.TimeoutError, OSError):
                outcome, detail = "outcome_unknown", "no definitive acknowledgement before the deadline"
        return result_for(request, requester, outcome, detail, data)

    async def _run(self):
        self.loop = asyncio.get_running_loop()
        self.task = asyncio.current_task()
        o = self.options
        first = True
        backoff = o.reconnect_initial_s
        while not self.stopping.is_set():
            adapter = self.factory(o)
            active = None
            active_request = None
            session = str(uuid.uuid4())
            try:
                await asyncio.wait_for(adapter.connect(), o.connect_timeout_s)
                self._status("connected", session, "expected vehicle connected")
                if o.setpoints_enabled:
                    self.streaming = SetpointStream(adapter, o, session, self._stream_status, self._pull_setpoint)
                first = False
                self.ready.set()
                connected_at = time.monotonic()
                while not self.stopping.is_set():
                    await adapter.poll()
                    if self.streaming is not None:
                        self.streaming.check()
                    if time.monotonic() - adapter.last_heartbeat > o.link_timeout_s:
                        raise LinkLost("heartbeat timed out")
                    self._telemetry(adapter, session)
                    if active is not None and active.done():
                        result = active.result()
                        active = None
                        active_request = None
                        self.emit(result)
                        if result.outcome == "outcome_unknown":
                            raise LinkLost("resetting connection after an uncertain command outcome")
                    if time.monotonic() - connected_at > o.link_timeout_s:
                        backoff = o.reconnect_initial_s
                    if active is None:
                        try:
                            request, requester = self.commands.get_nowait()
                        except queue.Empty:
                            continue
                        if request.session_id != session:
                            self.emit(
                                result_for(
                                    request,
                                    requester,
                                    "session_mismatch",
                                    "connection session changed before transmission",
                                )
                            )
                        else:
                            active_request = (request, requester)
                            active = asyncio.create_task(self._execute(adapter, request, requester))
            except (OSError, TimeoutError, asyncio.TimeoutError) as exc:
                if first:
                    raise
                self._status("disconnected", session, str(exc))
            finally:
                try:
                    if active is not None:
                        active.cancel()
                        await asyncio.gather(active, return_exceptions=True)
                        request, requester = active_request
                        self.emit(
                            result_for(
                                request,
                                requester,
                                "outcome_unknown",
                                "connection stopped during command transaction",
                            )
                        )
                    while not self.commands.empty():
                        request, requester = self.commands.get_nowait()
                        self.emit(
                            result_for(
                                request,
                                requester,
                                "disconnected",
                                "connection stopped before command transaction",
                            )
                        )
                finally:
                    try:
                        if self.streaming is not None:
                            await self.streaming.close()
                            self.streaming = None
                    finally:
                        await asyncio.wait_for(adapter.close(), o.cleanup_timeout_s)
            if not self.stopping.is_set():
                await asyncio.sleep(backoff)
                backoff = min(o.reconnect_max_s, backoff * 2)

    def close(self):
        self.stopping.set()
        if self.loop is not None and self.task is not None and not self.loop.is_closed():
            try:
                self.loop.call_soon_threadsafe(self.task.cancel)
            except RuntimeError:
                pass
        if self.thread is not None:
            self.thread.join(self.options.cleanup_timeout_s + 0.5)
            if self.thread.is_alive():
                raise RuntimeError("flight I/O thread did not stop within cleanup allowance")
        self.check()
