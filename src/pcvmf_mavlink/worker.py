"""Public PCVMF worker. Only this thread accesses injected transports."""

import queue
from collections import OrderedDict

from pcvmf.api import Worker

from .messages import ActionRequest, ActionRequestCodec, ActionResult, ConnectionStatus
from .options import Options
from .service import FlightService


class FlightControllerWorker(Worker):
    @classmethod
    def validate_options(cls, options):
        Options.parse(options)

    def initialize(self, context):
        self.context = context
        self.config = Options.parse(self.options)
        self.service = None
        if context.publisher is None:
            raise ValueError("FlightControllerWorker requires telemetry, status and result publications")
        self.cache = OrderedDict()
        self.pending = None
        self.last_status = self.last_telemetry = float("-inf")
        self.service = FlightService(self.config)
        self.service.start()

    def _publish_result(self, result, request=None):
        key = (result.requester, result.session_id, result.request_id)
        if request is not None:
            self.cache[key] = (request, result)
        elif key in self.cache:
            self.cache[key] = (self.cache[key][0], result)
        if key == self.pending and result.outcome != "processing":
            self.pending = None
        self.context.publisher.publish(self.config.result_topic, result)

    def _command(self, message):
        request = message.payload
        if message.topic != self.config.command_topic or type(request) is not ActionRequest:
            return
        ActionRequestCodec().encode(request)
        key = (message.source, request.session_id, request.request_id)
        # Retain executed requests until expiry so cache pressure cannot make
        # a still-valid retransmission execute for a second time.
        status, _ = self.service.snapshot()
        for cached_key, (cached_request, _) in list(self.cache.items()):
            if cached_key != self.pending and (
                cached_request.expires_at <= self.context.wall_clock()
                or (status is not None and cached_request.session_id != status.session_id)
            ):
                del self.cache[cached_key]
        if key in self.cache:
            previous, result = self.cache[key]
            if previous == request:
                self.context.publisher.publish(self.config.result_topic, result)
            else:
                self.context.publisher.publish(
                    self.config.result_topic,
                    ActionResult(
                        request.request_id,
                        request.session_id,
                        message.source,
                        request.action,
                        "request_conflict",
                        "request ID already used with different content",
                    ),
                )
            return
        if len(self.cache) >= self.config.dedup_size:
            self.context.publisher.publish(
                self.config.result_topic,
                ActionResult(
                    request.request_id,
                    request.session_id,
                    message.source,
                    request.action,
                    "busy",
                    "deduplication cache is full; request was not submitted",
                ),
            )
            return
        if not self.config.commands_enabled:
            outcome, detail = "disabled", "commands_enabled is false"
        elif request.expires_at <= self.context.wall_clock():
            outcome, detail = "expired", "request deadline has passed"
        elif status is None or status.state != "connected":
            outcome, detail = "disconnected", "vehicle is disconnected"
        elif request.session_id != status.session_id:
            outcome, detail = "session_mismatch", "request belongs to an old connection"
        elif self.pending is not None:
            outcome, detail = "busy", "another action transaction is in progress"
        else:
            try:
                self.service.submit(request, message.source)
                self.pending = key
                outcome, detail = "processing", "queued for this connection session"
            except queue.Full:
                outcome, detail = "busy", "command queue is full"
        self._publish_result(
            ActionResult(request.request_id, request.session_id, message.source, request.action, outcome, detail),
            request,
        )

    def step(self):
        self.service.check()
        now = self.context.monotonic()
        deadline = now + self.config.receive_budget_ms / 1000
        for _ in range(self.config.max_messages):
            try:
                event = self.service.events.get_nowait()
            except queue.Empty:
                break
            if isinstance(event, ConnectionStatus):
                self.context.publisher.publish(self.config.status_topic, event)
                self.last_status = now
            else:
                self._publish_result(event)
            if self.context.monotonic() >= deadline:
                break
        subscriber = self.context.subscriber
        for _ in range(self.config.max_messages):
            if subscriber is None or self.context.monotonic() >= deadline:
                break
            message = subscriber.receive(timeout_ms=0)
            if message is not None:
                self._command(message)
        status, telemetry = self.service.snapshot()
        if status is not None and now - self.last_status >= 1:
            self.context.publisher.publish(self.config.status_topic, status)
            self.last_status = now
        if telemetry is not None and now - self.last_telemetry >= 1 / self.config.telemetry_hz:
            self.context.publisher.publish(self.config.telemetry_topic, telemetry)
            self.last_telemetry = now
        return None

    def cleanup(self):
        service = getattr(self, "service", None)
        self.service = None
        if service is not None:
            service.close()
