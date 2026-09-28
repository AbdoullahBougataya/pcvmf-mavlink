"""Resource-free validation and backend-specific connection URL conversion."""

from dataclasses import dataclass, fields

from .messages import choice, number, text


@dataclass(frozen=True)
class Options:
    backend: str
    firmware: str
    connection: dict
    commands_enabled: bool = False
    missions_enabled: bool = False
    parameters_enabled: bool = False
    parameter_writes_enabled: bool = False
    transfer_timeout_s: float = 30
    parameter_timeout_s: float = 5
    max_mission_items: int = 500
    setpoints_enabled: bool = False
    mission_execution_enabled: bool = False
    setpoint_hz: float = 20
    setpoint_timeout_s: float = 0.5
    stream_stop_timeout_s: float = 2
    max_setpoint_speed_m_s: float = 5
    max_setpoint_distance_m: float = 100
    setpoint_topic: str = "flight/setpoint"
    stream_status_topic: str = "flight/stream_status"
    navigation_topic: str = "flight/navigation"
    target_system: int = 1
    target_component: int = 1
    source_system: int = 245
    source_component: int = 190
    connect_timeout_s: float = 15
    link_timeout_s: float = 3
    action_timeout_s: float = 5
    telemetry_stale_s: float = 3
    telemetry_hz: float = 10
    reconnect_initial_s: float = 1
    reconnect_max_s: float = 10
    cleanup_timeout_s: float = 2
    max_messages: int = 100
    receive_budget_ms: float = 5
    event_queue_size: int = 256
    dedup_size: int = 1024
    telemetry_topic: str = "flight/telemetry"
    status_topic: str = "flight/status"
    command_topic: str = "flight/command"
    result_topic: str = "flight/result"
    server: dict | None = None

    @classmethod
    def parse(cls, raw):
        if not isinstance(raw, dict) or set(raw) - {f.name for f in fields(cls)}:
            raise ValueError("unknown flight-controller option")
        if not {"backend", "firmware", "connection"} <= set(raw):
            raise ValueError("backend, firmware and connection are required")
        o = cls(**raw)
        choice(o.backend, ("pymavlink", "mavsdk", "fake"), "backend")
        choice(o.firmware, ("px4", "arducopter"), "firmware")
        if o.backend == "mavsdk" and o.firmware != "px4":
            raise ValueError("MAVSDK/ArduCopter is not supported in this release")
        for key in (
            "commands_enabled",
            "missions_enabled",
            "parameters_enabled",
            "parameter_writes_enabled",
            "setpoints_enabled",
            "mission_execution_enabled",
        ):
            if type(getattr(o, key)) is not bool:
                raise ValueError(f"{key} must be boolean")
        if o.parameter_writes_enabled and not o.parameters_enabled:
            raise ValueError("parameter_writes_enabled requires parameters_enabled")
        if o.backend == "mavsdk" and o.parameters_enabled and o.target_component != 1:
            raise ValueError("MAVSDK parameter API supports autopilot component 1 only")
        number(o.max_mission_items, "max_mission_items", 1, 10000, True)
        number(o.setpoint_hz, "setpoint_hz", 5, 50)
        number(o.setpoint_timeout_s, "setpoint_timeout_s", 0.1, 5)
        number(o.stream_stop_timeout_s, "stream_stop_timeout_s", 0.1, 5)
        number(o.max_setpoint_speed_m_s, "max_setpoint_speed_m_s", 0.01, 1000)
        number(o.max_setpoint_distance_m, "max_setpoint_distance_m", 0.01, 1000000)
        for k in ("target_system", "target_component", "source_system", "source_component"):
            number(getattr(o, k), k, 1, 255, True)
        if o.source_system == o.target_system:
            raise ValueError("source_system must differ from target_system")
        for k in (
            "connect_timeout_s",
            "link_timeout_s",
            "action_timeout_s",
            "transfer_timeout_s",
            "parameter_timeout_s",
            "telemetry_stale_s",
            "telemetry_hz",
            "reconnect_initial_s",
            "reconnect_max_s",
            "cleanup_timeout_s",
            "receive_budget_ms",
        ):
            number(getattr(o, k), k, 0.001)
        if o.setpoints_enabled and o.action_timeout_s <= 1.2:
            raise ValueError("setpoints require action_timeout_s > 1.2 for PX4 stream priming and mode acknowledgement")
        if o.reconnect_max_s < o.reconnect_initial_s:
            raise ValueError("reconnect_max_s must be >= reconnect_initial_s")
        for k in ("max_messages", "event_queue_size", "dedup_size"):
            number(getattr(o, k), k, 1, 100000, True)
        topics = [
            o.telemetry_topic,
            o.status_topic,
            o.command_topic,
            o.result_topic,
            o.setpoint_topic,
            o.stream_status_topic,
            o.navigation_topic,
        ]
        for topic in topics:
            text(topic, "topic")
        if len(set(topics)) != len(topics):
            raise ValueError("flight topics must be distinct")
        c = o.connection
        if o.backend == "fake":
            if c != {}:
                raise ValueError("fake connection must be {}")
        else:
            if not isinstance(c, dict):
                raise ValueError("connection must be a mapping")
            choice(c.get("transport"), ("udpin", "udpout", "tcp", "serial"), "transport")
            if c["transport"] == "serial":
                if set(c) != {"transport", "device", "baud"}:
                    raise ValueError("serial requires transport, device and baud")
                text(c["device"], "device")
                if not c["device"].startswith("/") or ":" in c["device"]:
                    raise ValueError("serial device must be an absolute Linux path without ':'")
                number(c["baud"], "baud", 1, 4000000, True)
            else:
                if set(c) != {"transport", "host", "port"}:
                    raise ValueError("network connection requires transport, host and port")
                text(c["host"], "host")
                if any(ch in c["host"] for ch in ":/ \t\n"):
                    raise ValueError("host must be an IPv4 address or hostname")
                number(c["port"], "port", 1, 65535, True)
        if o.server is not None:
            s = o.server
            if o.backend != "mavsdk" or not isinstance(s, dict) or set(s) - {"host", "port", "executable"}:
                raise ValueError("server is only valid for MAVSDK; allowed keys: host, port, executable")
            if "host" in s:
                text(s["host"], "server.host")
                if "executable" in s or "port" not in s:
                    raise ValueError("external server needs host and port, without executable")
            if "executable" in s:
                text(s["executable"], "server.executable")
            if "port" in s:
                number(s["port"], "server.port", 1, 65535, True)
        return o

    def address(self):
        c = self.connection
        if c["transport"] == "serial":
            return f"serial://{c['device']}:{c['baud']}" if self.backend == "mavsdk" else c["device"]
        transport = c["transport"]
        if self.backend == "mavsdk":
            transport = "tcpout" if transport == "tcp" else transport
            return f"{transport}://{c['host']}:{c['port']}"
        return f"{transport}:{c['host']}:{c['port']}"
