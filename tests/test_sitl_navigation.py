"""Explicit opt-in: arm/fly a dedicated disposable SITL vehicle only."""

import math
import os
import queue
import time
import uuid
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from pcvmf_mavlink.messages import ActionRequest, ActionResult
from pcvmf_mavlink.navigation import ControlRequest, ControlResult, Setpoint
from pcvmf_mavlink.options import Options
from pcvmf_mavlink.service import FlightService
from pcvmf_mavlink.transactions import MissionItem, MissionRequest, MissionResult


@pytest.mark.sitl
def test_sitl_stream_motion_expiry_and_mission_control():
    path = os.environ.get("PCVMF_SITL_NAVIGATION_CONFIG")
    if not path:
        pytest.skip("set PCVMF_SITL_NAVIGATION_CONFIG for a dedicated running flight simulator")
    raw = yaml.safe_load(Path(path).read_text())
    o = Options.parse(
        {
            **raw["workers"][0]["plugin"]["options"],
            "commands_enabled": True,
            "missions_enabled": True,
            "setpoints_enabled": True,
            "mission_execution_enabled": True,
            "action_timeout_s": 8,
        }
    )
    service = FlightService(o)
    current_point = None
    sequence = 0

    def refresh():
        nonlocal sequence
        if current_point is not None:
            sequence += 1
            service.submit_setpoint(replace(current_point, sequence=sequence, expires_at=time.time() + 0.5), "sitl")

    def wait(predicate, timeout=15):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            service.check()
            refresh()
            if predicate():
                return
            time.sleep(0.03)
        raise AssertionError(f"SITL state not observed: {service.snapshot()}, {service.navigation_snapshot()}")

    def values(group, navigation=False):
        payload = service.navigation_snapshot()[1] if navigation else service.snapshot()[1]
        if payload is None or group not in payload.samples or payload.samples[group].stale:
            return {}
        return payload.samples[group].values

    def transact(request):
        service.submit(request, "sitl")
        end = time.monotonic() + 12
        while time.monotonic() < end:
            service.check()
            refresh()
            try:
                event = service.events.get(timeout=0.03)
            except queue.Empty:
                continue
            if (
                isinstance(event, (ActionResult, ControlResult, MissionResult))
                and event.request_id == request.request_id
            ):
                assert event.outcome == "accepted", event
                return
        raise AssertionError("SITL transaction result missing")

    try:
        service.start()
        session = service.snapshot()[0].session_id

        def action(name):
            transact(
                ActionRequest(str(uuid.uuid4()), session, name, time.time() + 10, 3 if name == "takeoff" else None)
            )

        def control(name, **extra):
            transact(ControlRequest(str(uuid.uuid4()), session, name, time.time() + 10, **extra))

        # Allow the estimator and home position to settle without bypassing checks.
        ready_after = time.monotonic() + 15
        wait(lambda: time.monotonic() >= ready_after and bool(values("local", True)), 30)
        action("arm")
        wait(lambda: values("state").get("armed") is True)
        action("takeoff")
        wait(lambda: values("position").get("altitude_relative_home_m", 0) > 2.7, 30)
        local = values("local", True)
        north, east, down = local["north_m"], local["east_m"], local["down_m"]
        current_point = Setpoint(
            session, str(uuid.uuid4()), 0, time.time() + 0.5, 0, position_ned_m=[north + 2, east, down]
        )
        control("stream_start", stream_id=current_point.stream_id, initial_setpoint=current_point)
        wait(lambda: values("local", True).get("north_m", north) > north + 1, 20)
        east = values("local", True)["east_m"]
        current_point = replace(current_point, position_ned_m=None, velocity_ned_m_s=[0, 0.5, 0])
        wait(lambda: values("local", True).get("east_m", east) > east + 0.4, 10)
        current_point = None
        wait(lambda: service.navigation_snapshot()[0].state == "expired", 5)
        wait(lambda: values("state").get("flight_mode") in ("HOLD", "LOITER"), 5)
        position = values("position")
        lat, lon = round(position["latitude_deg"] * 1e7), round(position["longitude_deg"] * 1e7)
        items = [
            MissionItem(6, 22, lat, lon, 3, current=1),
            MissionItem(6, 16, lat + 500, lon, 3, param1=10),
            MissionItem(6, 21, lat, lon, 0),
        ]
        waypoint_index = 1
        if o.firmware == "arducopter":
            items = [MissionItem(0, 16, lat, lon, position["altitude_msl_m"], current=1)] + [
                replace(i, current=0) for i in items
            ]
            waypoint_index = 2
        transact(MissionRequest(str(uuid.uuid4()), session, "upload", time.time() + 30, items))
        # Transfer ACK precedes navigator feasibility/state propagation on PX4.
        if o.firmware == "px4":
            wait(lambda: values("mission", True).get("total_items") == len(items), 10)
        control("mission_start")
        wait(lambda: values("state").get("flight_mode") in ("AUTO", "MISSION"))
        wait(lambda: (values("mission", True).get("current_index") or 0) >= waypoint_index)
        control("mission_pause")
        wait(
            lambda: math.hypot(values("local", True).get("north_m_s", 100), values("local", True).get("east_m_s", 100))
            < 0.3,
            15,
        )
        control("mission_set_current", mission_index=waypoint_index)
        wait(lambda: values("mission", True).get("current_index") == waypoint_index)
        action("land")
        wait(lambda: values("landed").get("landed_state") == "ON_GROUND", 40)
        action("disarm")
        wait(lambda: values("state").get("armed") is False)
    finally:
        service.close()
