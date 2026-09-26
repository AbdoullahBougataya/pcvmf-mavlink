import asyncio
import time

import pytest

pytest.importorskip("pymavlink")

from vehicle_peer import VehiclePeer

from pcvmf_mavlink.backends.pymavlink import PymavlinkBackend
from pcvmf_mavlink.messages import ActionRequest, VehicleTelemetry, VehicleTelemetryCodec
from pcvmf_mavlink.options import Options


@pytest.mark.parametrize(
    "backend,firmware,transport",
    [
        ("pymavlink", "px4", "udpin"),
        ("pymavlink", "arducopter", "udpin"),
        ("pymavlink", "px4", "udpout"),
        ("pymavlink", "px4", "tcp"),
        ("pymavlink", "px4", "serial"),
        ("mavsdk", "px4", "udpin"),
        ("mavsdk", "px4", "tcp"),
        ("mavsdk", "px4", "serial"),
    ],
)
def test_real_transport_telemetry_and_actions(backend, firmware, transport):
    if backend == "mavsdk":
        pytest.importorskip("mavsdk")

    async def scenario():
        from pcvmf_mavlink.backends import create

        peer = VehiclePeer(firmware)
        connection = await peer.open(transport)
        adapter = create(Options.parse({"backend": backend, "firmware": firmware, "connection": connection}))
        polling = None
        polling_active = True
        try:
            await asyncio.wait_for(adapter.connect(), 12)

            async def poll():
                while polling_active:
                    await adapter.poll()

            polling = asyncio.create_task(poll())
            end = time.monotonic() + 3
            while not {"position", "state", "attitude", "battery", "gps", "landed"} <= set(adapter.samples):
                if polling.done():
                    await polling
                assert time.monotonic() < end, adapter.samples
                await asyncio.sleep(0.02)
            payload = VehicleTelemetry("s", 1, 1, dict(adapter.samples))
            assert VehicleTelemetryCodec().decode(VehicleTelemetryCodec().encode(payload)) == payload
            assert adapter.samples["position"].values["latitude_deg"] == pytest.approx(47.397742)
            assert adapter.samples["attitude"].values["roll_rad"] == pytest.approx(0.1, rel=1e-5)
            for index, action in enumerate(
                ["arm", "disarm", "arm", "takeoff", "land", "takeoff", "return_to_launch", "disarm"]
            ):
                # Observe heartbeat state before asking takeoff to check it.
                await asyncio.sleep(0.1)
                request = ActionRequest(str(index), "s", action, time.time() + 8, 2 if action == "takeoff" else None)
                result = await asyncio.wait_for(adapter.execute(request), 8)
                assert result[0] == "accepted", result
            if polling.done():
                await polling
            sent_actions = [m for m in peer.commands if m.command in (20, 21, 22, 176, 400)]
            assert len(sent_actions) >= 8
            takeoff = next(m for m in peer.commands if m.command == 22)
            if backend == "pymavlink":
                assert takeoff.param7 == (490 if firmware == "px4" else 2)
        finally:
            polling_active = False
            if polling:
                polling.cancel()
                await asyncio.gather(polling, return_exceptions=True)
            await adapter.close()
            await peer.close()
        if backend == "mavsdk":
            assert adapter.process.returncode is not None
            assert all(task.done() for task in adapter.tasks)

    asyncio.run(scenario())


def test_takeoff_without_arm_never_transmits():
    async def scenario():
        peer = VehiclePeer()
        adapter = PymavlinkBackend(
            Options.parse({"backend": "pymavlink", "firmware": "px4", "connection": await peer.open()})
        )
        try:
            await asyncio.wait_for(adapter.connect(), 2)
            with pytest.raises(ValueError, match="explicit arming"):
                await adapter.execute(ActionRequest("r", "s", "takeoff", time.time() + 5, 2))
            assert not any(m.command == 22 for m in peer.commands)
        finally:
            await adapter.close()
            await peer.close()

    asyncio.run(scenario())


def test_wrong_firmware_fails_connection():
    async def scenario():
        peer = VehiclePeer("arducopter")
        adapter = PymavlinkBackend(
            Options.parse({"backend": "pymavlink", "firmware": "px4", "connection": await peer.open()})
        )
        try:
            with pytest.raises(ValueError, match="expected px4"):
                await asyncio.wait_for(adapter.connect(), 2)
        finally:
            await adapter.close()
            await peer.close()

    asyncio.run(scenario())
