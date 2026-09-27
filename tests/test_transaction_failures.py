import asyncio
import time
from contextlib import asynccontextmanager
from dataclasses import replace

import pytest

from pcvmf_mavlink.backends.fake import FakeBackend
from pcvmf_mavlink.options import Options
from pcvmf_mavlink.service import FlightService
from pcvmf_mavlink.transactions import MissionRequest, ParameterRequest


@asynccontextmanager
async def connected_peer(**options):
    pytest.importorskip("pymavlink")
    from transaction_peer import TransactionPeer

    from pcvmf_mavlink.backends.pymavlink import PymavlinkBackend

    peer = TransactionPeer()
    adapter = PymavlinkBackend(
        Options.parse({"backend": "pymavlink", "firmware": "px4", "connection": await peer.open(), **options})
    )
    polling = None
    try:
        await asyncio.wait_for(adapter.connect(), 2)

        async def poll():
            while True:
                await adapter.poll()

        polling = asyncio.create_task(poll())
        yield peer, adapter
        if polling.done():
            await polling
    finally:
        if polling:
            polling.cancel()
            await asyncio.gather(polling, return_exceptions=True)
        await adapter.close()
        await peer.close()


def test_unconfirmed_parameter_write_is_unknown_and_not_retried():
    async def scenario():
        async with connected_peer(parameters_enabled=True, parameter_writes_enabled=True, parameter_timeout_s=0.15) as (
            peer,
            adapter,
        ):
            peer.ignore_sets = True
            result = await FlightService(adapter.options)._execute(
                adapter, ParameterRequest("r", "s", "set", time.time() + 5, "TEST_INT", "int", 99), "producer"
            )
            assert result.outcome == "outcome_unknown"
            assert result.value is None
            assert len([m for m in peer.transactions if m.get_type() == "PARAM_SET"]) == 1
            assert adapter.transaction_inbox is None
            assert peer.params["TEST_INT"] == (6, 42)

    asyncio.run(scenario())


def test_wrong_mission_recipient_is_ignored_and_cancelled():
    async def scenario():
        async with connected_peer() as (peer, adapter):
            peer.wrong_address = True
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(
                    adapter.transfer_mission(MissionRequest("r", "s", "clear", time.time() + 5)), 0.1
                )
            await asyncio.sleep(0.05)
            assert any(m.get_type() == "MISSION_ACK" and m.type == 15 for m in peer.transactions)
            assert adapter.transaction_inbox is None

    asyncio.run(scenario())


def test_oversized_download_stops_before_requesting_items():
    async def scenario():
        async with connected_peer(max_mission_items=1) as (peer, adapter):
            peer.items = [{}, {}]
            with pytest.raises(ValueError, match="max_mission_items"):
                await adapter.transfer_mission(MissionRequest("r", "s", "download", time.time() + 5))
            assert not any(m.get_type() == "MISSION_REQUEST_INT" for m in peer.transactions)
            assert adapter.transaction_inbox is None

    asyncio.run(scenario())


def test_mavsdk_parameter_component_limit_is_structural():
    with pytest.raises(ValueError, match="component 1"):
        Options.parse(
            {
                "backend": "mavsdk",
                "firmware": "px4",
                "target_component": 2,
                "parameters_enabled": True,
                "connection": {"transport": "udpin", "host": "127.0.0.1", "port": 14540},
            }
        )


def test_expired_and_disabled_requests_never_reach_adapter():
    class Never(FakeBackend):
        async def parameter(self, request):
            raise AssertionError("disabled or expired request reached adapter")

        transfer_mission = parameter

    async def scenario():
        o = Options.parse({"backend": "fake", "firmware": "px4", "connection": {}})
        for request in (
            MissionRequest("r", "s", "clear", time.time() + 5),
            ParameterRequest("r", "s", "set", time.time() + 5, "TEST_INT", "int", 2),
        ):
            result = await FlightService(o)._execute(Never(o), request, "producer")
            assert result.outcome == "disabled"
            enabled = replace(o, missions_enabled=True, parameters_enabled=True, parameter_writes_enabled=True)
            result = await FlightService(enabled)._execute(Never(enabled), replace(request, expires_at=0), "producer")
            assert result.outcome == "expired"

    asyncio.run(scenario())
