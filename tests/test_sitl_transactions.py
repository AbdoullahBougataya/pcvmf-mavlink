"""Opt-in destructive mission round trip against a dedicated simulator only."""

import asyncio
import os
import time
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from pcvmf_mavlink.backends import create
from pcvmf_mavlink.options import Options
from pcvmf_mavlink.transactions import MissionItem, MissionRequest, ParameterRequest


@pytest.mark.sitl
def test_simulator_mission_parameter_roundtrip():
    path = os.environ.get("PCVMF_SITL_TRANSACTIONS_CONFIG")
    if not path:
        pytest.skip("set PCVMF_SITL_TRANSACTIONS_CONFIG for a dedicated running simulator")
    raw = yaml.safe_load(Path(path).read_text())
    config = Options.parse(raw["workers"][0]["plugin"]["options"])

    async def scenario():
        adapter = create(config)
        polling = None
        try:
            await asyncio.wait_for(adapter.connect(), 15)

            async def poll():
                while True:
                    await adapter.poll()

            polling = asyncio.create_task(poll())
            items = [
                MissionItem(6, 22, 473977420, 85455940, 5, current=1),
                MissionItem(6, 16, 473977520, 85455940, 5),
                MissionItem(6, 21, 473977420, 85455940, 0),
            ]
            if config.firmware == "arducopter":
                items = [MissionItem(0, 16, 473977420, 85455940, 488, current=1)] + [
                    replace(x, current=0) for x in items
                ]
            for index, operation in enumerate(("upload", "download", "clear", "download")):
                result = await asyncio.wait_for(
                    adapter.transfer_mission(
                        MissionRequest(
                            operation, "sitl", operation, time.time() + 30, items if operation == "upload" else []
                        )
                    ),
                    30,
                )
                assert result[0] == "accepted", (operation, result)
                if operation == "download":
                    if index == 1:
                        assert len(result[2]) == len(items)
                        assert [x.command for x in result[2]] == [x.command for x in items]
                        assert [(x.x, x.y, x.z) for x in result[2][1:]] == [(x.x, x.y, x.z) for x in items[1:]]
                    else:
                        assert result[2] == [] or config.firmware == "arducopter"
            # Read and write the same values: no parameter tuning in this test.
            names = (
                [("SYS_AUTOSTART", "int"), ("MPC_TKO_SPEED", "float")]
                if config.firmware == "px4"
                else [("SYSID_THISMAV", "int"), ("WPNAV_SPEED", "float")]
            )
            for name, kind in names:
                result = await asyncio.wait_for(
                    adapter.parameter(ParameterRequest("read", "sitl", "get", time.time() + 10, name, kind)), 10
                )
                assert result[0] == "accepted", result
                value = result[2]
                result = await asyncio.wait_for(
                    adapter.parameter(ParameterRequest("write", "sitl", "set", time.time() + 10, name, kind, value)), 10
                )
                assert result[0] == "accepted" and result[2] == value, result
            if polling.done():
                await polling
        finally:
            if polling:
                polling.cancel()
                await asyncio.gather(polling, return_exceptions=True)
            await adapter.close()

    asyncio.run(scenario())
