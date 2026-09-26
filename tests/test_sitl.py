"""Opt-in tests against an explicitly supplied running SITL configuration."""

import json
import os
from pathlib import Path

import pytest
import yaml
from pcvmf.config import parse_config
from pcvmf.runtime import Application


@pytest.mark.sitl
def test_sitl_flight_actions_and_observed_state(tmp_path):
    config_path = os.environ.get("PCVMF_SITL_CONFIG")
    if not config_path:
        pytest.skip("set PCVMF_SITL_CONFIG to a running simulator's YAML to opt in")
    raw = yaml.safe_load(Path(config_path).read_text())
    assert len(raw["workers"]) == 1, "provide a single flight worker; the test adds its own command producer"
    flight = raw["workers"][0]
    assert flight["plugin"]["options"]["backend"] != "fake"
    flight["name"] = "flight"
    flight["plugin"]["options"]["commands_enabled"] = True
    flight["subscriptions"] = [{"source": "smoke", "topic": "flight/command", "delivery": "ordered"}]
    output = tmp_path / "sitl-result.json"
    actions = ["arm", "disarm", "arm", "takeoff", "land", "disarm"]
    if os.environ.get("PCVMF_SITL_RTL") == "1":
        actions = ["arm", "takeoff", "return_to_launch", "disarm"]
    raw["workers"].append(
        {
            "name": "smoke",
            "plugin": {
                "class": "pcvmf_mavlink.examples:SmokeWorker",
                "options": {
                    "actions": actions,
                    "timeout_s": 180,
                    "settle_s": 15,
                    "takeoff_altitude_m": 3,
                    "output": str(output),
                },
            },
            "rate_hz": 50,
            "publications": ["flight/command"],
            "subscriptions": [
                {
                    "source": "flight",
                    "topic": "flight/" + topic,
                    "delivery": "ordered" if topic == "result" else "latest",
                }
                for topic in ("status", "telemetry", "result")
            ],
        }
    )
    raw["logging"] = {"mcap": {"directory": str(tmp_path / "recordings")}}
    result = Application(parse_config(raw)).run()
    assert result.exit_code == 0, result.errors
    assert result.ready and json.loads(output.read_text())["actions_observed"] == actions
