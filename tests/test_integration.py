import json
import multiprocessing
from pathlib import Path

import pytest
import yaml
from pcvmf.config import load_config, parse_config
from pcvmf.runtime import Application

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("path", sorted((ROOT / "examples").glob("*.yaml")))
def test_example_configuration(path):
    load_config(path)


def test_spawned_typed_messages_commands_and_cleanup(tmp_path):
    raw = yaml.safe_load((ROOT / "examples/fake.yaml").read_text())
    raw["logging"]["mcap"]["directory"] = str(tmp_path / "recordings")
    output = tmp_path / "received.json"
    raw["workers"][1]["plugin"]["options"]["output"] = str(output)
    children = {p.pid for p in multiprocessing.active_children()}
    result = Application(parse_config(raw)).run()
    assert result.exit_code == 0, result.errors
    assert result.ready
    assert json.loads(output.read_text())["actions_observed"] == ["arm", "takeoff", "land", "disarm"]
    assert {p.pid for p in multiprocessing.active_children()} == children
    assert Path(result.recording_path).stat().st_size > 0


def test_spawned_mission_parameter_roundtrip(tmp_path):
    raw = yaml.safe_load((ROOT / "examples/fake-transactions.yaml").read_text())
    raw["logging"]["mcap"]["directory"] = str(tmp_path / "recordings")
    output = tmp_path / "received.json"
    raw["workers"][1]["plugin"]["options"] = {"output": str(output)}
    children = {p.pid for p in multiprocessing.active_children()}
    result = Application(parse_config(raw)).run()
    assert result.exit_code == 0, result.errors
    assert result.ready
    assert json.loads(output.read_text())["transactions_observed"] == [
        "upload",
        "download",
        "clear",
        "download_empty",
        "get_int",
        "set_int",
        "get_int_again",
        "get_float",
        "set_float",
        "get_float_again",
    ]
    assert {p.pid for p in multiprocessing.active_children()} == children
    assert Path(result.recording_path).stat().st_size > 0


def test_spawned_navigation_observed_and_cleaned_up(tmp_path):
    raw = yaml.safe_load((ROOT / "examples/fake-navigation.yaml").read_text())
    raw["logging"]["mcap"]["directory"] = str(tmp_path / "recordings")
    output = tmp_path / "observed.json"
    raw["workers"][1]["plugin"]["options"] = {"output": str(output)}
    children = {p.pid for p in multiprocessing.active_children()}
    result = Application(parse_config(raw)).run()
    assert result.exit_code == 0, result.errors
    assert result.ready
    assert json.loads(output.read_text())["navigation_observed"] == [
        "upload",
        "arm",
        "takeoff",
        "stream_start",
        "stream_stop",
        "mission_start",
        "mission_pause",
        "mission_set_current",
        "land",
        "disarm",
    ]
    assert {p.pid for p in multiprocessing.active_children()} == children
    assert Path(result.recording_path).stat().st_size > 0
