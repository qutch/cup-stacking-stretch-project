"""config/sim.yaml and config/robot.yaml (roadmap 0.2) must set every parameter the nodes
read, with the right types, and robot.yaml must not carry sim-only workarounds."""

import os

import pytest
import yaml

from stack_cups import params

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")


def load(name):
    with open(os.path.join(CONFIG, name)) as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]


@pytest.mark.parametrize("name", ["sim.yaml", "robot.yaml"])
def test_every_parameter_is_set_and_no_unknown_ones(name):
    values = load(name)
    assert set(values) == set(params.ALL), (
        f"missing: {sorted(set(params.ALL) - set(values))}, "
        f"unknown: {sorted(set(values) - set(params.ALL))}")


@pytest.mark.parametrize("name", ["sim.yaml", "robot.yaml"])
def test_types_match_the_code_defaults(name):
    for key, value in load(name).items():
        default = params.ALL[key]
        if isinstance(default, bool):
            assert isinstance(value, bool), key
        elif isinstance(default, float):
            # ROS rejects an int for a double parameter declared with a float default
            assert isinstance(value, float), f"{key}: write {value} as {float(value)}"
        else:
            assert type(value) is type(default), key


def test_code_defaults_are_the_sim_values():
    assert load("sim.yaml") == params.ALL


def test_robot_values_are_slow_and_without_sim_workarounds():
    sim, robot = load("sim.yaml"), load("robot.yaml")
    assert robot["joint_tries"] == 1 and robot["aborted_ok_tol"] == 0.0
    assert robot["step_confirm"] is True
    assert robot["max_base_speed"] <= sim["max_base_speed"]
    assert robot["v_max"] <= robot["max_base_speed"] and robot["v_max"] <= sim["v_max"]
    assert robot["max_joint_speed_m"] > 0.0 and robot["max_joint_speed_rad"] > 0.0


def test_both_files_use_the_same_cup():
    sim, robot = load("sim.yaml"), load("robot.yaml")
    assert {k: v for k, v in sim.items() if k.startswith("cup_")} == \
        {k: v for k, v in robot.items() if k.startswith("cup_")}
