"""Unit tests for the ROS-independent helpers of collision_monitor_node.py."""
import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "collision_monitor_node.py"
SPEC = importlib.util.spec_from_file_location("collision_monitor_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_side_of_faces():
    assert MODULE.side_of(1.4, 0.1) == "front"
    assert MODULE.side_of(-1.4, -0.1) == "rear"
    assert MODULE.side_of(0.2, 0.85) == "left"
    assert MODULE.side_of(0.2, -0.85) == "right"


def test_side_of_corner_uses_normalised_distance():
    # 1.2 m forward is 80 % of the half-length; 0.8 m left is 89 % of the half-beam
    assert MODULE.side_of(1.2, 0.8) == "left"


def test_ground_contact_is_ignored():
    assert MODULE.is_ground_normal(0.99)       # flat ground
    assert MODULE.is_ground_normal(-0.95)      # normal pointing the other way
    assert MODULE.is_ground_normal(0.94)       # 20 deg slope
    assert not MODULE.is_ground_normal(0.1)    # a wall or a root: a real hit


def test_jolt_side_from_deceleration():
    assert MODULE.jolt_side(-9.0, 0.0) == "front"    # hit on the bow decelerates
    assert MODULE.jolt_side(9.0, 0.0) == "rear"
    assert MODULE.jolt_side(0.0, -9.0) == "left"


def test_model_of_scoped_names():
    assert MODULE.model_of("hovercraft_v3::base_link::collision") == "hovercraft_v3"
    assert MODULE.model_of("mangrove_03::link::collision") == "mangrove_03"
    assert MODULE.model_of("") == ""
