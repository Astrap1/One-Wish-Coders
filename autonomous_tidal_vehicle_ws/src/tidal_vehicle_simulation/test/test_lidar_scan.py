"""Unit tests for the LiDAR terrain-return filter."""

import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts" / "lidar_scan_node.py"
SPEC = importlib.util.spec_from_file_location("lidar_scan_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
cloud_to_ranges = MODULE.cloud_to_ranges


def test_ground_returns_are_rejected_but_tall_obstacles_remain() -> None:
    """The 2D scan must not turn the low terrain surface into an obstacle ring."""
    points = np.array([
        [4.0, 0.0, -0.80],  # decorative ground / shallow-water surface
        [5.0, 0.0, -0.50],  # rock, root, or trunk above the rejection band
        [6.0, 0.0, 0.70],   # canopy: above the local-navigation band
    ])

    ranges, angle_min, increment = cloud_to_ranges(
        points,
        bins=360,
        min_h=-0.62,
        max_h=0.50,
        rmin=0.3,
        rmax=30.0,
        self_box=(-1.32, 1.30, -0.78, 0.78),
    )

    forward_index = int((0.0 - angle_min) / increment)
    assert ranges[forward_index] == 5.0


def test_vehicle_body_returns_are_rejected() -> None:
    points = np.array([
        [0.5, 0.0, -0.50],  # within the hovercraft footprint
        [2.0, 0.0, -0.50],  # external obstacle
    ])

    ranges, angle_min, increment = cloud_to_ranges(
        points,
        bins=360,
        min_h=-0.62,
        max_h=0.50,
        rmin=0.3,
        rmax=30.0,
        self_box=(-1.32, 1.30, -0.78, 0.78),
    )

    forward_index = int((0.0 - angle_min) / increment)
    assert ranges[forward_index] == 2.0


def test_self_filter_includes_box_edges_and_v3_corner_envelope() -> None:
    """V3 must not plan around its own skirt/track corner returns."""
    points = np.array([
        [-1.56, -0.11, -0.42],  # old V3 box edge: now inclusive
        [-0.99, 1.27, -0.43],   # V3 outboard corner: inside 1.75 m envelope
        [1.76, 0.0, -0.42],     # 1 cm outside the V3 envelope: real obstacle
    ])

    ranges, angle_min, increment = cloud_to_ranges(
        points,
        bins=720,
        min_h=-1.00,
        max_h=0.50,
        rmin=0.3,
        rmax=30.0,
        self_footprint_radius=1.75,
    )

    forward_index = int((0.0 - angle_min) / increment)
    assert ranges[forward_index] == 1.76
