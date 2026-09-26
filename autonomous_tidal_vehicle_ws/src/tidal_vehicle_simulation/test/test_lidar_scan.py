"""Unit tests for the LiDAR terrain-return filter."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "lidar_scan_node.py"
SPEC = importlib.util.spec_from_file_location("lidar_scan_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
cloud_to_ranges = MODULE.cloud_to_ranges
height_above_corridor_ground = MODULE.height_above_corridor_ground


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
        [-0.99, 1.27, -0.43],   # V3 outboard corner: inside 1.85 m envelope
        [1.86, 0.0, -0.42],     # 1 cm outside the verified V3 envelope
    ])

    ranges, angle_min, increment = cloud_to_ranges(
        points,
        bins=720,
        min_h=-1.00,
        max_h=0.50,
        rmin=0.3,
        rmax=30.0,
        self_box=(-1.56, 1.56, -0.95, 0.95),
        self_footprint_radius=1.85,
    )

    forward_index = int((0.0 - angle_min) / increment)
    assert ranges[forward_index] == 1.86
    assert np.count_nonzero(np.isfinite(ranges)) == 1


def test_v3_filter_keeps_mapped_obstacles_and_rejects_terrain_returns() -> None:
    """V3 keeps low collision geometry, not ground or overhead visual canopy."""
    points = np.array([
        [4.0, 0.0, -1.45],    # low rock/root collision geometry
        [0.0, 6.0, -0.70],    # mangrove trunk collision proxy
        [-8.0, 0.0, -0.20],   # terrain/slope return
        [7.0, 0.0, 0.20],     # visual-only overhanging canopy
    ])

    ranges, _, _ = cloud_to_ranges(
        points,
        bins=720,
        min_h=-1.70,
        max_h=0.50,
        rmin=0.3,
        rmax=30.0,
        self_box=(-1.56, 1.56, -0.95, 0.95),
        self_radius=1.85,
        height_above_ground=np.array([0.80, 1.10, 0.05, 1.90]),
        ground_clearance=0.30,
        ground_obstacle_height=1.30,
    )

    assert sorted(ranges[np.isfinite(ranges)]) == [4.0, 6.0]


def test_corridor_ground_filter_handles_uphill_returns() -> None:
    """World-aware filtering rejects slope points even when lidar-frame z is high."""
    points = np.array([
        [-2.4, 0.0, 0.45],  # raised terrain behind a vehicle descending the bank
        [4.0, 0.0, 0.10],   # obstacle protruding above the surface ahead
    ])
    # A synthetic base pose chosen so the first transformed point lies on the
    # corridor profile.  Feed explicit heights to isolate scan filtering.
    above_ground = np.array([0.05, 0.80])

    ranges, _, _ = cloud_to_ranges(
        points,
        bins=720,
        min_h=-0.35,
        max_h=0.50,
        rmin=0.3,
        rmax=30.0,
        self_box=(-1.56, 1.56, -0.95, 0.95),
        self_radius=1.85,
        height_above_ground=above_ground,
        ground_clearance=0.30,
        ground_obstacle_height=1.30,
    )

    assert sorted(ranges[np.isfinite(ranges)]) == [4.0]


def test_corridor_ground_height_transform_uses_vehicle_pose() -> None:
    points = np.array([[0.0, 0.0, -1.82], [0.0, 0.0, -0.82]])
    heights = height_above_corridor_ground(
        points,
        base_position=(52.0, 0.0, -2.75),
        base_orientation=(0.0, 0.0, 0.0, 1.0),
        lidar_height=1.82,
    )

    assert heights == pytest.approx([0.25, 1.25])


def test_corridor_ground_height_transform_ignores_nonfinite_points() -> None:
    heights = height_above_corridor_ground(
        np.array([[0.0, 0.0, -0.82], [np.inf, 0.0, 0.0]]),
        base_position=(52.0, 0.0, -2.75),
        base_orientation=(0.0, 0.0, 0.0, 1.0),
        lidar_height=1.82,
    )

    assert heights[0] == pytest.approx(1.25)
    assert np.isinf(heights[1])
