from math import pi

import pytest

from tidal_vehicle_autonomy.path_follower_core import (
    compute_command,
    FollowerConfig,
    Pose2D,
)


def test_drives_forward_on_a_straight_path() -> None:
    command = compute_command(
        Pose2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        FollowerConfig(),
    )

    assert command.linear_x == pytest.approx(0.8)
    assert command.angular_z == pytest.approx(0.0)
    assert command.target_index == 1


def test_rotates_before_driving_when_path_is_to_the_side() -> None:
    command = compute_command(
        Pose2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (0.0, 2.0)],
        FollowerConfig(),
    )

    assert command.linear_x == 0.0
    assert command.angular_z == pytest.approx(1.0)


def test_slows_down_near_the_goal() -> None:
    command = compute_command(
        Pose2D(0.0, 0.0, 0.0),
        [(0.0, 0.0), (0.5, 0.0)],
        FollowerConfig(goal_tolerance=0.1, slow_down_distance=1.0),
    )

    assert command.linear_x == pytest.approx(0.4)
    assert command.angular_z == pytest.approx(0.0)


def test_stops_inside_goal_tolerance() -> None:
    command = compute_command(
        Pose2D(0.9, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0)],
        FollowerConfig(goal_tolerance=0.2),
    )

    assert command.goal_reached
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0


def test_heading_wrap_uses_the_short_turn() -> None:
    command = compute_command(
        Pose2D(0.0, 0.0, pi - 0.05),
        [(0.0, 0.0), (-1.0, -0.05)],
        FollowerConfig(),
    )

    assert 0.0 < command.angular_z < 0.2


def test_progress_does_not_move_backwards() -> None:
    command = compute_command(
        Pose2D(0.1, 0.0, 0.0),
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        FollowerConfig(),
        progress_index=2,
    )

    assert command.progress_index >= 2
    assert command.target_index >= 2
