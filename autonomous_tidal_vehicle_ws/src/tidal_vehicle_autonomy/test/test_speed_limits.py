"""Zone and curvature speed limits for higher-speed vehicles (Version 3).

Added by Person 4 with the split cost band (AGENTS.md, *Shared terrain-cost
semantics*); the follower only uses them when given zone limits."""
from math import pi

from tidal_vehicle_autonomy.motion_limits import (
    sensor_limited_speed,
    speed_aware_sensor_range,
    stopping_distance,
)
from tidal_vehicle_autonomy.path_follower_core import (
    curvature_speed_limit,
    lateral_accel_yaw_rate_limit,
    path_points_ahead,
    Pose2D,
    stopping_reach,
    upcoming_curve_speed_limit,
    zone_speed_limit,
)

LIMITS = [4.2, 13.9, 2.8, 2.8]          # firm, open surveyed water, mud/roots/debris, elevated


def test_zone_limit_uses_the_slowest_cell() -> None:
    assert zone_speed_limit([25, 22, 28], LIMITS) == 13.9    # open surveyed water
    assert zone_speed_limit([25, 35], LIMITS) == 2.8         # roots/debris ahead
    assert zone_speed_limit([5], LIMITS) == 4.2              # firm shore
    assert zone_speed_limit([95, -1], LIMITS) is None        # no-go / unknown: planner's job
    assert zone_speed_limit([25], []) is None                # off (Version 2)


def test_shared_stopping_distance_supports_a_zero_minimum() -> None:
    assert abs(stopping_distance(4.0, 1.0) - 12.0) < 1e-9
    assert stopping_distance(0.0, 1.0) == 0.0


def test_sensor_limited_speed_stops_within_the_available_range() -> None:
    speed = sensor_limited_speed(30.0, 1.0, 1.0)
    assert abs(speed - 6.8102496759) < 1e-9
    assert abs(stopping_distance(speed, 1.0, 1.0) - 30.0) < 1e-9


def test_speed_aware_sensor_range_grows_and_clamps() -> None:
    assert speed_aware_sensor_range(0.0, 8.0, 30.0, 1.0) == 8.0
    assert speed_aware_sensor_range(4.0, 8.0, 30.0, 1.0, clearance_m=2.0) == 14.0
    assert speed_aware_sensor_range(4.0, 8.0, 30.0, 1.0) == 12.0
    assert speed_aware_sensor_range(13.9, 8.0, 30.0, 1.0) == 30.0


def test_stopping_reach_grows_with_speed() -> None:
    assert stopping_reach(0.0, 1.5) == 2.0
    # 50 km/h: 13.9 m reaction + 64 m braking at 1.5 m/s^2
    assert 75.0 < stopping_reach(13.9, 1.5) < 80.0


def test_path_points_ahead_stop_at_reach() -> None:
    path = [(float(x), 0.0) for x in range(0, 101, 10)]
    points = path_points_ahead(Pose2D(0.0, 0.0, 0.0), path, 0, 20.0)
    assert points[0] == (0.0, 0.0)
    assert max(p[0] for p in points) <= 30.0
    assert min(p[0] for p in points) >= 0.0


def test_curvature_limit() -> None:
    assert curvature_speed_limit(0.0, 5.0, 1.0) is None            # straight: no limit
    v = curvature_speed_limit(pi / 6, 5.0, 1.0)                    # k = 0.2 /m
    assert abs(v - (1.0 / 0.2) ** 0.5) < 1e-9
    assert curvature_speed_limit(pi / 6, 5.0, 0.0) is None         # off (Version 2)


def test_lateral_acceleration_caps_yaw_rate_while_moving() -> None:
    assert abs(lateral_accel_yaw_rate_limit(2.8, 1.0) - (1.0 / 2.8)) < 1e-9
    assert lateral_accel_yaw_rate_limit(0.0, 1.0) is None
    assert lateral_accel_yaw_rate_limit(2.8, 0.0) is None


def test_upcoming_curve_preview_does_not_limit_a_straight_route() -> None:
    points = [(float(x), 0.0) for x in range(10)]

    assert upcoming_curve_speed_limit(
        points, 1.0, 2.0, 0.7, 1.0, 4.0, 1.0
    ) is None


def test_upcoming_curve_preview_slows_before_a_sharp_turn() -> None:
    near_corner = [
        (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0),
        (3.0, 1.0), (3.0, 2.0), (3.0, 3.0),
    ]
    far_corner = [
        (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0),
        (4.0, 0.0), (5.0, 0.0), (6.0, 0.0), (7.0, 0.0),
        (8.0, 0.0), (8.0, 1.0), (8.0, 2.0), (8.0, 3.0),
    ]

    near_limit = upcoming_curve_speed_limit(
        near_corner, 1.0, 2.0, 0.7, 1.0, 4.0, 1.0
    )
    far_limit = upcoming_curve_speed_limit(
        far_corner, 1.0, 2.0, 0.7, 1.0, 4.0, 1.0
    )

    assert near_limit is not None
    assert far_limit is not None
    assert near_limit < far_limit
    assert near_limit < 2.0
