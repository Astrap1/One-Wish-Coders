"""ROS-independent lookahead controller used by the path follower node."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin, sqrt
from typing import Sequence

from .motion_limits import stopping_distance

Waypoint = tuple[float, float]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FollowerConfig:
    max_linear_speed: float = 0.8
    max_angular_speed: float = 1.0
    lookahead_distance: float = 0.75
    goal_tolerance: float = 0.25
    heading_gain: float = 1.5
    slow_down_distance: float = 1.0
    rotate_in_place_angle: float = 0.7

    def __post_init__(self) -> None:
        positive_values = {
            "max_linear_speed": self.max_linear_speed,
            "max_angular_speed": self.max_angular_speed,
            "lookahead_distance": self.lookahead_distance,
            "heading_gain": self.heading_gain,
            "slow_down_distance": self.slow_down_distance,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.goal_tolerance < 0.0:
            raise ValueError("goal_tolerance must not be negative")
        if not 0.0 < self.rotate_in_place_angle <= pi:
            raise ValueError("rotate_in_place_angle must be in the range (0, pi]")


@dataclass(frozen=True)
class MotionCommand:
    linear_x: float
    angular_z: float
    target_index: int | None
    progress_index: int
    goal_reached: bool
    target: Waypoint | None = None


def compute_command(
    pose: Pose2D,
    waypoints: Sequence[Waypoint],
    config: FollowerConfig,
    progress_index: int = 0,
) -> MotionCommand:
    """Return a bounded forward and turning proposal for the current path."""
    if not waypoints:
        return MotionCommand(0.0, 0.0, None, 0, False, None)

    last_index = len(waypoints) - 1
    start_index = min(max(progress_index, 0), last_index)
    goal_distance = _distance(pose, waypoints[last_index])
    if goal_distance <= config.goal_tolerance:
        return MotionCommand(
            0.0, 0.0, last_index, last_index, True, waypoints[last_index]
        )

    target, target_index, route_index = _target_along_path(
        pose,
        waypoints,
        start_index,
        config.lookahead_distance,
    )

    target_x, target_y = target
    target_heading = atan2(target_y - pose.y, target_x - pose.x)
    heading_error = normalise_angle(target_heading - pose.yaw)
    angular_z = _clamp(
        config.heading_gain * heading_error,
        -config.max_angular_speed,
        config.max_angular_speed,
    )

    if abs(heading_error) >= config.rotate_in_place_angle:
        linear_x = 0.0
    else:
        distance_scale = min(1.0, goal_distance / config.slow_down_distance)
        heading_scale = max(0.0, cos(heading_error))
        linear_x = config.max_linear_speed * distance_scale * heading_scale

    return MotionCommand(
        linear_x, angular_z, target_index, route_index, False, target
    )


def normalise_angle(angle: float) -> float:
    """Wrap an angle to the range [-pi, pi]."""
    return atan2(sin(angle), cos(angle))


def _distance(pose: Pose2D, waypoint: Waypoint) -> float:
    return hypot(waypoint[0] - pose.x, waypoint[1] - pose.y)


def _target_along_path(
    pose: Pose2D,
    waypoints: Sequence[Waypoint],
    start_index: int,
    lookahead_distance: float,
) -> tuple[Waypoint, int, int]:
    """Project onto the route, then interpolate a target along its arc length.

    The route is a safe corridor centreline, not a list of points the vehicle
    must touch.  Interpolation avoids snapping steering from one grid-cell
    centre to the next, while the caller's speed-scaled lookahead determines
    how far ahead the local trajectory should aim.
    """
    last_index = len(waypoints) - 1
    if last_index == 0 or start_index >= last_index:
        return waypoints[last_index], last_index, last_index

    first_segment = min(max(start_index, 0), last_index - 1)
    projection = waypoints[first_segment]
    projection_segment = first_segment
    best_distance = float("inf")
    for segment_index in range(first_segment, last_index):
        start = waypoints[segment_index]
        end = waypoints[segment_index + 1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1.0e-12:
            candidate = start
        else:
            fraction = (
                (pose.x - start[0]) * dx + (pose.y - start[1]) * dy
            ) / length_squared
            fraction = _clamp(fraction, 0.0, 1.0)
            candidate = (start[0] + fraction * dx, start[1] + fraction * dy)
        distance = hypot(candidate[0] - pose.x, candidate[1] - pose.y)
        if distance < best_distance:
            best_distance = distance
            projection = candidate
            projection_segment = segment_index

    remaining = lookahead_distance
    current = projection
    for target_index in range(projection_segment + 1, len(waypoints)):
        endpoint = waypoints[target_index]
        segment_length = hypot(endpoint[0] - current[0], endpoint[1] - current[1])
        if segment_length >= remaining and segment_length > 1.0e-12:
            fraction = remaining / segment_length
            target = (
                current[0] + fraction * (endpoint[0] - current[0]),
                current[1] + fraction * (endpoint[1] - current[1]),
            )
            return target, target_index, projection_segment
        remaining -= segment_length
        current = endpoint

    return waypoints[last_index], last_index, projection_segment


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


# --------------------------------------------------------------------------
# Speed limits for higher-speed vehicles (Version 3). Off unless the node is
# given zone limits / a lateral-acceleration limit, so Version 2 is unchanged.
# --------------------------------------------------------------------------
def zone_speed_limit(costs: Sequence[int], limits: Sequence[float]) -> float | None:
    """Lowest speed limit (m/s) of the sampled cost cells, using the split cost
    bands in AGENTS.md: limits = [firm 0-19, open surveyed water 20-29,
    mud/shallow water/roots/debris 30-59, elevated risk 60-89]. No-go and
    unknown cells are the planner's job. None when nothing matched."""
    if len(limits) != 4:
        return None
    bands = ((0, 19), (20, 29), (30, 59), (60, 89))
    out = None
    for cost in costs:
        for (low, high), limit in zip(bands, limits):
            if low <= cost <= high:
                out = limit if out is None else min(out, limit)
    return out


def path_points_ahead(
    pose: Pose2D, waypoints: Sequence[Waypoint], start_index: int, reach_m: float,
    step_m: float = 0.5,
) -> list[Waypoint]:
    """Points every step_m along the path, from the vehicle out to reach_m."""
    points: list[Waypoint] = [(pose.x, pose.y)]
    previous = (pose.x, pose.y)
    travelled = 0.0
    for waypoint in waypoints[max(start_index, 0):]:
        segment = hypot(waypoint[0] - previous[0], waypoint[1] - previous[1])
        steps = max(1, int(segment / step_m))
        for k in range(1, steps + 1):
            f = k / steps
            points.append((previous[0] + f * (waypoint[0] - previous[0]),
                           previous[1] + f * (waypoint[1] - previous[1])))
        travelled += segment
        previous = waypoint
        if travelled >= reach_m:
            break
    return points


def stopping_reach(speed: float, brake_decel: float, reaction_s: float = 1.0,
                   minimum_m: float = 2.0) -> float:
    """How far ahead to look for a slower zone: reaction plus braking distance."""
    return stopping_distance(
        max(0.0, speed),
        max(0.1, brake_decel),
        max(0.0, reaction_s),
        max(0.0, minimum_m),
    )


def curvature_speed_limit(heading_error: float, lookahead_m: float,
                          lateral_accel: float) -> float | None:
    """Pure-pursuit curvature k = 2 sin(error) / lookahead; v <= sqrt(a / k)."""
    if lateral_accel <= 0.0 or lookahead_m <= 0.0:
        return None
    curvature = 2.0 * abs(sin(heading_error)) / lookahead_m
    if curvature < 1e-6:
        return None
    return (lateral_accel / curvature) ** 0.5


def upcoming_curve_speed_limit(
    points: Sequence[Waypoint],
    step_m: float,
    sample_distance_m: float,
    lateral_accel_mps2: float,
    brake_decel_mps2: float,
    current_speed_mps: float,
    reaction_time_s: float,
) -> float | None:
    """Return a speed ceiling that begins braking before route curvature.

    Curvature is estimated from three route samples separated by
    ``sample_distance_m``. For each upcoming curve, the ceiling is the speed
    from which the vehicle can decelerate to that curve's lateral-acceleration
    limit over the available distance after reaction travel.
    """
    if (
        len(points) < 3
        or step_m <= 0.0
        or sample_distance_m <= 0.0
        or lateral_accel_mps2 <= 0.0
        or brake_decel_mps2 <= 0.0
    ):
        return None

    stride = max(1, int(round(sample_distance_m / step_m)))
    if len(points) <= 2 * stride:
        return None

    best = None
    reaction_distance = max(0.0, current_speed_mps) * max(0.0, reaction_time_s)
    for centre in range(stride, len(points) - stride):
        first = points[centre - stride]
        middle = points[centre]
        last = points[centre + stride]
        a = hypot(middle[0] - first[0], middle[1] - first[1])
        b = hypot(last[0] - middle[0], last[1] - middle[1])
        c = hypot(last[0] - first[0], last[1] - first[1])
        denominator = a * b * c
        if denominator <= 1.0e-9:
            continue
        cross = abs(
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        curvature = 2.0 * cross / denominator
        if curvature <= 1.0e-6:
            continue
        curve_speed = sqrt(lateral_accel_mps2 / curvature)
        distance_to_curve = centre * step_m
        available = max(0.0, distance_to_curve - reaction_distance)
        approach_speed = sqrt(
            curve_speed * curve_speed + 2.0 * brake_decel_mps2 * available
        )
        best = approach_speed if best is None else min(best, approach_speed)
    return best


def lateral_accel_yaw_rate_limit(
    speed_mps: float, lateral_accel_mps2: float
) -> float | None:
    """Return the yaw-rate limit that keeps v * yaw_rate within lateral accel."""
    speed = abs(speed_mps)
    if speed < 1.0e-6 or lateral_accel_mps2 <= 0.0:
        return None
    return lateral_accel_mps2 / speed


def damped_yaw_command(
    requested_yaw_rate: float,
    measured_yaw_rate: float,
    damping_gain: float,
    maximum_yaw_rate: float,
) -> float:
    """Damp measured rotation so a high-inertia craft does not over-turn.

    A zero gain preserves the original controller.  A positive gain can ask
    for counter-yaw when the vehicle is still rotating after its heading error
    has fallen, which is the behaviour needed to settle a hovercraft turn.
    """
    if damping_gain < 0.0:
        raise ValueError("damping_gain must not be negative")
    if maximum_yaw_rate <= 0.0:
        raise ValueError("maximum_yaw_rate must be positive")
    return _clamp(
        requested_yaw_rate - damping_gain * measured_yaw_rate,
        -maximum_yaw_rate,
        maximum_yaw_rate,
    )


def turn_braking_command(
    requested_linear_speed: float,
    heading_error: float,
    measured_speed: float,
    turn_angle: float,
    brake_speed: float,
) -> float:
    """Request bounded reverse thrust before a moving craft turns too far.

    Disabled when either V3-only tuning value is zero.  This is a proposed
    command only: the Safety supervisor still owns the final command limit.
    """
    if turn_angle <= 0.0 or brake_speed <= 0.0 or measured_speed <= 0.0:
        return requested_linear_speed
    if abs(heading_error) < turn_angle:
        return requested_linear_speed
    return -min(brake_speed, measured_speed)
