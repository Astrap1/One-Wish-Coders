"""Shared motion-envelope calculations for autonomy."""

from math import isfinite, sqrt


def stopping_distance(
    speed_mps: float,
    brake_decel_mps2: float,
    reaction_time_s: float = 1.0,
    minimum_m: float = 0.0,
) -> float:
    """Return reaction distance plus constant-deceleration braking distance."""
    if not isfinite(speed_mps) or speed_mps < 0.0:
        raise ValueError("speed_mps must be finite and non-negative")
    if not isfinite(brake_decel_mps2) or brake_decel_mps2 <= 0.0:
        raise ValueError("brake_decel_mps2 must be finite and positive")
    if not isfinite(reaction_time_s) or reaction_time_s < 0.0:
        raise ValueError("reaction_time_s must be finite and non-negative")
    if not isfinite(minimum_m) or minimum_m < 0.0:
        raise ValueError("minimum_m must be finite and non-negative")

    distance = speed_mps * reaction_time_s + (
        speed_mps * speed_mps / (2.0 * brake_decel_mps2)
    )
    return max(minimum_m, distance)


def sensor_limited_speed(
    sensor_range_m: float,
    brake_decel_mps2: float,
    reaction_time_s: float = 1.0,
) -> float:
    """Return the greatest speed that can stop within the usable sensor range."""
    if not isfinite(sensor_range_m) or sensor_range_m <= 0.0:
        raise ValueError("sensor_range_m must be finite and positive")
    if not isfinite(brake_decel_mps2) or brake_decel_mps2 <= 0.0:
        raise ValueError("brake_decel_mps2 must be finite and positive")
    if not isfinite(reaction_time_s) or reaction_time_s < 0.0:
        raise ValueError("reaction_time_s must be finite and non-negative")

    reaction_term = brake_decel_mps2 * reaction_time_s
    return sqrt(
        reaction_term * reaction_term
        + 2.0 * brake_decel_mps2 * sensor_range_m
    ) - reaction_term


def speed_aware_sensor_range(
    speed_mps: float,
    minimum_range_m: float,
    maximum_range_m: float,
    brake_decel_mps2: float,
    reaction_time_s: float = 1.0,
    clearance_m: float = 0.0,
) -> float:
    """Clamp stopping distance to the configured useful sensor range."""
    if (
        not isfinite(minimum_range_m)
        or not isfinite(maximum_range_m)
        or minimum_range_m < 0.0
        or maximum_range_m <= 0.0
    ):
        raise ValueError("sensor ranges must be non-negative and maximum positive")
    if minimum_range_m > maximum_range_m:
        raise ValueError("minimum_range_m must not exceed maximum_range_m")
    if not isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError("clearance_m must be finite and non-negative")

    required = stopping_distance(
        speed_mps,
        brake_decel_mps2,
        reaction_time_s,
    )
    return min(maximum_range_m, max(minimum_range_m, required + clearance_m))
