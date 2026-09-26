"""ROS 2 node that converts a planned path into proposed motion commands."""

from __future__ import annotations

from dataclasses import replace
from math import atan2, floor, hypot
from time import monotonic

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node

from .motion_limits import sensor_limited_speed
from .path_follower_core import (
    FollowerConfig,
    Pose2D,
    Waypoint,
    compute_command,
    curvature_speed_limit,
    lateral_accel_yaw_rate_limit,
    normalise_angle,
    path_points_ahead,
    stopping_reach,
    zone_speed_limit,
)


class PathFollowerNode(Node):
    """Follow the current path without bypassing the safety supervisor."""

    def __init__(self) -> None:
        super().__init__("path_follower")
        self._declare_parameters()
        self._config = FollowerConfig(
            max_linear_speed=self._parameter("max_linear_speed"),
            max_angular_speed=self._parameter("max_angular_speed"),
            lookahead_distance=self._parameter("lookahead_distance"),
            goal_tolerance=self._parameter("goal_tolerance"),
            heading_gain=self._parameter("heading_gain"),
            slow_down_distance=self._parameter("slow_down_distance"),
            rotate_in_place_angle=self._parameter("rotate_in_place_angle"),
        )
        self._odom_timeout = self._parameter("odom_timeout")
        control_rate = self._parameter("control_rate_hz")
        if self._odom_timeout <= 0.0 or control_rate <= 0.0:
            raise ValueError("odom_timeout and control_rate_hz must be positive")

        self._path: list[Waypoint] = []
        self._path_frame = ""
        self._odometry: Odometry | None = None
        self._odometry_received_at: float | None = None
        self._progress_index = 0
        self._goal_reported = False
        self._frame_warning_active = False

        # Zone speed limits from the split cost bands (AGENTS.md), a
        # curvature limit and a speed-scaled lookahead. All off by default
        # (Version 2); sim.launch.py sets them for Version 3.
        self._zone_limits = [float(v) for v in self.get_parameter("zone_speed_limits_mps").value]
        if len(self._zone_limits) != 4:
            self._zone_limits = []
        self._brake_decel = self._parameter("brake_decel_mps2")
        self._lateral_accel = self._parameter("lateral_accel_limit_mps2")
        self._lookahead_time = self._parameter("lookahead_time_s")
        self._reaction_time = self._parameter("reaction_time_s")
        self._obstacle_detection_range = self._parameter(
            "obstacle_detection_range_m"
        )
        self._obstacle_clearance = self._parameter("obstacle_clearance_m")
        self._costmap: tuple | None = None
        if self._lookahead_time < 0.0 or self._reaction_time < 0.0:
            raise ValueError("lookahead_time_s and reaction_time_s must be non-negative")
        if self._obstacle_detection_range < 0.0:
            raise ValueError("obstacle_detection_range_m must be non-negative")
        if self._obstacle_clearance < 0.0:
            raise ValueError("obstacle_clearance_m must be non-negative")
        if (
            self._obstacle_detection_range > 0.0
            and self._obstacle_clearance >= self._obstacle_detection_range
        ):
            raise ValueError("obstacle_clearance_m must be below detection range")
        if (
            self._zone_limits or self._obstacle_detection_range > 0.0
        ) and self._brake_decel <= 0.0:
            raise ValueError(
                "brake_decel_mps2 must be positive when motion limits are active"
            )
        if self._zone_limits:
            self.create_subscription(OccupancyGrid, "/terrain_costmap", self._on_costmap, 10)

        self._command_publisher = self.create_publisher(Twist, "/cmd_vel_proposed", 10)
        self.create_subscription(Path, "/planned_path", self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odometry, 20)
        self.create_timer(1.0 / control_rate, self._on_control_timer)
        self.get_logger().info("Path follower ready; waiting for a path and odometry")

    def _declare_parameters(self) -> None:
        defaults = {
            "control_rate_hz": 10.0,
            "max_linear_speed": 0.8,
            # Keep yaw demand within the stable operating envelope of the
            # simplified air-cushion dynamics.
            "max_angular_speed": 0.45,
            "lookahead_distance": 0.75,
            "goal_tolerance": 0.25,
            "heading_gain": 0.9,
            "slow_down_distance": 1.0,
            "rotate_in_place_angle": 1.05,
            "odom_timeout": 0.5,
            # Version 3 (0 / empty = off): [firm, open surveyed water,
            # mud/roots/debris, elevated risk] in m/s; braking deceleration for
            # the look-ahead reach; lateral acceleration allowed in curves;
            # lookahead grows to this many seconds of travel.
            "zone_speed_limits_mps": [0.0],
            "brake_decel_mps2": 1.0,
            "lateral_accel_limit_mps2": 0.0,
            "lookahead_time_s": 0.0,
            "reaction_time_s": 1.0,
            "obstacle_detection_range_m": 0.0,
            "obstacle_clearance_m": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_path(self, message: Path) -> None:
        path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        path_frame = message.header.frame_id
        if path == self._path and path_frame == self._path_frame:
            return

        self._path = path
        self._path_frame = path_frame
        self._progress_index = 0
        self._goal_reported = False
        if self._path:
            self.get_logger().info(f"Following a new {len(self._path)}-pose path")
        else:
            self.get_logger().warning("Received an empty path; proposing a stop")
            self._publish_stop()

    def _on_odometry(self, message: Odometry) -> None:
        self._odometry = message
        self._odometry_received_at = monotonic()

    def _on_control_timer(self) -> None:
        if not self._path or self._odometry is None:
            self._publish_stop()
            return
        if (
            self._odometry_received_at is None
            or monotonic() - self._odometry_received_at > self._odom_timeout
        ):
            self._publish_stop()
            return
        if not self._frames_match():
            self._publish_stop()
            return

        pose = self._pose_2d(self._odometry)
        config = self._speed_limited_config(pose)
        command = compute_command(
            pose,
            self._path,
            config,
            self._progress_index,
        )
        self._progress_index = command.progress_index
        linear_x = command.linear_x
        if self._lateral_accel > 0.0 and command.target_index is not None:
            tx, ty = self._path[command.target_index]
            error = normalise_angle(atan2(ty - pose.y, tx - pose.x) - pose.yaw)
            curve = curvature_speed_limit(error, config.lookahead_distance, self._lateral_accel)
            if curve is not None:
                linear_x = min(linear_x, curve)
        angular_z = command.angular_z
        if self._lateral_accel > 0.0 and linear_x > 0.0:
            measured = self._odometry.twist.twist.linear
            reference_speed = max(linear_x, hypot(measured.x, measured.y))
            yaw_limit = lateral_accel_yaw_rate_limit(
                reference_speed, self._lateral_accel
            )
            if yaw_limit is not None:
                angular_z = max(-yaw_limit, min(angular_z, yaw_limit))
        proposed = Twist()
        proposed.linear.x = linear_x
        proposed.angular.z = angular_z
        self._command_publisher.publish(proposed)

        if command.goal_reached and not self._goal_reported:
            self._goal_reported = True
            self.get_logger().info("Path goal reached; proposing a stop")

    def _on_costmap(self, message: OccupancyGrid) -> None:
        info = message.info
        self._costmap = (info.resolution, info.width, info.height,
                         info.origin.position.x, info.origin.position.y, list(message.data))

    def _speed_limited_config(self, pose: Pose2D) -> FollowerConfig:
        """This tick's config: max speed capped by the slowest zone between
        the vehicle and its stopping distance, lookahead scaled with speed."""
        if (not self._zone_limits
                and self._lookahead_time <= 0.0
                and self._obstacle_detection_range <= 0.0):
            return self._config
        twist = self._odometry.twist.twist.linear
        speed = hypot(twist.x, twist.y)
        changes = {}
        max_speed = self._config.max_linear_speed
        if self._obstacle_detection_range > 0.0:
            max_speed = min(
                max_speed,
                sensor_limited_speed(
                    self._obstacle_detection_range - self._obstacle_clearance,
                    self._brake_decel,
                    self._reaction_time,
                ),
            )
        if self._lookahead_time > 0.0:
            changes["lookahead_distance"] = max(self._config.lookahead_distance,
                                                speed * self._lookahead_time)
        if self._zone_limits and self._costmap is not None:
            res, width, height, ox, oy, data = self._costmap
            reach = stopping_reach(
                speed, self._brake_decel, reaction_s=self._reaction_time
            )
            costs = []
            for x, y in path_points_ahead(pose, self._path, self._progress_index, reach):
                col, row = int(floor((x - ox) / res)), int(floor((y - oy) / res))
                if 0 <= col < width and 0 <= row < height:
                    costs.append(int(data[row * width + col]))
            zone = zone_speed_limit(costs, self._zone_limits)
            if zone is not None:
                max_speed = max(0.05, min(max_speed, zone))
        if max_speed < self._config.max_linear_speed:
            changes["max_linear_speed"] = max_speed
        return replace(self._config, **changes) if changes else self._config

    def _frames_match(self) -> bool:
        odom_frame = self._odometry.header.frame_id
        if self._path_frame and odom_frame and self._path_frame != odom_frame:
            if not self._frame_warning_active:
                self.get_logger().warning(
                    "Cannot follow a path in a different frame without TF: "
                    f"{self._path_frame}, {odom_frame}"
                )
                self._frame_warning_active = True
            return False
        self._frame_warning_active = False
        return True

    def _publish_stop(self) -> None:
        self._command_publisher.publish(Twist())

    @staticmethod
    def _pose_2d(message: Odometry) -> Pose2D:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return Pose2D(position.x, position.y, yaw)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except KeyboardInterrupt:
                    pass


if __name__ == "__main__":
    main()
