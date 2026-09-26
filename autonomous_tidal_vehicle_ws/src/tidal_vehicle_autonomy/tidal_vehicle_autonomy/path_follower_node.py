"""ROS 2 node that converts a planned path into proposed motion commands."""

from __future__ import annotations

from math import atan2
from time import monotonic

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node

from .path_follower_core import compute_command, FollowerConfig, Pose2D, Waypoint


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
        command = compute_command(
            pose,
            self._path,
            self._config,
            self._progress_index,
        )
        self._progress_index = command.progress_index
        proposed = Twist()
        proposed.linear.x = command.linear_x
        proposed.angular.z = command.angular_z
        self._command_publisher.publish(proposed)

        if command.goal_reached and not self._goal_reported:
            self._goal_reported = True
            self.get_logger().info("Path goal reached; proposing a stop")

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
