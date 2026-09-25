"""ROS-level path-follower test using local mock publishers and a subscriber."""

from time import monotonic

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from tidal_vehicle_autonomy.path_follower_node import PathFollowerNode


class CommandCapture(Node):
    def __init__(self) -> None:
        super().__init__("path_follower_command_capture")
        self.commands: list[Twist] = []
        self.create_subscription(Twist, "/cmd_vel_proposed", self.commands.append, 10)


def _spin_for(executor: SingleThreadedExecutor, seconds: float = 0.25) -> None:
    deadline = monotonic() + seconds
    while monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def _straight_path() -> Path:
    message = Path()
    message.header.frame_id = "map"
    for x_m in (0.0, 1.0, 2.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x_m
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def _odometry() -> Odometry:
    message = Odometry()
    message.header.frame_id = "map"
    message.pose.pose.orientation.w = 1.0
    return message


def test_follower_publishes_forward_proposal_and_stops_on_empty_path() -> None:
    rclpy.init()
    follower = PathFollowerNode()
    publisher = Node("path_follower_mock_publishers")
    capture = CommandCapture()
    executor = SingleThreadedExecutor()
    for node in (follower, publisher, capture):
        executor.add_node(node)

    try:
        path_publisher = publisher.create_publisher(Path, "/planned_path", 10)
        odom_publisher = publisher.create_publisher(Odometry, "/odom", 10)
        _spin_for(executor, 0.1)

        path_publisher.publish(_straight_path())
        odom_publisher.publish(_odometry())
        _spin_for(executor)

        assert capture.commands
        assert any(command.linear.x > 0.0 for command in capture.commands)
        assert all(command.linear.y == 0.0 for command in capture.commands)

        empty_path = Path()
        empty_path.header.frame_id = "map"
        capture.commands.clear()
        path_publisher.publish(empty_path)
        _spin_for(executor)

        assert capture.commands
        assert capture.commands[-1].linear.x == 0.0
        assert capture.commands[-1].angular.z == 0.0
    finally:
        for node in (capture, publisher, follower):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()
