"""ROS-level test for invalidating a route after a terrain update."""

from time import monotonic

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from tidal_vehicle_autonomy.global_planner_node import GlobalPlannerNode


class PathCapture(Node):
    def __init__(self) -> None:
        super().__init__("global_planner_path_capture")
        self.paths: list[Path] = []
        self.create_subscription(Path, "/planned_path", self.paths.append, 10)


def _spin_for(executor: SingleThreadedExecutor, seconds: float = 0.25) -> None:
    deadline = monotonic() + seconds
    while monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def _costmap(costs: list[int]) -> OccupancyGrid:
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 3
    message.info.height = 1
    message.info.resolution = 1.0
    message.info.origin.orientation.w = 1.0
    message.data = costs
    return message


def _odometry() -> Odometry:
    message = Odometry()
    message.header.frame_id = "map"
    message.pose.pose.position.x = 0.5
    message.pose.pose.position.y = 0.5
    message.pose.pose.orientation.w = 1.0
    return message


def _goal() -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x = 2.5
    message.pose.position.y = 0.5
    message.pose.orientation.w = 1.0
    return message


def test_planner_clears_previous_path_when_route_becomes_blocked() -> None:
    rclpy.init()
    planner = GlobalPlannerNode()
    publisher = Node("global_planner_mock_publishers")
    capture = PathCapture()
    executor = SingleThreadedExecutor()
    for node in (planner, publisher, capture):
        executor.add_node(node)

    try:
        costmap_publisher = publisher.create_publisher(
            OccupancyGrid, "/terrain_costmap", 10
        )
        odom_publisher = publisher.create_publisher(Odometry, "/odom", 10)
        goal_publisher = publisher.create_publisher(
            PoseStamped, "/mission_goal", 10
        )
        _spin_for(executor, 0.1)

        costmap_publisher.publish(_costmap([0, 0, 0]))
        odom_publisher.publish(_odometry())
        goal_publisher.publish(_goal())
        _spin_for(executor)

        assert capture.paths
        assert len(capture.paths[-1].poses) == 3

        costmap_publisher.publish(_costmap([0, 100, 0]))
        _spin_for(executor)

        assert capture.paths[-1].poses == []
    finally:
        for node in (capture, publisher, planner):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()
