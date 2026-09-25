"""ROS-level tests for terrain and LiDAR-triggered replanning."""

from math import inf
from time import monotonic

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

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


def _costmap(
    costs: list[int],
    width: int = 3,
    height: int = 1,
) -> OccupancyGrid:
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = width
    message.info.height = height
    message.info.resolution = 1.0
    message.info.origin.orientation.w = 1.0
    message.data = costs
    return message


def _odometry(x_m: float = 0.5, y_m: float = 0.5) -> Odometry:
    message = Odometry()
    message.header.frame_id = "map"
    message.pose.pose.position.x = x_m
    message.pose.pose.position.y = y_m
    message.pose.pose.orientation.w = 1.0
    return message


def _goal(x_m: float = 2.5, y_m: float = 0.5) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x = x_m
    message.pose.position.y = y_m
    message.pose.orientation.w = 1.0
    return message


def _scan(measured_range: float) -> LaserScan:
    message = LaserScan()
    message.header.frame_id = "base_scan"
    message.angle_min = 0.0
    message.angle_max = 0.0
    message.angle_increment = 0.0
    message.range_min = 0.1
    message.range_max = 10.0
    message.ranges = [measured_range]
    return message


def _planner_test_nodes() -> tuple[
    GlobalPlannerNode,
    Node,
    PathCapture,
    SingleThreadedExecutor,
]:
    planner = GlobalPlannerNode()
    publisher = Node("global_planner_mock_publishers")
    capture = PathCapture()
    executor = SingleThreadedExecutor()
    for node in (planner, publisher, capture):
        executor.add_node(node)
    return planner, publisher, capture, executor


def _destroy_test_nodes(
    planner: GlobalPlannerNode,
    publisher: Node,
    capture: PathCapture,
    executor: SingleThreadedExecutor,
) -> None:
    for node in (capture, publisher, planner):
        executor.remove_node(node)
        node.destroy_node()


def test_planner_clears_previous_path_when_route_becomes_blocked() -> None:
    rclpy.init()
    planner, publisher, capture, executor = _planner_test_nodes()

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
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()


def test_lidar_obstacle_causes_detour_and_clear_scan_restores_route() -> None:
    rclpy.init()
    planner, publisher, capture, executor = _planner_test_nodes()

    try:
        costmap_publisher = publisher.create_publisher(
            OccupancyGrid, "/terrain_costmap", 10
        )
        odom_publisher = publisher.create_publisher(Odometry, "/odom", 10)
        goal_publisher = publisher.create_publisher(
            PoseStamped, "/mission_goal", 10
        )
        scan_publisher = publisher.create_publisher(LaserScan, "/scan", 10)
        _spin_for(executor, 0.1)

        costmap_publisher.publish(_costmap([0] * 63, width=9, height=7))
        odom_publisher.publish(_odometry(x_m=0.5, y_m=3.5))
        goal_publisher.publish(_goal(x_m=8.5, y_m=3.5))
        _spin_for(executor)

        assert capture.paths
        direct_path = capture.paths[-1]
        assert all(pose.pose.position.y == 3.5 for pose in direct_path.poses)

        scan_publisher.publish(_scan(4.0))
        _spin_for(executor)

        detour_path = capture.paths[-1]
        detour_points = {
            (pose.pose.position.x, pose.pose.position.y)
            for pose in detour_path.poses
        }
        assert (4.5, 3.5) not in detour_points
        assert any(y_m != 3.5 for _, y_m in detour_points)

        scan_publisher.publish(_scan(inf))
        _spin_for(executor)

        restored_path = capture.paths[-1]
        assert all(pose.pose.position.y == 3.5 for pose in restored_path.poses)
    finally:
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()
