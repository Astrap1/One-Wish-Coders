"""ROS-level tests for terrain, LiDAR and return-route replanning."""

import json
import os

os.environ.setdefault("ROS_DOMAIN_ID", "31")


from math import inf
from time import monotonic

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from tidal_vehicle_interfaces.msg import SafetyStatus
import tidal_vehicle_autonomy.global_planner_node as planner_module
from tidal_vehicle_autonomy.global_planner_node import GlobalPlannerNode


class PlannerCapture(Node):
    def __init__(self, node_name: str = "global_planner_capture") -> None:
        super().__init__(node_name)
        self.paths: list[Path] = []
        self.return_paths: list[Path] = []
        self.mission_events: list[str] = []
        self.mission_event_histories: list[str] = []
        mission_event_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Path, "/planned_path", self.paths.append, 10)
        self.create_subscription(Path, "/return_path", self.return_paths.append, 10)
        self.create_subscription(
            String,
            "/mission_event",
            lambda message: self.mission_events.append(message.data),
            mission_event_qos,
        )
        self.create_subscription(
            String,
            "/mission_event_history",
            lambda message: self.mission_event_histories.append(message.data),
            mission_event_qos,
        )


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


def _safety_status(return_required: bool, reason: str = "") -> SafetyStatus:
    message = SafetyStatus()
    message.return_required = return_required
    message.reason = reason
    return message


def _status_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _planner_test_nodes() -> tuple[
    GlobalPlannerNode,
    Node,
    PlannerCapture,
    SingleThreadedExecutor,
]:
    planner = GlobalPlannerNode()
    publisher = Node("global_planner_mock_publishers")
    capture = PlannerCapture()
    executor = SingleThreadedExecutor()
    for node in (planner, publisher, capture):
        executor.add_node(node)
    return planner, publisher, capture, executor


def _destroy_test_nodes(
    planner: GlobalPlannerNode,
    publisher: Node,
    capture: PlannerCapture,
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

        # One scan is insufficient to divert the route. Confirmation keeps a
        # single noisy return from producing a visible steering correction.
        scan_publisher.publish(_scan(4.0))
        _spin_for(executor)
        assert capture.paths[-1] is direct_path

        scan_publisher.publish(_scan(4.0))
        _spin_for(executor)

        detour_path = capture.paths[-1]
        detour_points = {
            (pose.pose.position.x, pose.pose.position.y)
            for pose in detour_path.poses
        }
        assert (4.5, 3.5) not in detour_points
        assert any(y_m != 3.5 for _, y_m in detour_points)

        # A brief dropout must not restore the direct route. Five consecutive
        # clear scans confirm that the obstacle is gone.
        for _ in range(4):
            scan_publisher.publish(_scan(inf))
            _spin_for(executor, 0.05)
        assert capture.paths[-1] is detour_path

        scan_publisher.publish(_scan(inf))
        _spin_for(executor)

        restored_path = capture.paths[-1]
        assert all(pose.pose.position.y == 3.5 for pose in restored_path.poses)
    finally:
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()


def test_lidar_hit_on_a_mapped_obstacle_does_not_add_a_second_overlay() -> None:
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

        # The static map's middle cell is already no-go.  The planner's
        # private clearance map covers its collision footprint before A*.
        costs = [0] * 63
        costs[3 * 9 + 4] = 100
        costmap_publisher.publish(_costmap(costs, width=9, height=7))
        odom_publisher.publish(_odometry(x_m=0.5, y_m=3.5))
        goal_publisher.publish(_goal(x_m=8.5, y_m=3.5))
        _spin_for(executor)

        static_detour = capture.paths[-1]
        assert all(
            (pose.pose.position.x, pose.pose.position.y) != (4.5, 3.5)
            for pose in static_detour.poses
        )

        # Two scans would normally confirm a new dynamic hit. They must not
        # produce an overlay when the endpoint is already statically covered.
        scan_publisher.publish(_scan(4.0))
        _spin_for(executor)
        scan_publisher.publish(_scan(4.0))
        _spin_for(executor)

        assert planner._dynamic_obstacles == frozenset()
        assert capture.paths[-1] is static_detour
    finally:
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()


def test_safety_request_switches_to_fresh_latched_return_route() -> None:
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
        status_publisher = publisher.create_publisher(
            SafetyStatus,
            "/safety_status",
            _status_qos(),
        )
        _spin_for(executor, 0.1)

        open_map = _costmap([0] * 27, width=9, height=3)
        costmap_publisher.publish(open_map)
        odom_publisher.publish(_odometry(x_m=6.5, y_m=1.5))
        goal_publisher.publish(_goal(x_m=8.5, y_m=1.5))
        _spin_for(executor)

        assert capture.paths[-1].poses[-1].pose.position.x == 8.5
        assert capture.return_paths
        assert capture.return_paths[-1].poses[-1].pose.position.x == 0.5
        status_publisher.publish(
            _safety_status(True, "Tide margin requires return")
        )
        _spin_for(executor)

        assert capture.return_paths
        return_path = capture.return_paths[-1]
        assert return_path.poses[-1].pose.position.x == 0.5
        assert return_path.poses[-1].pose.position.y == 0.5
        assert [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in capture.paths[-1].poses
        ] == [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in return_path.poses
        ]

        return_count = len(capture.return_paths)
        planned_count = len(capture.paths)
        # Allow scheduling margin beyond the 0.5 s refresh period.
        _spin_for(executor, 0.8)
        assert len(capture.return_paths) > return_count
        assert len(capture.paths) == planned_count

        return_count = len(capture.return_paths)
        costmap_publisher.publish(open_map)
        _spin_for(executor)
        assert len(capture.return_paths) > return_count

        status_publisher.publish(_safety_status(False))
        odom_publisher.publish(_odometry(x_m=5.5, y_m=1.5))
        _spin_for(executor)
        assert capture.return_paths[-1].poses[-1].pose.position.x == 0.5
    finally:
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()


def test_goal_events_complete_delivery_return_and_reset_lifecycle() -> None:
    rclpy.init()
    planner, publisher, capture, executor = _planner_test_nodes()
    assert planner.get_parameter("goal_event_tolerance_m").value == 2.0

    try:
        costmap_publisher = publisher.create_publisher(
            OccupancyGrid, "/terrain_costmap", 10
        )
        odom_publisher = publisher.create_publisher(Odometry, "/odom", 10)
        goal_publisher = publisher.create_publisher(
            PoseStamped, "/mission_goal", 10
        )
        status_publisher = publisher.create_publisher(
            SafetyStatus,
            "/safety_status",
            _status_qos(),
        )
        scenario_publisher = publisher.create_publisher(
            String,
            "/scenario_event",
            10,
        )
        _spin_for(executor, 0.1)

        costmap_publisher.publish(_costmap([0, 0, 0]))
        odom_publisher.publish(_odometry())
        goal_publisher.publish(_goal())
        _spin_for(executor)

        # A pose near the edge of the two-metre delivery zone must still
        # trigger the delivery lifecycle transition.
        odom_publisher.publish(_odometry(x_m=2.01, y_m=0.01))
        _spin_for(executor)
        assert capture.mission_events.count("delivery_confirmed") == 1

        status_publisher.publish(
            _safety_status(True, "Payload delivered; return home")
        )
        _spin_for(executor)
        assert capture.return_paths[-1].poses[-1].pose.position.x == 0.5

        odom_publisher.publish(_odometry(x_m=0.01, y_m=0.01))
        _spin_for(executor)
        assert capture.mission_events.count("mission_complete") == 1
        assert capture.paths[-1].poses == []

        reset = String()
        # Dashboard operators commonly reset at low tide. That must reset the
        # mission lifecycle too, otherwise stale routes can block the next goal.
        reset.data = "tide_reset"
        scenario_publisher.publish(reset)
        _spin_for(executor)
        assert capture.mission_events.count("mission_reset") == 1
        assert capture.return_paths[-1].poses == []
    finally:
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()


def test_unchanged_costmap_refreshes_without_running_astar(monkeypatch) -> None:
    calls = 0
    real_plan_path = planner_module.plan_path

    def counted_plan_path(costmap, start, goal):
        nonlocal calls
        calls += 1
        return real_plan_path(costmap, start, goal)

    monkeypatch.setattr(planner_module, "plan_path", counted_plan_path)
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

        open_map = _costmap([0] * 9, width=9, height=1)
        costmap_publisher.publish(open_map)
        odom_publisher.publish(_odometry())
        goal_publisher.publish(_goal(x_m=8.5))
        _spin_for(executor)

        calls_after_goal = calls
        path_count = len(capture.paths)
        original_geometry = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in capture.paths[-1].poses
        ]

        costmap_publisher.publish(open_map)
        _spin_for(executor)

        assert calls == calls_after_goal
        assert len(capture.paths) == path_count + 1
        assert [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in capture.paths[-1].poses
        ] == original_geometry
    finally:
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()


def test_late_operator_recovers_retained_mission_event_history() -> None:
    rclpy.init()
    planner, publisher, capture, executor = _planner_test_nodes()
    late_capture: PlannerCapture | None = None

    try:
        scenario_publisher = publisher.create_publisher(String, "/scenario_event", 10)
        _spin_for(executor, 0.1)

        reset = String()
        reset.data = "reset"
        scenario_publisher.publish(reset)
        _spin_for(executor)
        assert capture.mission_events[-1] == "mission_reset"

        # This node joins after the transition.  It must receive both the
        # current lifecycle event and the retained dashboard timeline.
        late_capture = PlannerCapture("late_global_planner_capture")
        executor.add_node(late_capture)
        _spin_for(executor)

        assert late_capture.mission_events == ["mission_reset"]
        history = json.loads(late_capture.mission_event_histories[-1])
        assert history["events"][-1]["event"] == "mission_reset"
    finally:
        if late_capture is not None:
            executor.remove_node(late_capture)
            late_capture.destroy_node()
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()


def test_odometry_updates_return_without_replacing_active_path(monkeypatch) -> None:
    calls: list[tuple[tuple[int, int], tuple[int, int]]] = []
    real_plan_path = planner_module.plan_path

    def counted_plan_path(costmap, start, goal):
        calls.append((start, goal))
        return real_plan_path(costmap, start, goal)

    monkeypatch.setattr(planner_module, "plan_path", counted_plan_path)
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
        odom_publisher.publish(_odometry(x_m=0.99, y_m=0.5))
        goal_publisher.publish(_goal())
        _spin_for(executor)

        calls_after_goal = len(calls)
        assert calls_after_goal > 0
        active_paths_after_goal = len(capture.paths)
        return_paths_after_goal = len(capture.return_paths)

        # Crossing a grid boundary by a few centimetres must not let odometry
        # noise repeatedly reset an otherwise unchanged route.
        odom_publisher.publish(_odometry(x_m=1.01, y_m=0.5))
        _spin_for(executor, 0.1)
        assert len(calls) == calls_after_goal

        odom_publisher.publish(_odometry(x_m=1.6, y_m=0.5))
        _spin_for(executor, 0.1)
        assert len(calls) > calls_after_goal
        assert len(capture.paths) == active_paths_after_goal
        assert len(capture.return_paths) > return_paths_after_goal
    finally:
        _destroy_test_nodes(planner, publisher, capture, executor)
        rclpy.shutdown()
