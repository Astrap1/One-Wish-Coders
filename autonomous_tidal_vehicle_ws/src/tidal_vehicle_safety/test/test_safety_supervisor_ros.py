"""ROS-level command-gate test using local mock publishers and a subscriber."""

import os

os.environ.setdefault("ROS_DOMAIN_ID", "32")


from time import monotonic

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from tidal_vehicle_interfaces.msg import TerrainState, VehicleHealth
from tidal_vehicle_safety.safety_supervisor import SafetySupervisor


class CommandCapture(Node):
    def __init__(self):
        super().__init__("safety_command_capture")
        self.commands = []
        self.create_subscription(Twist, "/cmd_vel", self.commands.append, 10)


def _spin_for(executor: SingleThreadedExecutor, seconds: float = 0.2) -> None:
    deadline = monotonic() + seconds
    while monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def _costmap() -> OccupancyGrid:
    message = OccupancyGrid()
    message.info.width = 10
    message.info.height = 10
    message.info.resolution = 1.0
    message.info.origin.position.x = 0.0
    message.info.origin.position.y = 0.0
    message.data = [0] * 100
    return message


def _path(points) -> Path:
    message = Path()
    for x_m, y_m in points:
        pose = PoseStamped()
        pose.pose.position.x = x_m
        pose.pose.position.y = y_m
        message.poses.append(pose)
    return message


def _health(mobility_percent: float) -> VehicleHealth:
    message = VehicleHealth()
    message.battery_percent = 80.0
    message.mobility_health_percent = mobility_percent
    message.link_ok = True
    message.payload_secured = True
    return message


def _terrain() -> TerrainState:
    message = TerrainState()
    message.tide_risk = 0.2
    message.seconds_until_corridor_unsafe = 600.0
    message.corridor_traversable = True
    return message


def test_supervisor_forwards_healthy_command_and_holds_critical_fault():
    rclpy.init()
    supervisor = SafetySupervisor()
    publisher = Node("safety_mock_publishers")
    capture = CommandCapture()
    executor = SingleThreadedExecutor()
    for node in (supervisor, publisher, capture):
        executor.add_node(node)

    try:
        costmap_pub = publisher.create_publisher(OccupancyGrid, "/terrain_costmap", 10)
        health_pub = publisher.create_publisher(VehicleHealth, "/vehicle_health", 10)
        terrain_pub = publisher.create_publisher(TerrainState, "/terrain_state", 10)
        goal_pub = publisher.create_publisher(PoseStamped, "/mission_goal", 10)
        planned_pub = publisher.create_publisher(Path, "/planned_path", 10)
        return_pub = publisher.create_publisher(Path, "/return_path", 10)
        proposed_pub = publisher.create_publisher(Twist, "/cmd_vel_proposed", 10)

        costmap_pub.publish(_costmap())
        _spin_for(executor)
        health_pub.publish(_health(100.0))
        terrain_pub.publish(_terrain())
        goal_pub.publish(PoseStamped())
        planned_pub.publish(_path([(1.0, 1.0), (2.0, 1.0)]))
        return_pub.publish(_path([(1.0, 1.0), (0.0, 0.0)]))
        _spin_for(executor)

        capture.commands.clear()
        proposed = Twist()
        proposed.linear.x = 0.5
        proposed_pub.publish(proposed)
        _spin_for(executor)
        assert capture.commands
        assert capture.commands[-1].linear.x == 0.5

        health_pub.publish(_health(30.0))
        _spin_for(executor)
        capture.commands.clear()
        proposed_pub.publish(proposed)
        _spin_for(executor)
        assert capture.commands
        assert capture.commands[-1].linear.x == 0.0
        assert capture.commands[-1].angular.z == 0.0
    finally:
        for node in (capture, publisher, supervisor):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


def test_reference_only_mode_forwards_lidar_route_across_no_go_map():
    rclpy.init()
    supervisor = SafetySupervisor(
        parameter_overrides=[Parameter("costmap_reference_only", value=True)]
    )
    publisher = Node("safety_reference_only_mock_publishers")
    capture = CommandCapture()
    executor = SingleThreadedExecutor()
    for node in (supervisor, publisher, capture):
        executor.add_node(node)

    try:
        costmap_pub = publisher.create_publisher(OccupancyGrid, "/terrain_costmap", 10)
        health_pub = publisher.create_publisher(VehicleHealth, "/vehicle_health", 10)
        terrain_pub = publisher.create_publisher(TerrainState, "/terrain_state", 10)
        goal_pub = publisher.create_publisher(PoseStamped, "/mission_goal", 10)
        planned_pub = publisher.create_publisher(Path, "/planned_path", 10)
        return_pub = publisher.create_publisher(Path, "/return_path", 10)
        proposed_pub = publisher.create_publisher(Twist, "/cmd_vel_proposed", 10)
        _spin_for(executor, 0.1)

        blocked_map = _costmap()
        blocked_map.data = [100] * 100
        costmap_pub.publish(blocked_map)
        health_pub.publish(_health(100.0))
        terrain_pub.publish(_terrain())
        goal_pub.publish(PoseStamped())
        planned_pub.publish(_path([(1.0, 1.0), (2.0, 1.0)]))
        return_pub.publish(_path([(2.0, 1.0), (0.0, 0.0)]))
        _spin_for(executor)

        proposed = Twist()
        proposed.linear.x = 0.5
        proposed_pub.publish(proposed)
        _spin_for(executor)

        assert supervisor._estimate.valid
        assert capture.commands[-1].linear.x == 0.5
    finally:
        for node in (capture, publisher, supervisor):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


def test_outbound_goal_update_requires_a_fresh_replanned_path():
    rclpy.init()
    supervisor = SafetySupervisor()
    publisher = Node("safety_retarget_mock_publishers")
    capture = CommandCapture()
    executor = SingleThreadedExecutor()
    for node in (supervisor, publisher, capture):
        executor.add_node(node)

    try:
        costmap_pub = publisher.create_publisher(OccupancyGrid, "/terrain_costmap", 10)
        health_pub = publisher.create_publisher(VehicleHealth, "/vehicle_health", 10)
        terrain_pub = publisher.create_publisher(TerrainState, "/terrain_state", 10)
        goal_pub = publisher.create_publisher(PoseStamped, "/mission_goal", 10)
        planned_pub = publisher.create_publisher(Path, "/planned_path", 10)
        return_pub = publisher.create_publisher(Path, "/return_path", 10)
        proposed_pub = publisher.create_publisher(Twist, "/cmd_vel_proposed", 10)

        costmap_pub.publish(_costmap())
        health_pub.publish(_health(100.0))
        terrain_pub.publish(_terrain())
        goal_pub.publish(PoseStamped())
        planned_pub.publish(_path([(1.0, 1.0), (2.0, 1.0)]))
        return_pub.publish(_path([(1.0, 1.0), (0.0, 0.0)]))
        _spin_for(executor)

        proposed = Twist()
        proposed.linear.x = 0.5
        proposed_pub.publish(proposed)
        _spin_for(executor)
        assert capture.commands[-1].linear.x == 0.5

        # Retargeting while outbound must stop the old route until Autonomy
        # confirms a fresh replacement path.
        capture.commands.clear()
        goal_pub.publish(PoseStamped())
        proposed_pub.publish(proposed)
        _spin_for(executor)
        assert capture.commands
        assert capture.commands[-1].linear.x == 0.0

        planned_pub.publish(_path([(1.0, 1.0), (1.0, 2.0)]))
        _spin_for(executor, 0.05)
        proposed_pub.publish(proposed)
        _spin_for(executor)
        assert capture.commands[-1].linear.x == 0.5
    finally:
        for node in (capture, publisher, supervisor):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


def test_remote_mode_uses_remote_command_despite_automatic_return_policy():
    rclpy.init()
    supervisor = SafetySupervisor()
    publisher = Node("remote_control_mock_publishers")
    capture = CommandCapture()
    executor = SingleThreadedExecutor()
    for node in (supervisor, publisher, capture):
        executor.add_node(node)

    try:
        health_pub = publisher.create_publisher(VehicleHealth, "/vehicle_health", 10)
        terrain_pub = publisher.create_publisher(TerrainState, "/terrain_state", 10)
        remote_mode_pub = publisher.create_publisher(
            Bool,
            "/operator_remote_enabled",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        remote_cmd_pub = publisher.create_publisher(Twist, "/operator_cmd_vel", 10)
        autonomy_cmd_pub = publisher.create_publisher(Twist, "/cmd_vel_proposed", 10)

        terrain = _terrain()
        terrain.tide_risk = 0.95  # would normally request an automatic return/hold
        health_pub.publish(_health(100.0))
        terrain_pub.publish(terrain)
        remote_mode_pub.publish(Bool(data=True))
        _spin_for(executor)

        remote = Twist()
        remote.linear.x = 2.0  # remote safety cap is 0.8 m/s
        remote.angular.z = 1.0
        remote_cmd_pub.publish(remote)
        _spin_for(executor)
        assert capture.commands
        assert capture.commands[-1].linear.x == 0.8
        assert capture.commands[-1].angular.z == 0.6

        capture.commands.clear()
        autonomous = Twist()
        autonomous.linear.x = 0.5
        autonomy_cmd_pub.publish(autonomous)
        _spin_for(executor)
        assert not capture.commands
    finally:
        for node in (capture, publisher, supervisor):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()
