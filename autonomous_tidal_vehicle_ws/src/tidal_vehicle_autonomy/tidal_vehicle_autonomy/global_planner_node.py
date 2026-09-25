"""ROS 2 wrapper that publishes terrain-aware routes with LiDAR obstacles."""

from math import atan2

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from tidal_vehicle_interfaces.msg import TerrainState

from .lidar_obstacle_core import obstacle_cells_from_scan, overlay_obstacles
from .planner_core import GridCell, GridCostMap, plan_path


class GlobalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("global_planner")
        self.declare_parameter("blocked_cost_threshold", 90)
        self.declare_parameter("require_matching_frame", True)
        self.declare_parameter("obstacle_inflation_radius_m", 1.7)
        self.declare_parameter("obstacle_max_range_m", 8.0)

        self._base_costmap: GridCostMap | None = None
        self._costmap_msg: OccupancyGrid | None = None
        self._costmap: GridCostMap | None = None
        self._pose: Odometry | None = None
        self._goal: PoseStamped | None = None
        self._dynamic_obstacles: frozenset[GridCell] = frozenset()
        self._last_scan: LaserScan | None = None
        self._last_scan_pose: tuple[float, float, float] | None = None
        self._last_path_cells: tuple[GridCell, ...] | None = None
        self._path_available = False
        self._scan_frame_warning_active = False

        inflation_radius = self._float_parameter("obstacle_inflation_radius_m")
        obstacle_max_range = self._float_parameter("obstacle_max_range_m")
        if inflation_radius < 0.0 or obstacle_max_range <= 0.0:
            raise ValueError(
                "Obstacle inflation radius must be non-negative and range positive"
            )

        self.create_subscription(OccupancyGrid, "/terrain_costmap", self._on_costmap, 10)
        self.create_subscription(Odometry, "/odom", self._on_odometry, 20)
        self.create_subscription(PoseStamped, "/mission_goal", self._on_goal, 10)
        self.create_subscription(TerrainState, "/terrain_state", self._on_terrain_state, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self._path_publisher = self.create_publisher(Path, "/planned_path", 10)

        self.get_logger().info(
            "Global planner ready; waiting for map, pose, mission goal, and LiDAR"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_costmap(self, message: OccupancyGrid) -> None:
        try:
            self._base_costmap = GridCostMap(
                width=message.info.width,
                height=message.info.height,
                costs=message.data,
                resolution=message.info.resolution,
                origin_x=message.info.origin.position.x,
                origin_y=message.info.origin.position.y,
                blocked_cost=int(self.get_parameter("blocked_cost_threshold").value),
            )
        except ValueError as error:
            self.get_logger().error(f"Ignoring invalid terrain costmap: {error}")
            return

        self._costmap_msg = message
        if self._last_scan is not None and self._last_scan_pose is not None:
            try:
                self._dynamic_obstacles = self._project_scan(
                    self._last_scan,
                    self._last_scan_pose,
                )
            except ValueError as error:
                self.get_logger().error(
                    f"Ignoring stored LiDAR scan after costmap update: {error}"
                )
                self._dynamic_obstacles = frozenset()
        self._rebuild_planning_costmap()
        self._replan("terrain costmap update")

    def _on_odometry(self, message: Odometry) -> None:
        self._pose = message
        self._replan("odometry update")

    def _on_goal(self, message: PoseStamped) -> None:
        self._goal = message
        self._last_path_cells = None
        self._replan("new mission goal")

    def _on_terrain_state(self, _: TerrainState) -> None:
        self._replan("terrain state update")

    def _on_scan(self, message: LaserScan) -> None:
        if self._base_costmap is None or self._costmap_msg is None or self._pose is None:
            return
        if not self._map_and_odometry_frames_match():
            return

        position = self._pose.pose.pose.position
        scan_pose = (
            position.x,
            position.y,
            self._yaw_from_odometry(self._pose),
        )
        try:
            obstacles = self._project_scan(message, scan_pose)
        except ValueError as error:
            self.get_logger().error(f"Ignoring invalid LiDAR scan: {error}")
            return

        self._last_scan = message
        self._last_scan_pose = scan_pose
        if obstacles == self._dynamic_obstacles:
            return

        self._dynamic_obstacles = obstacles
        self._rebuild_planning_costmap()
        self.get_logger().info(
            f"LiDAR obstacle overlay now contains {len(obstacles)} blocked cells"
        )
        self._replan("LiDAR obstacle update")

    def _project_scan(
        self,
        message: LaserScan,
        scan_pose: tuple[float, float, float],
    ) -> frozenset[GridCell]:
        scan_range_max = min(
            float(message.range_max),
            self._float_parameter("obstacle_max_range_m"),
        )
        return obstacle_cells_from_scan(
            self._base_costmap,
            robot_x=scan_pose[0],
            robot_y=scan_pose[1],
            robot_yaw=scan_pose[2],
            ranges=message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            range_min=float(message.range_min),
            range_max=scan_range_max,
            inflation_radius=self._float_parameter("obstacle_inflation_radius_m"),
        )

    def _rebuild_planning_costmap(self) -> None:
        if self._base_costmap is None:
            self._costmap = None
            return
        self._costmap = overlay_obstacles(
            self._base_costmap,
            self._dynamic_obstacles,
        )

    def _replan(self, reason: str) -> None:
        if (
            self._costmap is None
            or self._costmap_msg is None
            or self._pose is None
            or self._goal is None
        ):
            return
        if not self._frames_match():
            return

        start = self._costmap.grid_from_world(
            self._pose.pose.pose.position.x,
            self._pose.pose.pose.position.y,
        )
        goal = self._costmap.grid_from_world(
            self._goal.pose.position.x,
            self._goal.pose.position.y,
        )
        cells = plan_path(self._costmap, start, goal)
        if cells is None:
            self._last_path_cells = None
            if self._path_available:
                empty_path = Path()
                empty_path.header = self._costmap_msg.header
                self._path_publisher.publish(empty_path)
                self._path_available = False
                self.get_logger().warning(
                    f"No safe route available after {reason}; cleared previous path"
                )
            else:
                self.get_logger().warning(f"No safe route available after {reason}")
            return

        path_cells = tuple(cells)
        if path_cells == self._last_path_cells:
            return
        self._last_path_cells = path_cells

        path = Path()
        path.header = self._costmap_msg.header
        for cell in cells:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = self._costmap.world_from_grid(cell)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_publisher.publish(path)
        self._path_available = True
        self.get_logger().info(f"Published {len(cells)}-cell route after {reason}")

    def _map_and_odometry_frames_match(self) -> bool:
        if not self.get_parameter("require_matching_frame").value:
            return True
        frames = {
            frame
            for frame in (
                self._costmap_msg.header.frame_id,
                self._pose.header.frame_id,
            )
            if frame
        }
        if len(frames) <= 1:
            self._scan_frame_warning_active = False
            return True
        if not self._scan_frame_warning_active:
            self.get_logger().warning(
                "Cannot project LiDAR across differing map and odometry frames: "
                + ", ".join(sorted(frames))
            )
            self._scan_frame_warning_active = True
        return False

    def _frames_match(self) -> bool:
        if not self.get_parameter("require_matching_frame").value:
            return True
        frames = {
            frame
            for frame in (
                self._costmap_msg.header.frame_id,
                self._pose.header.frame_id,
                self._goal.header.frame_id,
            )
            if frame
        }
        if len(frames) <= 1:
            return True
        self.get_logger().warning(
            "Cannot plan across differing frames without TF: "
            + ", ".join(sorted(frames))
        )
        return False

    @staticmethod
    def _yaw_from_odometry(message: Odometry) -> float:
        orientation = message.pose.pose.orientation
        return atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
