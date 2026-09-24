"""ROS 2 wrapper that publishes a terrain-aware route proposal."""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node

from tidal_vehicle_interfaces.msg import TerrainState

from .planner_core import GridCostMap, plan_path


class GlobalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("global_planner")
        self.declare_parameter("blocked_cost_threshold", 90)
        self.declare_parameter("require_matching_frame", True)

        self._costmap_msg: OccupancyGrid | None = None
        self._costmap: GridCostMap | None = None
        self._pose: Odometry | None = None
        self._goal: PoseStamped | None = None
        self._last_path_cells: tuple[tuple[int, int], ...] | None = None

        self.create_subscription(OccupancyGrid, "/terrain_costmap", self._on_costmap, 10)
        self.create_subscription(Odometry, "/odom", self._on_odometry, 20)
        self.create_subscription(PoseStamped, "/mission_goal", self._on_goal, 10)
        self.create_subscription(TerrainState, "/terrain_state", self._on_terrain_state, 10)
        self._path_publisher = self.create_publisher(Path, "/planned_path", 10)

        self.get_logger().info("Global planner ready; waiting for map, pose, and mission goal")

    def _on_costmap(self, message: OccupancyGrid) -> None:
        try:
            self._costmap = GridCostMap(
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

    def _replan(self, reason: str) -> None:
        if self._costmap is None or self._costmap_msg is None or self._pose is None or self._goal is None:
            return
        if not self._frames_match():
            return
        start = self._costmap.grid_from_world(self._pose.pose.pose.position.x, self._pose.pose.pose.position.y)
        goal = self._costmap.grid_from_world(self._goal.pose.position.x, self._goal.pose.position.y)
        cells = plan_path(self._costmap, start, goal)
        if cells is None:
            self._last_path_cells = None
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
        self.get_logger().info(f"Published {len(cells)}-cell route after {reason}")

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
        self.get_logger().warning("Cannot plan across differing frames without TF: " + ", ".join(sorted(frames)))
        return False


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
