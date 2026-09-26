"""ROS 2 planner for terrain-aware outbound and return routes."""

import json
from math import atan2, hypot
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from tidal_vehicle_interfaces.msg import SafetyStatus, TerrainState

from .lidar_obstacle_core import obstacle_cells_from_scan, overlay_obstacles
from .planner_core import GridCell, GridCostMap, plan_path


class GlobalPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("global_planner")
        self.declare_parameter("blocked_cost_threshold", 90)
        self.declare_parameter("require_matching_frame", True)
        # Version 1 is approximately 1.2 m by 0.7 m. Use its half-diagonal
        # plus a small clearance as the standalone default. The shared launch
        # overrides this with the Version 2 footprint-specific value.
        self.declare_parameter("obstacle_inflation_radius_m", 0.75)
        self.declare_parameter("obstacle_max_range_m", 8.0)
        self.declare_parameter("home_x_m", 0.0)
        self.declare_parameter("home_y_m", 0.0)
        self.declare_parameter("home_frame", "map")
        self.declare_parameter("goal_event_tolerance_m", 2.0)
        self.declare_parameter("return_path_refresh_rate_hz", 2.0)

        self._base_costmap: GridCostMap | None = None
        self._costmap_msg: OccupancyGrid | None = None
        self._costmap: GridCostMap | None = None
        self._pose: Odometry | None = None
        self._goal: PoseStamped | None = None
        self._dynamic_obstacles: frozenset[GridCell] = frozenset()
        self._last_scan: LaserScan | None = None
        self._last_scan_pose: tuple[float, float, float] | None = None
        self._last_odometry_replan_position: tuple[float, float] | None = None
        self._last_path_cells: tuple[GridCell, ...] | None = None
        self._last_return_path_cells: tuple[GridCell, ...] | None = None
        self._return_path_available = False
        self._active_path_end: tuple[float, float] | None = None
        self._path_available = False
        self._return_requested = False
        self._mission_finished = False
        self._delivery_reported = False
        self._completion_reported = False
        self._scan_frame_warning_active = False
        self._mission_event_history: list[dict[str, object]] = []
        self._mission_event_sequence = 0

        inflation_radius = self._float_parameter("obstacle_inflation_radius_m")
        obstacle_max_range = self._float_parameter("obstacle_max_range_m")
        event_tolerance = self._float_parameter("goal_event_tolerance_m")
        return_refresh_rate = self._float_parameter("return_path_refresh_rate_hz")
        if inflation_radius < 0.0 or obstacle_max_range <= 0.0:
            raise ValueError(
                "Obstacle inflation radius must be non-negative and range positive"
            )
        if event_tolerance <= 0.0 or return_refresh_rate <= 0.0:
            raise ValueError(
                "goal_event_tolerance_m and return_path_refresh_rate_hz must be positive"
            )
        if not self._string_parameter("home_frame"):
            raise ValueError("home_frame must not be empty")

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/terrain_costmap", self._on_costmap, 10)
        self.create_subscription(Odometry, "/odom", self._on_odometry, 20)
        self.create_subscription(PoseStamped, "/mission_goal", self._on_goal, 10)
        self.create_subscription(TerrainState, "/terrain_state", self._on_terrain_state, 10)
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            SafetyStatus,
            "/safety_status",
            self._on_safety_status,
            status_qos,
        )
        self.create_subscription(String, "/scenario_event", self._on_scenario_event, 10)

        self._path_publisher = self.create_publisher(Path, "/planned_path", 10)
        self._return_path_publisher = self.create_publisher(Path, "/return_path", 10)
        self._mission_event_publisher = self.create_publisher(
            String,
            "/mission_event",
            status_qos,
        )
        # A retained snapshot lets a dashboard opened after a transition
        # recover the small, demo-facing mission timeline.
        self._mission_event_history_publisher = self.create_publisher(
            String,
            "/mission_event_history",
            status_qos,
        )
        self.create_timer(
            1.0 / return_refresh_rate,
            self._on_return_path_refresh_timer,
        )

        self.get_logger().info(
            "Global planner ready; waiting for map, pose, mission goal, and LiDAR"
        )

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

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
        # Safety requires a route received after every terrain-map update,
        # even when A* selects the same cells.
        self._replan("terrain costmap update", force_publish=True)

    def _on_odometry(self, message: Odometry) -> None:
        self._pose = message
        self._check_goal_reached()
        if self._costmap is None or self._active_target() is None:
            return

        position = (message.pose.pose.position.x, message.pose.pose.position.y)
        if self._last_odometry_replan_position is not None:
            distance = hypot(
                position[0] - self._last_odometry_replan_position[0],
                position[1] - self._last_odometry_replan_position[1],
            )
            if distance < self._costmap.resolution * 0.5:
                return
        self._last_odometry_replan_position = position
        self._replan("odometry update")

    def _remember_odometry_replan_position(self) -> None:
        if self._pose is None:
            self._last_odometry_replan_position = None
            return
        position = self._pose.pose.pose.position
        self._last_odometry_replan_position = (position.x, position.y)

    def _on_goal(self, message: PoseStamped) -> None:
        if self._return_requested or self._mission_finished:
            self.get_logger().warning(
                "Ignoring a new mission goal until the current mission is reset"
            )
            return
        self._goal = message
        self._last_path_cells = None
        self._last_return_path_cells = None
        self._return_path_available = False
        self._active_path_end = None
        self._delivery_reported = False
        self._completion_reported = False
        self._remember_odometry_replan_position()
        self._replan("new mission goal", force_publish=True)

    def _on_terrain_state(self, _: TerrainState) -> None:
        self._replan("terrain state update")

    def _on_safety_status(self, message: SafetyStatus) -> None:
        if (
            not message.return_required
            or self._return_requested
            or self._mission_finished
            or self._goal is None
        ):
            return

        self._return_requested = True
        self._last_path_cells = None
        self._active_path_end = None
        self._path_available = False
        self._publish_empty_routes(include_return=True)
        self.get_logger().warning(
            f"Safety requested return to HOME: {message.reason or 'no reason provided'}"
        )
        self._remember_odometry_replan_position()
        self._replan("safety return request", force_publish=True)

    def _on_scenario_event(self, message: String) -> None:
        if message.data.strip().lower() != "reset":
            return

        self._goal = None
        self._return_requested = False
        self._mission_finished = False
        self._delivery_reported = False
        self._completion_reported = False
        self._last_path_cells = None
        self._last_return_path_cells = None
        self._return_path_available = False
        self._active_path_end = None
        self._path_available = False
        self._last_odometry_replan_position = None
        self._publish_empty_routes(include_return=True)
        self._publish_mission_event("mission_reset")
        self.get_logger().info("Mission reset; waiting for a new goal")

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

    def _replan(self, reason: str, force_publish: bool = False) -> None:
        target = self._active_target()
        if (
            self._mission_finished
            or self._costmap is None
            or self._costmap_msg is None
            or self._pose is None
            or target is None
        ):
            return
        if not self._frames_match(target[2]):
            return

        start = self._costmap.grid_from_world(
            self._pose.pose.pose.position.x,
            self._pose.pose.pose.position.y,
        )
        if not self._return_requested:
            self._update_prospective_return_path(
                start,
                reason,
                force_publish,
            )

        goal = self._costmap.grid_from_world(target[0], target[1])
        cells = plan_path(self._costmap, start, goal)
        if cells is None:
            self._last_path_cells = None
            self._active_path_end = None
            if self._path_available or force_publish:
                self._publish_empty_routes(include_return=self._return_requested)
                self._path_available = False
                self.get_logger().warning(
                    f"No safe {self._route_name()} available after {reason}; "
                    "cleared previous path"
                )
            else:
                self.get_logger().warning(
                    f"No safe {self._route_name()} available after {reason}"
                )
            return

        path_cells = tuple(cells)
        if path_cells == self._last_path_cells and not force_publish:
            return
        self._last_path_cells = path_cells

        path = self._path_message(cells)
        self._path_publisher.publish(path)
        if self._return_requested:
            self._last_return_path_cells = path_cells
            self._return_path_available = True
            self._return_path_publisher.publish(path)
        self._path_available = True
        self._active_path_end = self._costmap.world_from_grid(cells[-1])
        self.get_logger().info(
            f"Published {len(cells)}-cell {self._route_name()} after {reason}"
        )

    def _update_prospective_return_path(
        self,
        start: GridCell,
        reason: str,
        force_publish: bool,
    ) -> None:
        home_frame = self._string_parameter("home_frame")
        if not self._frames_match(home_frame):
            return
        home = self._costmap.grid_from_world(
            self._float_parameter("home_x_m"),
            self._float_parameter("home_y_m"),
        )
        cells = plan_path(self._costmap, start, home)
        if cells is None:
            self._last_return_path_cells = None
            if self._return_path_available or force_publish:
                self._publish_empty_return_path()
                self.get_logger().warning(
                    f"No prospective return route available after {reason}"
                )
            self._return_path_available = False
            return

        path_cells = tuple(cells)
        if path_cells == self._last_return_path_cells and not force_publish:
            return
        self._last_return_path_cells = path_cells
        self._return_path_available = True
        self._return_path_publisher.publish(self._path_message(cells))

    def _on_return_path_refresh_timer(self) -> None:
        if (
            self._mission_finished
            or not self._return_path_available
            or self._last_return_path_cells is None
            or self._costmap is None
            or self._costmap_msg is None
        ):
            return
        # Refresh only Safety's copy, preserving path-follower progress.
        self._return_path_publisher.publish(
            self._path_message(list(self._last_return_path_cells))
        )

    def _active_target(self) -> tuple[float, float, str] | None:
        if self._return_requested:
            return (
                self._float_parameter("home_x_m"),
                self._float_parameter("home_y_m"),
                self._string_parameter("home_frame"),
            )
        if self._goal is None:
            return None
        return (
            self._goal.pose.position.x,
            self._goal.pose.position.y,
            self._goal.header.frame_id,
        )

    def _path_message(self, cells: list[GridCell]) -> Path:
        path = Path()
        path.header.frame_id = self._costmap_msg.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for cell in cells:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = self._costmap.world_from_grid(
                cell
            )
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _empty_path_message(self) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        if self._costmap_msg is not None:
            path.header.frame_id = self._costmap_msg.header.frame_id
        else:
            path.header.frame_id = self._string_parameter("home_frame")
        return path

    def _publish_empty_return_path(self) -> None:
        self._return_path_publisher.publish(self._empty_path_message())

    def _publish_empty_routes(self, include_return: bool) -> None:
        path = self._empty_path_message()
        self._path_publisher.publish(path)
        if include_return:
            self._last_return_path_cells = None
            self._return_path_available = False
            self._return_path_publisher.publish(path)

    def _check_goal_reached(self) -> None:
        if (
            self._pose is None
            or self._active_path_end is None
            or not self._path_available
            or self._mission_finished
        ):
            return
        position = self._pose.pose.pose.position
        distance = hypot(
            position.x - self._active_path_end[0],
            position.y - self._active_path_end[1],
        )
        # A grid planner can only navigate to a cell, not an exact point
        # inside that cell. Treat entry into the target cell as arrival so a
        # vehicle with momentum cannot cross the cell, replan behind itself,
        # and circle forever after missing a sub-cell tolerance.
        in_target_cell = (
            self._costmap is not None
            and self._costmap.grid_from_world(position.x, position.y)
            == self._costmap.grid_from_world(*self._active_path_end)
        )
        if (
            not in_target_cell
            and distance > self._float_parameter("goal_event_tolerance_m")
        ):
            return

        if self._return_requested:
            if self._completion_reported:
                return
            self._completion_reported = True
            self._mission_finished = True
            self._path_available = False
            self._publish_empty_routes(include_return=True)
            self._publish_mission_event("mission_complete")
            self.get_logger().info("HOME reached; mission complete")
            return

        if not self._delivery_reported:
            self._delivery_reported = True
            self._publish_mission_event("delivery_confirmed")
            self.get_logger().info(
                "Delivery goal reached; waiting for Safety's return request"
            )

    def _publish_mission_event(self, event: str) -> None:
        self._mission_event_sequence += 1
        self._mission_event_history.append(
            {
                "sequence": self._mission_event_sequence,
                "time": time.strftime("%H:%M:%S"),
                "event": event,
            }
        )
        self._mission_event_history = self._mission_event_history[-20:]

        history = String()
        history.data = json.dumps({"events": self._mission_event_history})
        self._mission_event_history_publisher.publish(history)

        message = String()
        message.data = event
        self._mission_event_publisher.publish(message)

    def _route_name(self) -> str:
        return "return route" if self._return_requested else "outbound route"

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

    def _frames_match(self, target_frame: str) -> bool:
        if not self.get_parameter("require_matching_frame").value:
            return True
        frames = {
            frame
            for frame in (
                self._costmap_msg.header.frame_id,
                self._pose.header.frame_id,
                target_frame,
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
