"""ROS wrapper that exposes the browser-based operator dashboard."""

from __future__ import annotations

import math
import threading
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tidal_vehicle_interfaces.msg import SafetyStatus, TerrainState, VehicleHealth

from .operator_dashboard import build_dashboard_payload
from .web_dashboard import update_dashboard_state, serve_dashboard


class BrowserDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("browser_dashboard")
        self._mission_state = "HOLD"
        self._return_required = False
        self._planned_route_points = 0
        self._return_route_points = 0
        self._battery_percent = 0.0
        self._return_margin_percent = 0.0
        self._reason = "Awaiting telemetry."
        self._estimated_return_energy_percent = 0.0
        self._estimated_return_time_s = 0.0
        self._mobility_health_percent = 0.0
        self._link_ok = False
        self._payload_secured = False
        self._fault = ""
        self._tide_state = "UNKNOWN"
        self._tide_risk = 0.0
        self._water_level_m = 0.0
        self._tide_rate_m_per_minute = 0.0
        self._seconds_until_corridor_unsafe = 0.0
        self._corridor_traversable = False
        self._nearest_obstacle_m = 0.0
        self._nearby_obstacle_samples = 0
        self._hold_started_s: float | None = None
        self._speed_mps = 0.0
        self._vehicle_x = 0.0
        self._vehicle_y = 0.0
        self._planned_path: list[dict[str, float]] = []
        self._return_path: list[dict[str, float]] = []

        self.create_subscription(String, "/mission_event", self._on_mission_event, 10)
        self.create_subscription(SafetyStatus, "/safety_status", self._on_safety_status, 10)
        self.create_subscription(VehicleHealth, "/vehicle_health", self._on_vehicle_health, 10)
        self.create_subscription(TerrainState, "/terrain_state", self._on_terrain_state, 10)
        self.create_subscription(Path, "/planned_path", self._on_planned_path, 10)
        self.create_subscription(Path, "/return_path", self._on_return_path, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(PoseStamped, "/mission_goal", self._on_mission_goal, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_timer(1.0, self._publish_state)

    def _on_mission_event(self, message: String) -> None:
        event = message.data.strip()
        if event == "delivery_confirmed":
            self._mission_state = "DELIVERED"
        elif event in {"mission_complete", "mission_reset"}:
            self._mission_state = "HOLD"
        self._reason = f"Mission event: {event}"
        self._publish_state()

    def _on_safety_status(self, message: SafetyStatus) -> None:
        self._mission_state = message.state
        self._return_required = bool(message.return_required)
        self._return_margin_percent = float(message.return_margin_percent)
        self._estimated_return_energy_percent = float(message.estimated_return_energy_percent)
        self._estimated_return_time_s = float(message.estimated_return_time_s)
        self._reason = message.reason
        if message.state == "HOLD" and self._hold_started_s is None:
            self._hold_started_s = time.monotonic()
        elif message.state != "HOLD":
            self._hold_started_s = None
        self._publish_state()

    def _on_scan(self, message: LaserScan) -> None:
        valid_ranges = [
            value for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        self._nearest_obstacle_m = min(valid_ranges, default=0.0)
        self._nearby_obstacle_samples = sum(value < 5.0 for value in valid_ranges)
        self._publish_state()

    def _on_vehicle_health(self, message: VehicleHealth) -> None:
        self._battery_percent = float(message.battery_percent)
        self._mobility_health_percent = float(message.mobility_health_percent)
        self._link_ok = bool(message.link_ok)
        self._payload_secured = bool(message.payload_secured)
        self._fault = message.fault
        self._publish_state()

    def _on_terrain_state(self, message: TerrainState) -> None:
        self._tide_state = message.tide_state
        self._tide_risk = float(message.tide_risk)
        self._water_level_m = float(message.water_level_m)
        self._tide_rate_m_per_minute = float(message.tide_rate_m_per_minute)
        self._seconds_until_corridor_unsafe = float(message.seconds_until_corridor_unsafe)
        self._corridor_traversable = bool(message.corridor_traversable)
        self._publish_state()

    def _on_planned_path(self, message: Path) -> None:
        self._planned_route_points = len(message.poses)
        self._planned_path = [{"x": pose.pose.position.x, "y": pose.pose.position.y} for pose in message.poses]
        self._publish_state()

    def _on_return_path(self, message: Path) -> None:
        self._return_route_points = len(message.poses)
        self._return_path = [{"x": pose.pose.position.x, "y": pose.pose.position.y} for pose in message.poses]
        self._publish_state()

    def _on_odom(self, message: Odometry) -> None:
        velocity = message.twist.twist.linear
        self._speed_mps = math.hypot(float(velocity.x), float(velocity.y))
        self._vehicle_x = float(message.pose.pose.position.x)
        self._vehicle_y = float(message.pose.pose.position.y)
        self._publish_state()

    def _on_mission_goal(self, _: PoseStamped) -> None:
        self._reason = "Mission goal received; monitoring route progress."
        self._publish_state()

    def _on_cmd_vel(self, _: Twist) -> None:
        self._reason = "Command updated; monitoring vehicle response."
        self._publish_state()

    def _publish_state(self) -> None:
        payload = build_dashboard_payload(
            mission_state=self._mission_state,
            return_required=self._return_required,
            planned_route_points=self._planned_route_points,
            return_route_points=self._return_route_points,
            battery_percent=self._battery_percent,
            return_margin_percent=self._return_margin_percent,
            reason=self._reason,
            telemetry={
                "estimated_return_energy_percent": self._estimated_return_energy_percent,
                "estimated_return_time_s": self._estimated_return_time_s,
                "mobility_health_percent": self._mobility_health_percent,
                "link_ok": self._link_ok,
                "payload_secured": self._payload_secured,
                "fault": self._fault,
                "tide_state": self._tide_state,
                "tide_risk": self._tide_risk,
                "water_level_m": self._water_level_m,
                "tide_rate_m_per_minute": self._tide_rate_m_per_minute,
                "seconds_until_corridor_unsafe": self._seconds_until_corridor_unsafe,
                "corridor_traversable": self._corridor_traversable,
                "nearest_obstacle_m": self._nearest_obstacle_m,
                "nearby_obstacle_samples": self._nearby_obstacle_samples,
                "hold_duration_s": (
                    time.monotonic() - self._hold_started_s
                    if self._hold_started_s is not None else 0.0
                ),
                "speed_mps": self._speed_mps,
                "vehicle_x": self._vehicle_x,
                "vehicle_y": self._vehicle_y,
                "planned_path": self._planned_path,
                "return_path": self._return_path,
            },
        )
        update_dashboard_state(**payload)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BrowserDashboardNode()
    server_thread = threading.Thread(target=serve_dashboard, daemon=True)
    server_thread.start()
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
