"""ROS wrapper that exposes the browser-based operator dashboard."""

from __future__ import annotations

import math
import io
from queue import Empty, Queue
import threading
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image, LaserScan
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from std_msgs.msg import Bool, String

from tidal_vehicle_interfaces.msg import SafetyStatus, TerrainState, VehicleHealth

from .operator_dashboard import build_dashboard_payload
from .web_dashboard import (
    set_remote_request_handler,
    update_camera_frame,
    update_dashboard_state,
    serve_dashboard,
)


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
        self._mission_events: list[dict[str, str]] = []
        self._cost_map: dict[str, object] = {"width": 0, "height": 0, "resolution": 1.0, "origin_x": 0.0, "origin_y": 0.0, "data": []}
        self._remote_enabled = False
        self._remote_action = "stop"
        self._remote_action_at = 0.0
        self._remote_requests: Queue[dict[str, object]] = Queue()
        self._remote_timeout_s = 0.35

        self.create_subscription(String, "/mission_event", self._on_mission_event, 10)
        self.create_subscription(SafetyStatus, "/safety_status", self._on_safety_status, 10)
        self.create_subscription(VehicleHealth, "/vehicle_health", self._on_vehicle_health, 10)
        self.create_subscription(TerrainState, "/terrain_state", self._on_terrain_state, 10)
        self.create_subscription(Path, "/planned_path", self._on_planned_path, 10)
        self.create_subscription(Path, "/return_path", self._on_return_path, 10)
        self.create_subscription(OccupancyGrid, "/terrain_costmap", self._on_costmap, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/mission_goal", self._on_mission_goal, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Image, "/camera/image_raw", self._on_camera, 10)
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._remote_mode_pub = self.create_publisher(Bool, "/operator_remote_enabled", mode_qos)
        self._remote_command_pub = self.create_publisher(Twist, "/operator_cmd_vel", 10)
        self._scenario_event_pub = self.create_publisher(String, "/scenario_event", 10)
        self._mission_goal_pub = self.create_publisher(PoseStamped, "/mission_goal", 10)
        set_remote_request_handler(self._queue_remote_request)
        self.create_timer(1.0, self._publish_state)
        self.create_timer(0.1, self._remote_control_timer)

    def _queue_remote_request(self, request: dict[str, object]) -> bool:
        """Validate browser input in the HTTP thread, then handle it in ROS."""
        kind = request.get("kind")
        if kind == "mode" and isinstance(request.get("enabled"), bool):
            self._remote_requests.put(request)
            return True
        if kind == "motion" and request.get("action") in {
            "forward", "reverse", "left", "right", "stop",
        }:
            self._remote_requests.put(request)
            return True
        if kind == "abort":
            self._remote_requests.put(request)
            return True
        if (
            kind == "goal"
            and isinstance(request.get("x"), (int, float))
            and isinstance(request.get("y"), (int, float))
            and math.isfinite(float(request["x"]))
            and math.isfinite(float(request["y"]))
        ):
            self._remote_requests.put(request)
            return True
        return False

    def _remote_control_timer(self) -> None:
        while True:
            try:
                request = self._remote_requests.get_nowait()
            except Empty:
                break
            kind = request["kind"]
            if kind == "mode":
                self._remote_enabled = bool(request["enabled"])
                self._remote_action = "stop"
                self._remote_action_at = time.monotonic()
                self._remote_mode_pub.publish(Bool(data=self._remote_enabled))
                self._remote_command_pub.publish(Twist())
            elif kind == "motion" and self._remote_enabled:
                self._remote_action = str(request["action"])
                self._remote_action_at = time.monotonic()
                if self._remote_action == "stop":
                    self._remote_command_pub.publish(Twist())
            elif kind == "abort":
                self._remote_enabled = False
                self._remote_action = "stop"
                self._remote_mode_pub.publish(Bool(data=False))
                self._remote_command_pub.publish(Twist())
                self._scenario_event_pub.publish(String(data="operator_abort"))
            elif kind == "goal":
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = "map"
                goal.pose.position.x = float(request["x"])
                goal.pose.position.y = float(request["y"])
                goal.pose.orientation.w = 1.0
                self._mission_goal_pub.publish(goal)

        if not self._remote_enabled or self._remote_action == "stop":
            return
        if time.monotonic() - self._remote_action_at > self._remote_timeout_s:
            self._remote_action = "stop"
            self._remote_command_pub.publish(Twist())
            return
        command = Twist()
        if self._remote_action == "forward":
            command.linear.x = 0.8
        elif self._remote_action == "reverse":
            command.linear.x = -0.5
        elif self._remote_action == "left":
            command.angular.z = 0.6
        elif self._remote_action == "right":
            command.angular.z = -0.6
        self._remote_command_pub.publish(command)

    def _on_mission_event(self, message: String) -> None:
        event = message.data.strip()
        self._mission_events.append({"time": time.strftime("%H:%M:%S"), "event": event or "(empty event)"})
        self._mission_events = self._mission_events[-20:]
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

    def _on_costmap(self, message: OccupancyGrid) -> None:
        width = int(message.info.width)
        height = int(message.info.height)
        data = [int(value) for value in message.data]
        if width * height != len(data) or width <= 0 or height <= 0:
            self._cost_map = {"width": 0, "height": 0, "resolution": 1.0, "origin_x": 0.0, "origin_y": 0.0, "data": []}
        else:
            self._cost_map = {
                "width": width,
                "height": height,
                "resolution": float(message.info.resolution),
                "origin_x": float(message.info.origin.position.x),
                "origin_y": float(message.info.origin.position.y),
                "data": data,
            }
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

    def _on_camera(self, message: Image) -> None:
        if message.encoding not in {"rgb8", "bgr8"}:
            return
        channels = 3
        width = int(message.width)
        height = int(message.height)
        row_bytes = width * channels
        raw = bytes(message.data)
        if message.step > row_bytes:
            raw = b"".join(raw[row * int(message.step):row * int(message.step) + row_bytes] for row in range(height))
        image = PILImage.frombytes("RGB", (width, height), raw)
        if message.encoding == "bgr8":
            red, green, blue = image.split()
            image = PILImage.merge("RGB", (blue, green, red))
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=82, optimize=True)
        update_camera_frame(output.getvalue())

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
                "mission_events": self._mission_events,
                "cost_map": self._cost_map,
            },
        )
        payload["remote_enabled"] = self._remote_enabled
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
        set_remote_request_handler(None)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
