"""Minimal ROS 2 node that exposes operator-friendly summaries for telemetry."""

from __future__ import annotations

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from tidal_vehicle_interfaces.msg import SafetyStatus

from .operator_dashboard import summarize_dashboard


class OperatorDashboardNode(Node):
    """Aggregate live telemetry into a readable operator summary."""

    def __init__(self) -> None:
        super().__init__("operator_dashboard")
        self._mission_state = "HOLD"
        self._return_required = False
        self._planned_route_points = 0
        self._return_route_points = 0
        self._battery_percent = 0.0
        self._return_margin_percent = 0.0
        self._reason = "Awaiting telemetry."

        mission_event_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/mission_event", self._on_mission_event, mission_event_qos
        )
        self.create_subscription(SafetyStatus, "/safety_status", self._on_safety_status, 10)
        self.create_subscription(PoseStamped, "/mission_goal", self._on_mission_goal, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._summary_pub = self.create_publisher(String, "/operator_summary", 10)
        self.create_timer(1.0, self._publish_summary)

    def _on_mission_event(self, message: String) -> None:
        event = message.data.strip()
        if event == "delivery_confirmed":
            self._mission_state = "DELIVERED"
        elif event in {"mission_complete", "mission_reset"}:
            self._mission_state = "HOLD"
        self._reason = f"Mission event: {event}"

    def _on_safety_status(self, message: SafetyStatus) -> None:
        self._mission_state = message.state
        self._return_required = bool(message.return_required)
        self._return_margin_percent = float(message.return_margin_percent)
        self._reason = message.reason

    def _on_mission_goal(self, _: PoseStamped) -> None:
        self._reason = "Mission goal received; monitoring route progress."

    def _on_cmd_vel(self, _: Twist) -> None:
        self._reason = "Command updated; monitoring vehicle response."

    def _publish_summary(self) -> None:
        summary = summarize_dashboard(
            mission_state=self._mission_state,
            return_required=self._return_required,
            planned_route_points=self._planned_route_points,
            return_route_points=self._return_route_points,
            battery_percent=self._battery_percent,
            return_margin_percent=self._return_margin_percent,
            reason=self._reason,
        )
        self._summary_pub.publish(String(data=summary))
        self.get_logger().info(summary)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OperatorDashboardNode()
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
