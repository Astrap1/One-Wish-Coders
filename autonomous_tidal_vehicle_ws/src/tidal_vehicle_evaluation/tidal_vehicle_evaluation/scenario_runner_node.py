"""Minimal ROS 2 node for publishing deterministic scenario events."""

from __future__ import annotations

from std_msgs.msg import String
import rclpy
from rclpy.node import Node


class ScenarioRunnerNode(Node):
    """Publish predefined scenario controls for evaluation and fault injection."""

    def __init__(self) -> None:
        super().__init__("scenario_runner")
        self._publisher = self.create_publisher(String, "/scenario_event", 10)
        self.create_timer(5.0, self._emit_default_event)

    def _emit_default_event(self) -> None:
        message = String(data="reset")
        self._publisher.publish(message)
        self.get_logger().info("Published scenario event: reset")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScenarioRunnerNode()
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
