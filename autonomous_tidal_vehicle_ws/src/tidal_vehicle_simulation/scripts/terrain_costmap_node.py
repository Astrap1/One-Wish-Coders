#!/usr/bin/env python3
"""Publish a static integration cost map until Person 3's tide manager is ready."""

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class TerrainCostmapNode(Node):
    """Publish the terrain bands used by the Version 1 transition world."""

    def __init__(self) -> None:
        super().__init__("terrain_costmap")
        declare = self.declare_parameter
        self._frame = str(declare("frame_id", "map").value)
        self._resolution = float(declare("resolution_m", 0.5).value)
        self._width = int(declare("width_cells", 84).value)
        self._height = int(declare("height_cells", 30).value)
        self._origin_x = float(declare("origin_x_m", -5.0).value)
        self._origin_y = float(declare("origin_y_m", -7.5).value)
        self._water_start = float(declare("water_start_x_m", 4.0).value)
        self._water_end = float(declare("water_end_x_m", 12.0).value)
        self._mud_end = float(declare("mud_end_x_m", 22.0).value)
        self._firm_cost = int(declare("firm_cost", 10).value)
        self._water_cost = int(declare("water_cost", 30).value)
        self._mud_cost = int(declare("mud_cost", 50).value)
        publish_rate = float(declare("publish_rate_hz", 1.0).value)

        if (
            not self._frame
            or self._resolution <= 0.0
            or self._width <= 0
            or self._height <= 0
            or publish_rate <= 0.0
        ):
            raise ValueError("Cost-map frame, dimensions, resolution and rate must be valid")
        if not (
            0 <= self._firm_cost < 90
            and 0 <= self._water_cost < 90
            and 0 <= self._mud_cost < 90
        ):
            raise ValueError("Placeholder terrain costs must remain traversable (0..89)")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            OccupancyGrid,
            "/terrain_costmap",
            qos,
        )
        self._message = self._build_message()
        self.create_timer(1.0 / publish_rate, self._publish)
        self._publish()
        self.get_logger().warning(
            "Publishing the static Version 1 integration cost map; "
            "replace this node with Person 3's tide-aware map"
        )

    def _build_message(self) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.frame_id = self._frame
        message.info.resolution = self._resolution
        message.info.width = self._width
        message.info.height = self._height
        message.info.origin.position.x = self._origin_x
        message.info.origin.position.y = self._origin_y
        message.info.origin.orientation.w = 1.0

        costs: list[int] = []
        for _grid_y in range(self._height):
            for grid_x in range(self._width):
                x_m = self._origin_x + (grid_x + 0.5) * self._resolution
                if self._water_start <= x_m < self._water_end:
                    costs.append(self._water_cost)
                elif self._water_end <= x_m < self._mud_end:
                    costs.append(self._mud_cost)
                else:
                    costs.append(self._firm_cost)
        message.data = costs
        return message

    def _publish(self) -> None:
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TerrainCostmapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
