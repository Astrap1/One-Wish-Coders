#!/usr/bin/env python3
"""Deterministic tide state and costmap publisher for the demo world."""

from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tidal_vehicle_interfaces.msg import TerrainState


# Known static collision footprints from tidal_corridor.sdf: (x, y, radius_m).
# Dynamic or unmodelled detections remain Autonomy-owned LiDAR overlays.
STATIC_OBSTACLES = (
    # Mangroves
    (18.824, -19.0, 0.95), (54.588, -16.0, 0.75),
    (57.941, 17.0, 0.85), (72.471, -18.0, 1.10),
    (14.353, -13.0, 1.30), (81.412, 13.0, 1.05),
    (87.000, -12.0, 1.00), (25.000, -12.0, 0.75),
    (32.000, 14.0, 0.85), (48.000, -18.0, 0.75),
    (66.000, 15.0, 0.85), (94.000, -14.0, 0.95),
    # Rocks
    (-0.176, -9.0, 1.0), (18.824, 10.0, 1.0),
    (33.353, -10.0, 1.0), (36.706, 11.0, 1.0),
    (40.059, -7.0, 1.0), (43.412, 9.0, 1.0),
    (46.765, -12.0, 1.0), (50.118, 8.0, 1.0),
    (53.471, -10.0, 1.0), (57.941, 12.0, 1.0),
    (64.647, -7.0, 1.0), (69.118, 10.0, 1.0),
    (72.471, -11.0, 1.0), (75.824, 8.0, 1.0),
    (80.294, -9.0, 1.0), (85.882, 11.0, 1.0),
    (94.824, 8.0, 1.0), (106.000, -10.0, 1.0),
)


class TideManager(Node):
    def __init__(self) -> None:
        super().__init__("tide_manager")
        self.declare_parameter("scenario_duration_s", 180.0)
        self.declare_parameter("initial_water_level_m", -2.80)
        self.declare_parameter("water_rise_m", 2.80)
        # Tide is a changing terrain input for this air-cushion vehicle, not a
        # generic closure condition.  Keep its descriptive risk below Safety's
        # intervention thresholds unless a separate mission hazard is injected.
        self.declare_parameter("tide_risk_max", 0.50)
        self.declare_parameter("publish_period_s", 0.5)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("map_resolution_m", 1.0)
        self.declare_parameter("map_width_cells", 120)
        self.declare_parameter("map_height_cells", 60)
        self.declare_parameter("map_origin_x_m", -12.0)
        self.declare_parameter("map_origin_y_m", -30.0)
        self.declare_parameter("firm_cost", 10)
        self.declare_parameter("mud_cost", 45)
        self.declare_parameter("water_cost", 55)
        self.declare_parameter("channel_min_x_m", 4.0)
        self.declare_parameter("channel_max_x_m", 100.0)
        self.declare_parameter("channel_min_y_m", -30.0)
        self.declare_parameter("channel_max_y_m", 30.0)
        self.declare_parameter("mud_min_x_m", 7.7320508)
        self.declare_parameter("mud_max_x_m", 96.2679492)
        self.declare_parameter("mud_min_y_m", -30.0)
        self.declare_parameter("mud_max_y_m", 30.0)

        self.duration = float(self.get_parameter("scenario_duration_s").value)
        self.initial_level = float(self.get_parameter("initial_water_level_m").value)
        self.rise = float(self.get_parameter("water_rise_m").value)
        self.tide_risk_max = float(self.get_parameter("tide_risk_max").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.resolution = float(self.get_parameter("map_resolution_m").value)
        self.width = int(self.get_parameter("map_width_cells").value)
        self.height = int(self.get_parameter("map_height_cells").value)
        self.origin_x = float(self.get_parameter("map_origin_x_m").value)
        self.origin_y = float(self.get_parameter("map_origin_y_m").value)
        self.firm_cost = int(self.get_parameter("firm_cost").value)
        self.mud_cost = int(self.get_parameter("mud_cost").value)
        self.water_cost = int(self.get_parameter("water_cost").value)
        self.channel = self._rectangle("channel")
        self.mudflat = self._rectangle("mud")
        # The demo starts at low tide and rises automatically; scenario events
        # can still reset, hold, resume, or restart the progression.
        self.started_at: Optional[float] = self._now()
        self.hold = False
        self.held_fraction = 0.0
        self.last_level = self.initial_level

        state_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(TerrainState, "/terrain_state", state_qos)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/terrain_costmap", state_qos)
        self.event_sub = self.create_subscription(
            String, "/scenario_event", self._event_callback, 10
        )
        self.timer = self.create_timer(
            float(self.get_parameter("publish_period_s").value), self._publish
        )
        self._publish()
        self.get_logger().info(
            "Tide manager ready: automatic low-to-high tide progression is active"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _event_callback(self, message: String) -> None:
        event = message.data.strip().lower()
        if event in {"tide_rise", "rise", "start_tide"}:
            self.started_at = self._now()
            self.hold = False
            self.held_fraction = 0.0
        elif event in {"tide_reset", "reset"}:
            self.started_at = None
            self.hold = False
            self.held_fraction = 0.0
        elif event in {"tide_hold", "hold"}:
            if not self.hold and self.started_at is not None:
                self.held_fraction = self._fraction()
            self.hold = True
        elif event in {"tide_resume", "resume"}:
            if self.hold and self.started_at is not None:
                self.started_at = self._now() - self.held_fraction * self.duration
            self.hold = False

    def _fraction(self) -> float:
        if self.started_at is None or self.hold:
            return 0.0 if self.started_at is None else self.held_fraction
        return min(1.0, max(0.0, (self._now() - self.started_at) / self.duration))

    @property
    def last_level_fraction(self) -> float:
        return min(1.0, max(0.0, (self.last_level - self.initial_level) / self.rise))

    def _publish(self) -> None:
        fraction = self._fraction()
        self.last_level = self.initial_level + self.rise * fraction
        # Rising water changes the cost map and prompts a fresh plan, but mud,
        # shallow water and open water remain traversable in HOVER mode.
        # A later explicit hazard (debris, current, failed lift, or energy
        # margin) is what should cause Safety to hold or return.
        risk = min(self.tide_risk_max, self.tide_risk_max * fraction)
        if fraction <= 0.01:
            tide_state = "low"
        elif fraction < 1.0:
            tide_state = "rising"
        else:
            tide_state = "high"
        state = TerrainState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self.frame_id
        state.tide_state = tide_state
        state.tide_risk = float(risk)
        state.water_level_m = float(self.last_level)
        state.tide_rate_m_per_minute = float(
            0.0 if self.hold or self.started_at is None or fraction >= 1.0
            else self.rise / max(self.duration / 60.0, 1e-6)
        )
        state.seconds_until_corridor_unsafe = -1.0
        state.corridor_traversable = True
        self.state_pub.publish(state)
        self.costmap_pub.publish(self._make_costmap(fraction, risk))

    def _make_costmap(self, fraction: float, risk: float) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame_id
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        values = []
        for row in range(self.height):
            y = self.origin_y + (row + 0.5) * self.resolution
            for col in range(self.width):
                x = self.origin_x + (col + 0.5) * self.resolution
                value = self.firm_cost
                # The water rectangle mirrors TerrainZones; terrain height clips it
                # into a river that expands outward as the level rises.
                # Known static collision footprints are base-map no-go cells;
                # dynamic detections remain an Autonomy-owned LiDAR overlay.
                if self._contains(self.mudflat, x, y):
                    value = min(89, self.mud_cost + int(10 * fraction))
                if (self._contains(self.channel, x, y)
                        and self.last_level >= self._terrain_height(x)):
                    # Open water remains traversable for the air-cushion vehicle.
                    value = min(89, self.water_cost + int(10 * fraction))
                if self._is_static_obstacle(x, y):
                    value = 100
                values.append(value)
        grid.data = values
        return grid

    def _terrain_height(self, x: float) -> float:
        """Surface height of the piecewise-linear tidal valley."""
        if x <= 4.0 or x >= 100.0:
            return 0.0
        if x < 7.7320508:
            return -(x - 4.0) * 0.2679491924
        if x < 50.0:
            return -1.0 - (x - 7.7320508) * (2.0 / 42.2679492)
        if x <= 54.0:
            return -3.0
        if x < 96.2679492:
            return -3.0 + (x - 54.0) * (2.0 / 42.2679492)
        return -1.0 + (x - 96.2679492) * 0.2679491924

    def _rectangle(self, prefix: str) -> tuple[float, float, float, float]:
        return tuple(
            float(self.get_parameter(f"{prefix}_{axis}_m").value)
            for axis in ("min_x", "max_x", "min_y", "max_y")
        )

    @staticmethod
    def _is_static_obstacle(x: float, y: float) -> bool:
        return any(
            (x - obstacle_x) ** 2 + (y - obstacle_y) ** 2 <= radius ** 2
            for obstacle_x, obstacle_y, radius in STATIC_OBSTACLES
        )

    @staticmethod
    def _contains(rectangle: tuple[float, float, float, float], x: float, y: float) -> bool:
        min_x, max_x, min_y, max_y = rectangle
        return min_x <= x <= max_x and min_y <= y <= max_y


def main() -> None:
    rclpy.init()
    node = TideManager()
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
