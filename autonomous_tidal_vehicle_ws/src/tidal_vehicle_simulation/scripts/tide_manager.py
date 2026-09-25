#!/usr/bin/env python3
"""Deterministic tide state and costmap publisher for the demo world."""

import math
from typing import Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tidal_vehicle_interfaces.msg import TerrainState


class TideManager(Node):
    def __init__(self) -> None:
        super().__init__("tide_manager")
        self.declare_parameter("scenario_duration_s", 120.0)
        self.declare_parameter("initial_water_level_m", 0.02)
        self.declare_parameter("water_rise_m", 0.55)
        self.declare_parameter("risk_rate_per_minute", 0.05)
        self.declare_parameter("corridor_unsafe_risk_threshold", 0.85)
        self.declare_parameter("publish_period_s", 0.5)

        self.duration = float(self.get_parameter("scenario_duration_s").value)
        self.initial_level = float(self.get_parameter("initial_water_level_m").value)
        self.rise = float(self.get_parameter("water_rise_m").value)
        self.risk_rate = float(self.get_parameter("risk_rate_per_minute").value)
        self.unsafe_threshold = float(
            self.get_parameter("corridor_unsafe_risk_threshold").value
        )
        # The demo starts at low tide and rises automatically; scenario events
        # can still reset, hold, resume, or restart the progression.
        self.started_at: Optional[float] = self._now()
        self.hold = False
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
        elif event in {"tide_reset", "reset"}:
            self.started_at = None
            self.hold = False
        elif event in {"tide_hold", "hold"}:
            self.hold = True
        elif event in {"tide_resume", "resume"}:
            self.hold = False

    def _fraction(self) -> float:
        if self.started_at is None or self.hold:
            return 0.0 if self.started_at is None else self.last_level_fraction
        return min(1.0, max(0.0, (self._now() - self.started_at) / self.duration))

    @property
    def last_level_fraction(self) -> float:
        return min(1.0, max(0.0, (self.last_level - self.initial_level) / self.rise))

    def _publish(self) -> None:
        fraction = self._fraction()
        self.last_level = self.initial_level + self.rise * fraction
        elapsed_minutes = fraction * self.duration / 60.0
        risk = min(1.0, self.risk_rate * elapsed_minutes + 0.35 * fraction)
        if fraction <= 0.01:
            tide_state = "low"
        elif fraction < 1.0:
            tide_state = "rising"
        else:
            tide_state = "high"
        unsafe = max(0.0, (self.unsafe_threshold - risk) / max(self.risk_rate, 1e-6)) * 60.0

        state = TerrainState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "map"
        state.tide_state = tide_state
        state.tide_risk = float(risk)
        state.water_level_m = float(self.last_level)
        state.tide_rate_m_per_minute = float(self.rise / max(self.duration / 60.0, 1e-6))
        state.seconds_until_corridor_unsafe = float(unsafe if risk < self.unsafe_threshold else 0.0)
        state.corridor_traversable = risk < self.unsafe_threshold
        self.state_pub.publish(state)
        self.costmap_pub.publish(self._make_costmap(fraction, risk))

    def _make_costmap(self, fraction: float, risk: float) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = 1.0
        grid.info.width = 50
        grid.info.height = 50
        grid.info.origin.position.x = -25.0
        grid.info.origin.position.y = -25.0
        grid.info.origin.orientation.w = 1.0
        values = []
        for row in range(50):
            y = row - 24.5
            for col in range(50):
                x = col - 24.5
                # A soft, meandering channel risk band; this is a planning layer,
                # not a replacement for the fixed Gazebo collision heightmap.
                centre = 5.0 * math.sin(y * 0.18) - 2.0
                channel_distance = abs(x - centre)
                wet = channel_distance < 3.0 + 0.8 * math.sin(y * 0.13) ** 2
                value = 20
                if wet:
                    value = 55 + int(35 * fraction)
                if channel_distance < 1.2 + 0.7 * fraction:
                    value = min(100, 70 + int(30 * fraction))
                if risk >= self.unsafe_threshold and wet:
                    value = 100
                values.append(value)
        grid.data = values
        return grid


def main() -> None:
    rclpy.init()
    node = TideManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
