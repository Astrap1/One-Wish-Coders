#!/usr/bin/env python3
"""Route safety-approved motion to the Version 1 wheel or hover actuators."""

from __future__ import annotations

import math

FOLDED = math.pi / 2


class ModeMachine:
    """Implement the public TRACK, TRANSITION and HOVER mobility modes."""

    def __init__(
        self,
        policy: str = "hover_only",
        spinup_s: float = 1.5,
        fold_s: float = 1.6,
        firm_dwell_s: float = 3.0,
        transition_timeout_s: float = 8.0,
        settle_s: float = 0.5,
    ) -> None:
        if policy not in {"hover_only", "terrain_auto"}:
            raise ValueError("mode_policy must be hover_only or terrain_auto")
        self.policy = policy
        self.spinup_s = spinup_s
        self.fold_s = fold_s
        self.firm_dwell_s = firm_dwell_s
        self.transition_timeout_s = transition_timeout_s
        self.settle_s = settle_s
        self.mode = "TRACK"
        self.transition_target: str | None = None
        self.phase_t = 0.0
        self.firm_t = 0.0
        self.hover_enabled = False
        self.legs = 0.0
        self.fault = ""
        if policy == "hover_only":
            self._start_transition("HOVER")

    def _enter(self, mode: str) -> None:
        self.mode = mode
        self.phase_t = 0.0
        self.transition_target = None
        self.fault = ""

    def _start_transition(self, target: str) -> None:
        self.mode = "TRANSITION"
        self.transition_target = target
        self.phase_t = 0.0
        self.fault = ""

    def step(
        self,
        dt: float,
        terrain: str = "FIRM",
        hover_state: str = "",
    ) -> tuple[bool, float]:
        self.phase_t += dt
        self.firm_t = self.firm_t + dt if terrain == "FIRM" else 0.0

        if self.mode == "TRACK":
            self.hover_enabled = False
            self.legs = 0.0
            if self.policy == "terrain_auto" and terrain in {"MUD", "WATER"}:
                self._start_transition("HOVER")

        elif self.mode == "HOVER":
            self.hover_enabled = True
            self.legs = FOLDED
            if self.policy == "terrain_auto" and self.firm_t >= self.firm_dwell_s:
                self._start_transition("TRACK")

        elif self.transition_target == "HOVER":
            self.hover_enabled = True
            if self.phase_t >= self.spinup_s:
                self.legs = FOLDED
            if (
                self.phase_t >= self.spinup_s + self.fold_s
                and hover_state == "HOVER"
            ):
                self._enter("HOVER")
                self.hover_enabled = True
                self.legs = FOLDED
            elif self.phase_t >= self.transition_timeout_s:
                self.fault = "hover_not_ready"

        elif self.transition_target == "TRACK":
            self.legs = 0.0
            if self.phase_t >= self.fold_s:
                self.hover_enabled = False
            if (
                self.phase_t >= self.fold_s + self.settle_s
                and hover_state == "OFF"
            ):
                self._enter("TRACK")
                self.hover_enabled = False
                self.legs = 0.0
            elif self.phase_t >= self.transition_timeout_s:
                self.fault = "wheel_settle_timeout"

        return self.hover_enabled, self.legs

    @property
    def drive_mode(self) -> str | None:
        return self.mode if self.mode in {"TRACK", "HOVER"} and not self.fault else None


class Battery:
    """Simple declared energy model for the simulation demonstration."""

    def __init__(
        self,
        capacity_wh: float = 500.0,
        idle_w: float = 20.0,
        lift_fan_w: float = 300.0,
        thrust_w_per_n: float = 7.0,
        wheels_w: float = 60.0,
        glide_b1: float = 5.0,
        glide_b2: float = 6.0,
        initial_percent: float = 100.0,
    ) -> None:
        self.cap_j = capacity_wh * 3600.0
        self.idle_w = idle_w
        self.lift_w = lift_fan_w
        self.k_thrust = thrust_w_per_n
        self.wheels_w = wheels_w
        self.b1 = glide_b1
        self.b2 = glide_b2
        self.percent = initial_percent

    def step(
        self,
        dt: float,
        hover_enabled: bool,
        mode: str | None,
        v_cmd: float,
        w_cmd: float,
    ) -> tuple[float, float]:
        power = self.idle_w
        if hover_enabled:
            power += self.lift_w
        if mode == "HOVER":
            speed = abs(v_cmd)
            thrust = (
                self.b1 * speed
                + self.b2 * speed * speed
                + 20.0 * abs(w_cmd)
            )
            power += self.k_thrust * thrust
        elif mode == "TRACK" and (
            abs(v_cmd) > 1e-3 or abs(w_cmd) > 1e-3
        ):
            power += self.wheels_w
        self.percent = max(
            0.0,
            self.percent - 100.0 * power * dt / self.cap_j,
        )
        return self.percent, power


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float64, String

    from tidal_vehicle_interfaces.msg import TerrainState, VehicleHealth

    class VehicleMobility(Node):
        def __init__(self) -> None:
            super().__init__("vehicle_mobility")
            parameter = self.declare_parameter
            policy = str(parameter("mode_policy", "hover_only").value)
            self.cmd_timeout = float(parameter("cmd_timeout_s", 0.5).value)
            self.max_v = float(parameter("max_speed_mps", 2.5).value)
            self.max_w = float(parameter("max_yaw_rate_rps", 1.0).value)
            self.pub_terrain = bool(
                parameter("publish_terrain_state", True).value
            )
            self.frame = str(parameter("map_frame", "map").value)
            parameter("mobility_health_percent", 100.0)
            parameter("link_ok", True)
            parameter("payload_secured", True)
            parameter("fault", "")
            telemetry_rate = float(
                parameter("telemetry_rate_hz", 10.0).value
            )
            if telemetry_rate <= 0.0:
                raise ValueError("telemetry_rate_hz must be positive")
            self.modes = ModeMachine(
                policy=policy,
                spinup_s=float(parameter("spinup_s", 1.5).value),
                fold_s=float(parameter("fold_s", 1.6).value),
                firm_dwell_s=float(parameter("firm_dwell_s", 3.0).value),
                transition_timeout_s=float(
                    parameter("transition_timeout_s", 8.0).value
                ),
                settle_s=float(parameter("settle_s", 0.5).value),
            )
            self.battery = Battery(
                capacity_wh=float(parameter("battery_capacity_wh", 500.0).value),
                idle_w=float(parameter("idle_w", 20.0).value),
                lift_fan_w=float(parameter("lift_fan_w", 300.0).value),
                thrust_w_per_n=float(
                    parameter("thrust_w_per_n", 7.0).value
                ),
                wheels_w=float(parameter("wheels_w", 60.0).value),
                initial_percent=float(
                    parameter("initial_battery_percent", 100.0).value
                ),
            )
            self.cmd = Twist()
            self.cmd_stamp = None
            self.terrain = "FIRM"
            self.hover_state = ""
            self._last_mode = ""

            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
            self.create_subscription(
                String,
                "/vehicle/terrain_truth",
                lambda message: setattr(self, "terrain", message.data),
                10,
            )
            self.create_subscription(
                String,
                "/vehicle/hover_state",
                lambda message: setattr(self, "hover_state", message.data),
                10,
            )
            self.pub_hover = self.create_publisher(
                Twist,
                "/vehicle/cmd_vel_hover",
                10,
            )
            self.pub_wheels = self.create_publisher(
                Twist,
                "/vehicle/cmd_vel_wheels",
                10,
            )
            self.pub_enable = self.create_publisher(
                Bool,
                "/vehicle/hover_enabled",
                10,
            )
            self.pub_legs = self.create_publisher(
                Float64,
                "/vehicle/legs_cmd",
                10,
            )
            self.pub_mode = self.create_publisher(
                String,
                "/vehicle/mode",
                10,
            )
            self.pub_health = self.create_publisher(
                VehicleHealth,
                "/vehicle_health",
                10,
            )
            self.pub_tstate = self.create_publisher(
                TerrainState,
                "/terrain_state",
                10,
            )
            self.dt = 0.05
            self.create_timer(self.dt, self.tick)
            self.create_timer(1.0 / telemetry_rate, self.publish_health)
            self.get_logger().info(f"mobility: mode_policy={policy}")

        def on_cmd(self, message: Twist) -> None:
            self.cmd = message
            self.cmd_stamp = self.get_clock().now()

        def current_cmd(self) -> tuple[float, float]:
            if self.cmd_stamp is None:
                return 0.0, 0.0
            age = (
                self.get_clock().now() - self.cmd_stamp
            ).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                return 0.0, 0.0
            linear = max(
                -self.max_v,
                min(self.max_v, self.cmd.linear.x),
            )
            angular = max(
                -self.max_w,
                min(self.max_w, self.cmd.angular.z),
            )
            return linear, angular

        def tick(self) -> None:
            enabled, legs = self.modes.step(
                self.dt,
                self.terrain,
                self.hover_state,
            )
            self.pub_enable.publish(Bool(data=enabled))
            self.pub_legs.publish(Float64(data=legs))
            linear, angular = self.current_cmd()
            command = Twist()
            command.linear.x = linear
            command.angular.z = angular
            stop = Twist()
            drive_mode = self.modes.drive_mode
            self.pub_hover.publish(
                command if drive_mode == "HOVER" else stop
            )
            self.pub_wheels.publish(
                command if drive_mode == "TRACK" else stop
            )
            self.pub_mode.publish(String(data=self.modes.mode))
            self.battery.step(
                self.dt,
                enabled,
                drive_mode,
                linear if drive_mode else 0.0,
                angular if drive_mode else 0.0,
            )
            if self.modes.mode != self._last_mode:
                self.get_logger().info(
                    f"Mobility mode: {self.modes.mode}"
                )
                self._last_mode = self.modes.mode

        def publish_health(self) -> None:
            now = self.get_clock().now().to_msg()
            health = VehicleHealth()
            health.header.stamp = now
            health.header.frame_id = "base_link"
            health.battery_percent = float(self.battery.percent)
            health.mobility_health_percent = float(
                self.get_parameter("mobility_health_percent").value
            )
            health.link_ok = bool(self.get_parameter("link_ok").value)
            health.payload_secured = bool(
                self.get_parameter("payload_secured").value
            )
            injected_fault = str(self.get_parameter("fault").value)
            health.fault = injected_fault or self.modes.fault
            self.pub_health.publish(health)
            if self.pub_terrain:
                terrain = TerrainState()
                terrain.header.stamp = now
                terrain.header.frame_id = self.frame
                terrain.tide_state = "low"
                terrain.tide_risk = 0.0
                terrain.water_level_m = 0.0
                terrain.tide_rate_m_per_minute = 0.0
                terrain.seconds_until_corridor_unsafe = -1.0
                terrain.corridor_traversable = True
                self.pub_tstate.publish(terrain)

    rclpy.init()
    node = VehicleMobility()
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
