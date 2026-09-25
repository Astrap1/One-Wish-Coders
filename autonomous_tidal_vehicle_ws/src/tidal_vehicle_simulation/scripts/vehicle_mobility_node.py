#!/usr/bin/env python3
"""
vehicle_mobility_node — the vehicle's mobility abstraction (Person 4).

Turns the safety-approved /cmd_vel into commands for the Gazebo vehicle and
reports raw vehicle telemetry.

  /cmd_vel (geometry_msgs/Twist)  ── HOVER mode ──►  /vehicle/cmd_vel_hover  (fans, via AirCushion)
                                  ── GROUND mode ─►  /vehicle/cmd_vel_wheels (wheels, via DiffDrive)

Mode policy (parameter `mode_policy`):
  hover_only    (default) lift fan on and legs folded at start-up; the vehicle
                always drives on its cushion. Simplest, most reliable for the demo.
  terrain_auto  GROUND on firm terrain; switches to HOVER on MUD / WATER and back
                to GROUND after `firm_dwell_s` on firm ground. The transitions are
                scripted sequences (fan spin-up → fold legs, or unfold → fan off).
                NOTE: the HOVER→GROUND transition while moving is not yet
                validated in tests; use hover_only for demos until it is.

While a transition runs, both command outputs are held at zero.

Also publishes:
  /vehicle/mode    (std_msgs/String)  GROUND | TO_HOVER | HOVER | TO_GROUND
  /vehicle_health  (tidal_vehicle_interfaces/VehicleHealth)  raw telemetry:
                   battery from a simple power model (lift fan, thrust, wheels),
                   mobility health, link and payload flags. Fault injection by
                   the evaluation workstream can override via parameters.
  /terrain_state   (tidal_vehicle_interfaces/TerrainState)  OPTIONAL placeholder
                   (publish_terrain_state: true) until the environment
                   workstream's tide manager publishes the real one.

All numbers are stated simulation assumptions (config/vehicle_mobility.yaml),
not measured vehicle data.
"""
import math

FOLDED = math.pi / 2


# ---------------------------------------------------------------------------
# Pure logic (no ROS imports) so it can be unit-tested without a ROS install
# ---------------------------------------------------------------------------
class ModeMachine:
    """GROUND ⇄ HOVER sequencing. step() returns the actuator targets."""

    def __init__(self, policy="hover_only", spinup_s=1.5, fold_s=1.6, firm_dwell_s=3.0):
        self.policy = policy
        self.spinup_s, self.fold_s, self.firm_dwell_s = spinup_s, fold_s, firm_dwell_s
        self.mode = "GROUND"
        self.phase_t = 0.0
        self.firm_t = 0.0
        self.hover_enabled = False
        self.legs = 0.0
        if policy == "hover_only":
            self._enter("TO_HOVER")

    def _enter(self, mode):
        self.mode, self.phase_t = mode, 0.0

    def step(self, dt, terrain="FIRM", hover_state=""):
        self.phase_t += dt
        self.firm_t = self.firm_t + dt if terrain == "FIRM" else 0.0
        if self.mode == "GROUND":
            self.hover_enabled, self.legs = False, 0.0
            if self.policy == "terrain_auto" and terrain in ("MUD", "WATER"):
                self._enter("TO_HOVER")
        elif self.mode == "TO_HOVER":
            self.hover_enabled = True                       # 1) lift fan on
            if self.phase_t >= self.spinup_s:
                self.legs = FOLDED                          # 2) fold legs
            if self.phase_t >= self.spinup_s + self.fold_s and hover_state in ("HOVER", ""):
                self._enter("HOVER")                        # 3) riding on the cushion
        elif self.mode == "HOVER":
            self.hover_enabled, self.legs = True, FOLDED
            if self.policy == "terrain_auto" and self.firm_t >= self.firm_dwell_s:
                self._enter("TO_GROUND")
        elif self.mode == "TO_GROUND":
            self.legs = 0.0                                 # 1) unfold legs (still hovering)
            if self.phase_t >= self.fold_s:
                self.hover_enabled = False                  # 2) lift fan off
            if self.phase_t >= self.fold_s + self.spinup_s:
                self._enter("GROUND")
        return self.hover_enabled, self.legs

    @property
    def drive_mode(self):
        return self.mode if self.mode in ("GROUND", "HOVER") else None


class Battery:
    """Very simple energy model: P = idle + lift fan + thrust + wheels."""

    def __init__(self, capacity_wh=500.0, idle_w=20.0, lift_fan_w=300.0,
                 thrust_w_per_n=7.0, wheels_w=60.0, glide_b1=5.0, glide_b2=6.0,
                 initial_percent=100.0):
        self.cap_j = capacity_wh * 3600.0
        self.idle_w, self.lift_w, self.k_thrust, self.wheels_w = idle_w, lift_fan_w, thrust_w_per_n, wheels_w
        self.b1, self.b2 = glide_b1, glide_b2
        self.percent = initial_percent

    def step(self, dt, hover_enabled, mode, v_cmd, w_cmd):
        p = self.idle_w
        if hover_enabled:
            p += self.lift_w
        if mode == "HOVER":
            v = abs(v_cmd)
            thrust = self.b1 * v + self.b2 * v * v + 20.0 * abs(w_cmd)   # N, both fans
            p += self.k_thrust * thrust
        elif mode == "GROUND" and (abs(v_cmd) > 1e-3 or abs(w_cmd) > 1e-3):
            p += self.wheels_w
        self.percent = max(0.0, self.percent - 100.0 * p * dt / self.cap_j)
        return self.percent, p


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------
def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, Float64, String
    from tidal_vehicle_interfaces.msg import TerrainState, VehicleHealth

    class VehicleMobility(Node):
        def __init__(self):
            super().__init__("vehicle_mobility")
            p = self.declare_parameter
            policy = p("mode_policy", "hover_only").value
            self.cmd_timeout = p("cmd_timeout_s", 0.5).value
            self.max_v = p("max_speed_mps", 2.5).value
            self.max_w = p("max_yaw_rate_rps", 1.0).value
            self.pub_terrain = p("publish_terrain_state", True).value
            self.frame = p("map_frame", "map").value
            # fault-injection hooks (evaluation workstream may set these at runtime)
            self.mobility_health = p("mobility_health_percent", 100.0).value
            self.link_ok = p("link_ok", True).value
            self.payload_secured = p("payload_secured", True).value
            self.fault = p("fault", "").value
            self.modes = ModeMachine(policy, p("spinup_s", 1.5).value, p("fold_s", 1.6).value,
                                     p("firm_dwell_s", 3.0).value)
            self.battery = Battery(p("battery_capacity_wh", 500.0).value, p("idle_w", 20.0).value,
                                   p("lift_fan_w", 300.0).value, p("thrust_w_per_n", 7.0).value,
                                   p("wheels_w", 60.0).value,
                                   initial_percent=p("initial_battery_percent", 100.0).value)
            self.cmd = Twist()
            self.cmd_stamp = None
            self.terrain, self.hover_state = "FIRM", ""

            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
            self.create_subscription(String, "/vehicle/terrain_truth",
                                     lambda m: setattr(self, "terrain", m.data), 10)
            self.create_subscription(String, "/vehicle/hover_state",
                                     lambda m: setattr(self, "hover_state", m.data), 10)
            self.pub_hover = self.create_publisher(Twist, "/vehicle/cmd_vel_hover", 10)
            self.pub_wheels = self.create_publisher(Twist, "/vehicle/cmd_vel_wheels", 10)
            self.pub_enable = self.create_publisher(Bool, "/vehicle/hover_enabled", 10)
            self.pub_legs = self.create_publisher(Float64, "/vehicle/legs_cmd", 10)
            self.pub_mode = self.create_publisher(String, "/vehicle/mode", 10)
            self.pub_health = self.create_publisher(VehicleHealth, "/vehicle_health", 10)
            self.pub_tstate = self.create_publisher(TerrainState, "/terrain_state", 10)
            self.dt = 0.05
            self.create_timer(self.dt, self.tick)
            self.create_timer(0.5, self.publish_health)
            self.get_logger().info(f"mobility: mode_policy={policy}")

        def on_cmd(self, msg):
            self.cmd = msg
            self.cmd_stamp = self.get_clock().now()

        def current_cmd(self):
            if self.cmd_stamp is None:
                return 0.0, 0.0
            age = (self.get_clock().now() - self.cmd_stamp).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                return 0.0, 0.0                                  # stale → stop
            v = max(-self.max_v, min(self.max_v, self.cmd.linear.x))
            w = max(-self.max_w, min(self.max_w, self.cmd.angular.z))
            return v, w

        def tick(self):
            en, legs = self.modes.step(self.dt, self.terrain, self.hover_state)
            self.pub_enable.publish(Bool(data=en))
            self.pub_legs.publish(Float64(data=legs))
            v, w = self.current_cmd()
            out, zero = Twist(), Twist()
            out.linear.x, out.angular.z = v, w
            dm = self.modes.drive_mode
            self.pub_hover.publish(out if dm == "HOVER" else zero)
            self.pub_wheels.publish(out if dm == "GROUND" else zero)
            self.pub_mode.publish(String(data=self.modes.mode))
            self.battery.step(self.dt, en, dm, v if dm else 0.0, w if dm else 0.0)

        def publish_health(self):
            now = self.get_clock().now().to_msg()
            h = VehicleHealth()
            h.header.stamp, h.header.frame_id = now, "base_link"
            h.battery_percent = float(self.battery.percent)
            h.mobility_health_percent = float(self.mobility_health)
            h.link_ok, h.payload_secured, h.fault = bool(self.link_ok), bool(self.payload_secured), self.fault
            self.pub_health.publish(h)
            if self.pub_terrain:                  # placeholder until the tide manager exists
                t = TerrainState()
                t.header.stamp, t.header.frame_id = now, self.frame
                t.tide_state, t.tide_risk, t.water_level_m = "low", 0.0, 0.0
                t.tide_rate_m_per_minute, t.seconds_until_corridor_unsafe = 0.0, -1.0
                t.corridor_traversable = True
                self.pub_tstate.publish(t)

    rclpy.init()
    node = VehicleMobility()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
