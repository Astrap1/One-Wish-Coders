#!/usr/bin/env python3
"""Route safety-approved motion to the vehicle's hover fans or ground gear.

Version 1 (gear: wheels) drives four wheels on swing-up legs; Version 2
(gear: tracks) drives two retractable tracks and shares the vehicle's weight
with the air cushion in TRACK mode (AGENTS.md, "Vehicle Version 2").
"""

from __future__ import annotations

import math

FOLDED = math.pi / 2


class ModeMachine:
    """Implement the public TRACK, TRANSITION and HOVER mobility modes.

    gear="wheels" (Version 1): the legs fold after the lift fan spins up and
    HOVER needs the plugin's HOVER state; the return to TRACK waits for the
    plugin's OFF state plus a settling delay.

    gear="tracks" (Version 2), following AGENTS.md:
      TRACK -> HOVER: stop, full lift, wait for the plugin's HOVER state, then
        retract the tracks; HOVER starts once the measured track position is
        retracted.
      HOVER -> TRACK: stop, deploy the tracks while still hovering, lower the
        lift to the TRACK load share, then wait until the measured cushion gap
        has settled at the on-track height for settle_s.
      Mode selection (terrain_auto): TRACK on firm ground (after firm_dwell_s)
        or on slopes steeper than hover_slope_limit_deg; HOVER on MUD / WATER
        below that limit. In TRACK mode the cushion carries track_share_firm of
        the weight, or track_share_slope on slopes.
    """

    def __init__(
        self,
        policy: str = "hover_only",
        spinup_s: float = 1.5,
        fold_s: float = 1.6,
        firm_dwell_s: float = 3.0,
        transition_timeout_s: float = 8.0,
        settle_s: float = 0.5,
        gear: str = "wheels",
        retract_position: float = FOLDED,
        gear_tolerance: float = 0.01,
        track_share_firm: float = 0.2,
        track_share_slope: float = 0.6,
        hover_slope_limit_deg: float = 6.0,
        settle_gap_m: float = 0.03,
        settle_gap_tolerance_m: float = 0.008,
    ) -> None:
        if policy not in {"hover_only", "terrain_auto"}:
            raise ValueError("mode_policy must be hover_only or terrain_auto")
        if gear not in {"wheels", "tracks"}:
            raise ValueError("gear must be wheels or tracks")
        self.policy = policy
        self.gear = gear
        self.spinup_s = spinup_s
        self.fold_s = fold_s
        self.firm_dwell_s = firm_dwell_s
        self.transition_timeout_s = transition_timeout_s
        self.settle_s = settle_s
        self.retract_position = retract_position
        self.gear_tolerance = gear_tolerance
        self.track_share_firm = track_share_firm
        self.track_share_slope = track_share_slope
        self.hover_slope_limit_deg = hover_slope_limit_deg
        self.settle_gap_m = settle_gap_m
        self.settle_gap_tolerance_m = settle_gap_tolerance_m
        self.mode = "TRACK"
        self.transition_target: str | None = None
        self.phase_t = 0.0
        self.firm_t = 0.0
        self.settled_t = 0.0
        self.hover_enabled = False
        self.legs = 0.0
        self.lift_share: float | None = None     # None = normal full hover law
        self.fault = ""
        self._stage = 0
        if gear == "tracks" and policy == "terrain_auto":
            self.hover_enabled = True             # TRACK start: cushion shares the load
            self.lift_share = track_share_firm
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
        self.settled_t = 0.0
        self._stage = 0
        self.fault = ""

    def _gear_at(self, gear_pos: float | None, target: float, fallback: bool) -> bool:
        """Measured gear position at target; without a measurement, `fallback`."""
        if gear_pos is None:
            return fallback
        return abs(gear_pos - target) <= self.gear_tolerance

    def track_share(self, slope_deg: float) -> float:
        steep = slope_deg > self.hover_slope_limit_deg
        return self.track_share_slope if steep else self.track_share_firm

    def step(
        self,
        dt: float,
        terrain: str = "FIRM",
        hover_state: str = "",
        gap: float | None = None,
        gear_pos: float | None = None,
        slope_deg: float = 0.0,
    ) -> tuple[bool, float]:
        self.phase_t += dt
        self.firm_t = self.firm_t + dt if terrain == "FIRM" else 0.0
        if self.gear == "tracks":
            return self._step_tracks(dt, terrain, hover_state, gap, gear_pos, slope_deg)

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

    def _step_tracks(
        self,
        dt: float,
        terrain: str,
        hover_state: str,
        gap: float | None,
        gear_pos: float | None,
        slope_deg: float,
    ) -> tuple[bool, float]:
        steep = slope_deg > self.hover_slope_limit_deg
        if self.mode == "TRACK":
            self.legs = 0.0
            self.lift_share = self.track_share(slope_deg)
            self.hover_enabled = self.lift_share > 0.0
            if (self.policy == "terrain_auto" and terrain in {"MUD", "WATER"}
                    and not steep):
                self._start_transition("HOVER")

        elif self.mode == "HOVER":
            self.hover_enabled = True
            self.lift_share = None
            self.legs = self.retract_position
            if self.policy == "terrain_auto" and (self.firm_t >= self.firm_dwell_s or steep):
                self._start_transition("TRACK")

        elif self.transition_target == "HOVER":
            # 1) full lift; 2) hover-ready; 3) retract the tracks
            self.hover_enabled = True
            self.lift_share = None
            if self._stage == 0 and hover_state == "HOVER":
                self._stage = 1
                self.legs = self.retract_position
                self.settled_t = 0.0
            if self._stage == 1:
                self.settled_t += dt
                if self._gear_at(gear_pos, self.retract_position,
                                 self.settled_t >= self.fold_s):
                    self._enter("HOVER")
                    return self.hover_enabled, self.legs
            if self.phase_t >= self.transition_timeout_s:
                self.fault = "hover_not_ready" if self._stage == 0 else "track_deployment_fault"

        elif self.transition_target == "TRACK":
            # 1) deploy the tracks while hovering; 2) lower the lift to the
            # TRACK share; 3) wait until the measured gap is settled on the tracks
            self.hover_enabled = True
            self.legs = 0.0
            if self._stage == 0:
                self.lift_share = None
                if self._gear_at(gear_pos, 0.0, self.phase_t >= self.fold_s):
                    self._stage = 1
                    self.settled_t = 0.0
            if self._stage == 1:
                self.lift_share = self.track_share(slope_deg)
                on_tracks = (gap is not None
                             and abs(gap - self.settle_gap_m) <= self.settle_gap_tolerance_m)
                self.settled_t = self.settled_t + dt if on_tracks else 0.0
                if self.settled_t >= self.settle_s:
                    self._enter("TRACK")
                    self.hover_enabled = self.lift_share > 0.0
                    return self.hover_enabled, self.legs
            if self.phase_t >= self.transition_timeout_s:
                self.fault = "track_deployment_fault" if self._stage == 0 else "track_settle_timeout"

        return self.hover_enabled, self.legs

    @property
    def drive_mode(self) -> str | None:
        return self.mode if self.mode in {"TRACK", "HOVER"} and not self.fault else None


def terrain_from_costs(costs: list[int], firm_max: int = 19, no_go_min: int = 90) -> str | None:
    """Mobility terrain for the shared cost bands (AGENTS.md): FIRM when every
    sampled cell is firm (0-19), MUD when any is hover terrain (20-89). No-go
    (90-100) and unknown (-1) cells are left to the planner and ignored.
    None when nothing usable was sampled."""
    usable = [c for c in costs if 0 <= c < no_go_min]
    if not usable:
        return None
    return "FIRM" if max(usable) <= firm_max else "MUD"


def sample_costmap(info: tuple, data, x: float, y: float, yaw: float,
                   lookahead_m: float, step_m: float = 0.5) -> list[int]:
    """Costs from the vehicle position out to lookahead_m along its heading.
    info = (resolution, width, height, origin_x, origin_y)."""
    res, width, height, ox, oy = info
    out = []
    n = int(lookahead_m / step_m) + 1
    for k in range(n):
        d = k * step_m
        col = int(math.floor((x + d * math.cos(yaw) - ox) / res))
        row = int(math.floor((y + d * math.sin(yaw) - oy) / res))
        if 0 <= col < width and 0 <= row < height:
            out.append(int(data[row * width + col]))
    return out


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
        lift_fraction: float = 1.0,
    ) -> tuple[float, float]:
        power = self.idle_w
        if hover_enabled:
            power += self.lift_w * lift_fraction
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
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
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
            gear = str(parameter("gear", "wheels").value)
            self.max_track_v = float(
                parameter("track_max_speed_mps", self.max_v).value
            )
            self.gear_joints = list(
                parameter("gear_joints", ["unused"]).value
            )
            self.modes = ModeMachine(
                policy=policy,
                spinup_s=float(parameter("spinup_s", 1.5).value),
                fold_s=float(parameter("fold_s", 1.6).value),
                firm_dwell_s=float(parameter("firm_dwell_s", 3.0).value),
                transition_timeout_s=float(
                    parameter("transition_timeout_s", 8.0).value
                ),
                settle_s=float(parameter("settle_s", 0.5).value),
                gear=gear,
                retract_position=float(
                    parameter("retract_position", FOLDED).value
                ),
                track_share_firm=float(
                    parameter("track_share_firm", 0.2).value
                ),
                track_share_slope=float(
                    parameter("track_share_slope", 0.6).value
                ),
                hover_slope_limit_deg=float(
                    parameter("hover_slope_limit_deg", 6.0).value
                ),
                settle_gap_m=float(parameter("settle_gap_m", 0.03).value),
                settle_gap_tolerance_m=float(
                    parameter("settle_gap_tolerance_m", 0.008).value
                ),
            )
            self.battery = Battery(
                capacity_wh=float(parameter("battery_capacity_wh", 500.0).value),
                idle_w=float(parameter("idle_w", 20.0).value),
                lift_fan_w=float(parameter("lift_fan_w", 300.0).value),
                thrust_w_per_n=float(
                    parameter("thrust_w_per_n", 7.0).value
                ),
                wheels_w=float(parameter("wheels_w", 60.0).value),
                glide_b1=float(parameter("glide_b1", 5.0).value),
                glide_b2=float(parameter("glide_b2", 6.0).value),
                initial_percent=float(
                    parameter("initial_battery_percent", 100.0).value
                ),
            )
            self.cmd = Twist()
            self.cmd_stamp = None
            self.terrain = "FIRM"
            self.hover_state = ""
            self.gap: float | None = None
            self.gear_pos: float | None = None
            self.slope_deg = 0.0
            self.pose = (0.0, 0.0, 0.0)
            self.costmap = None
            # terrain for mode selection: "costmap" (look ahead on /terrain_costmap,
            # AGENTS.md cost bands) or "truth" (Gazebo ground truth under the vehicle)
            self.terrain_source = str(parameter("terrain_source", "truth").value)
            self.lookahead_m = float(parameter("terrain_lookahead_m", 2.5).value)
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
            self.create_subscription(
                Float64,
                "/vehicle/cushion_gap",
                lambda message: setattr(self, "gap", message.data),
                10,
            )
            self.create_subscription(
                JointState, "/joint_states", self.on_joints, 10
            )
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            if self.terrain_source == "costmap":
                self.create_subscription(
                    OccupancyGrid, "/terrain_costmap", self.on_costmap, 10
                )
            self.pub_hover = self.create_publisher(
                Twist,
                "/vehicle/cmd_vel_hover",
                10,
            )
            # Version 1: /vehicle/cmd_vel_wheels and /vehicle/legs_cmd;
            # Version 2: /vehicle/cmd_vel_tracks and /vehicle/tracks_cmd.
            self.pub_wheels = self.create_publisher(
                Twist,
                str(parameter("ground_cmd_topic", "/vehicle/cmd_vel_wheels").value),
                10,
            )
            self.pub_enable = self.create_publisher(
                Bool,
                "/vehicle/hover_enabled",
                10,
            )
            self.pub_legs = self.create_publisher(
                Float64,
                str(parameter("gear_cmd_topic", "/vehicle/legs_cmd").value),
                10,
            )
            self.pub_share = (
                self.create_publisher(Float64, "/vehicle/lift_share", 10)
                if gear == "tracks" else None
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

        def on_joints(self, message: JointState) -> None:
            positions = [
                p for n, p in zip(message.name, message.position)
                if n in self.gear_joints
            ]
            if positions:
                # the gear is ready only when every joint is: report the worst
                target = self.modes.legs
                self.gear_pos = max(positions, key=lambda p: abs(p - target))

        def on_costmap(self, message: OccupancyGrid) -> None:
            i = message.info
            self.costmap = ((i.resolution, i.width, i.height,
                             i.origin.position.x, i.origin.position.y),
                            list(message.data))

        def mobility_terrain(self) -> str:
            if self.terrain_source == "costmap" and self.costmap is not None:
                x, y, yaw = self.pose
                costs = sample_costmap(self.costmap[0], self.costmap[1], x, y, yaw,
                                       self.lookahead_m)
                terrain = terrain_from_costs(costs)
                if terrain is not None:
                    return terrain
            return self.terrain

        def on_odom(self, message: Odometry) -> None:
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = (p.x, p.y, yaw)
            # tilt of the body z axis from vertical = slope under the vehicle
            cos_tilt = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            self.slope_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))

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
            limit = self.max_track_v if self.modes.mode == "TRACK" else self.max_v
            linear = max(
                -limit,
                min(limit, self.cmd.linear.x),
            )
            angular = max(
                -self.max_w,
                min(self.max_w, self.cmd.angular.z),
            )
            return linear, angular

        def tick(self) -> None:
            enabled, legs = self.modes.step(
                self.dt,
                self.mobility_terrain(),
                self.hover_state,
                gap=self.gap,
                gear_pos=self.gear_pos,
                slope_deg=self.slope_deg,
            )
            share = self.modes.lift_share
            if self.pub_share is not None:
                # NaN switches the plugin back to its normal full-hover law
                self.pub_share.publish(
                    Float64(data=float("nan") if share is None else share)
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
                lift_fraction=1.0 if share is None else share,
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
