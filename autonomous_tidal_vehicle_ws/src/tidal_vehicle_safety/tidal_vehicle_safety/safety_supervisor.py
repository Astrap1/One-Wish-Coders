"""ROS 2 command gate for a terrain-aware tidal-vehicle return policy."""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from tidal_vehicle_interfaces.msg import SafetyStatus, TerrainState, VehicleHealth

from .mission_phase import MissionPhase
from .policy import CAUTION, CRUISE, HOLD, RETURN, Decision, PolicyConfig, PolicySnapshot, evaluate
from .return_estimator import (
    GridCostmap,
    ReturnEstimate,
    ReturnEstimatorConfig,
    estimate_return,
    path_ends_at_home,
)


class SafetySupervisor(Node):
    """Allow only safe proposed commands to reach the simulated vehicle."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        self._declare_parameters()
        self._phase = MissionPhase.PRELAUNCH
        self._state = HOLD
        self._reason = "Awaiting a mission goal and current telemetry."
        self._health: Optional[VehicleHealth] = None
        self._terrain: Optional[TerrainState] = None
        self._costmap: Optional[GridCostmap] = None
        self._planned_path_received_at: Optional[float] = None
        self._return_path_received_at: Optional[float] = None
        self._costmap_changed_at: Optional[float] = None
        self._return_requested_at: Optional[float] = None
        self._last_health_at: Optional[float] = None
        self._last_terrain_at: Optional[float] = None
        self._last_autonomous_command_at: Optional[float] = None
        self._last_remote_command_at: Optional[float] = None
        self._remote_enabled = False
        self._return_path_points: list[tuple[float, float]] = []
        self._last_costmap_signature: Optional[tuple[object, ...]] = None
        self._estimate = ReturnEstimate(False, "Awaiting a return route and terrain cost map.")

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._status_pub = self.create_publisher(SafetyStatus, "/safety_status", status_qos)
        self.create_subscription(Twist, "/cmd_vel_proposed", self._on_proposed_command, 10)
        remote_mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/operator_remote_enabled", self._on_remote_mode,
                                 remote_mode_qos)
        self.create_subscription(Twist, "/operator_cmd_vel", self._on_remote_command, 10)
        self.create_subscription(VehicleHealth, "/vehicle_health", self._on_health, 10)
        self.create_subscription(TerrainState, "/terrain_state", self._on_terrain, 10)
        self.create_subscription(OccupancyGrid, "/terrain_costmap", self._on_costmap, 10)
        self.create_subscription(PoseStamped, "/mission_goal", self._on_mission_goal, 10)
        self.create_subscription(Path, "/planned_path", self._on_planned_path, 10)
        self.create_subscription(Path, "/return_path", self._on_return_path, 10)
        self.create_subscription(
            String, "/mission_event", self._on_mission_event, status_qos
        )
        self.create_subscription(String, "/scenario_event", self._on_scenario_event, 10)
        self.create_timer(0.1, self._on_timer)

    def _declare_parameters(self) -> None:
        defaults = {
            "command_timeout_s": 0.5,
            "telemetry_timeout_s": 1.0,
            "path_replan_timeout_s": 2.0,
            "return_path_timeout_s": 2.0,
            "caution_tide_risk": 0.60,
            "return_tide_risk": 0.75,
            "hold_tide_risk": 0.85,
            "caution_return_margin_percent": 20.0,
            "return_margin_percent": 12.0,
            "hold_return_margin_percent": 0.0,
            "caution_mobility_health_percent": 75.0,
            "return_mobility_health_percent": 60.0,
            "hold_mobility_health_percent": 35.0,
            "tide_return_time_buffer_s": 30.0,
            "cruise_linear_speed_limit_mps": 2.0,
            "caution_linear_speed_limit_mps": 0.8,
            "return_linear_speed_limit_mps": 1.0,
            "remote_linear_speed_limit_mps": 0.8,
            "remote_angular_speed_limit_radps": 0.6,
            "home_x_m": 0.0,
            "home_y_m": 0.0,
            "home_yaw_rad": 0.0,
            "home_tolerance_m": 2.0,
            "route_sample_spacing_m": 0.5,
            "no_go_cost": 90,
            "track_mode_enabled": False,
            "track_cost_max": 19,
            "elevated_hover_cost_min": 60,
            "track_energy_percent_per_m": 0.025,
            "hover_energy_percent_per_m": 0.05,
            "track_nominal_speed_mps": 1.0,
            "hover_nominal_speed_mps": 0.7,
            "elevated_hover_energy_multiplier": 1.25,
            "elevated_hover_speed_multiplier": 0.70,
            "mode_transition_time_s": 5.0,
            "mode_transition_energy_percent": 1.0,
            "terrain_cost_energy_weight": 0.5,
            "mobility_degradation_weight": 0.7,
            "minimum_return_buffer_percent": 8.0,
            "return_energy_contingency_ratio": 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_health(self, message: VehicleHealth) -> None:
        self._health = message
        self._last_health_at = self._now_s()

    def _on_terrain(self, message: TerrainState) -> None:
        self._terrain = message
        self._last_terrain_at = self._now_s()

    def _on_costmap(self, message: OccupancyGrid) -> None:
        now = self._now_s()
        info = message.info
        self._costmap = GridCostmap(
            width=info.width,
            height=info.height,
            resolution_m=info.resolution,
            origin_x_m=info.origin.position.x,
            origin_y_m=info.origin.position.y,
            data=tuple(message.data),
        )
        signature = (
            info.width,
            info.height,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            tuple(message.data),
        )
        if signature != self._last_costmap_signature:
            self._last_costmap_signature = signature
            self._costmap_changed_at = now

    def _on_mission_goal(self, _message: PoseStamped) -> None:
        if self._phase is MissionPhase.PRELAUNCH:
            self._phase = MissionPhase.OUTBOUND
            self._planned_path_received_at = None
            self._last_autonomous_command_at = None
            self._reason = "Mission goal accepted; validating operating margin."

    def _on_planned_path(self, _message: Path) -> None:
        self._planned_path_received_at = self._now_s()

    def _on_return_path(self, message: Path) -> None:
        self._return_path_points = [
            (pose.pose.position.x, pose.pose.position.y) for pose in message.poses
        ]
        self._return_path_received_at = self._now_s()
        self._refresh_return_estimate()

    def _on_mission_event(self, message: String) -> None:
        event = message.data.strip().lower()
        if event == "delivery_confirmed" and self._phase is MissionPhase.OUTBOUND:
            self._phase = MissionPhase.DELIVERED
            self._request_return("Payload delivery confirmed; returning to HOME.")
        elif event in {"mission_complete", "mission_reset"}:
            self._reset("Mission reset; awaiting a new goal.")

    def _on_scenario_event(self, message: String) -> None:
        event = message.data.strip().lower()
        if event == "operator_abort":
            self._remote_enabled = False
            self._last_remote_command_at = None
            if self._phase is not MissionPhase.PRELAUNCH:
                self._request_return("Operator requested a controlled return.")
            else:
                self._state = HOLD
                self._reason = "Operator abort received before mission start."
                self._publish_stop()
                self._publish_status(False)
        elif event == "reset":
            self._reset("Scenario reset; awaiting a new goal.")

    def _on_proposed_command(self, command: Twist) -> None:
        self._last_autonomous_command_at = self._now_s()
        if not self._remote_enabled:
            self._apply_active_command(command)

    def _on_remote_mode(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled == self._remote_enabled:
            return
        self._remote_enabled = enabled
        self._last_remote_command_at = None
        self._last_autonomous_command_at = None
        self._publish_stop()
        self._state = HOLD
        self._reason = (
            "Remote control enabled; awaiting a hold-to-run operator command."
            if enabled else "Autonomous control enabled; awaiting a fresh autonomy command."
        )
        self._publish_status(False)

    def _on_remote_command(self, command: Twist) -> None:
        self._last_remote_command_at = self._now_s()
        if self._remote_enabled:
            self._apply_active_command(command)

    def _on_timer(self) -> None:
        decision = self._active_decision()
        self._apply_decision(decision)
        if decision.state == HOLD:
            self._publish_stop()

    def _apply_active_command(self, command: Twist) -> None:
        decision = self._active_decision()
        self._apply_decision(decision)
        if decision.state == HOLD:
            self._publish_stop()
            return
        self._cmd_pub.publish(self._limited_command(command, decision.state,
                                                    remote=self._remote_enabled))

    def _active_decision(self) -> Decision:
        """Keep manual control independent from mission-policy return decisions.

        Remote mode intentionally lets the operator take responsibility for
        route, tide and return-margin choices. Telemetry loss, a controller
        fault and a stale dead-man command remain non-bypassable stops.
        """
        if not self._remote_enabled:
            return self._evaluate()
        health = self._health
        terrain = self._terrain
        if health is None or terrain is None or not self._telemetry_fresh():
            return Decision(HOLD, "Remote control blocked: vehicle telemetry is stale.")
        if health.fault.strip():
            return Decision(HOLD, f"Remote control blocked: {health.fault.strip()}.")
        if not self._command_fresh():
            return Decision(HOLD, "Remote control waiting for a fresh hold-to-run command.")
        return Decision(CRUISE, "Remote control active; autonomy return and hold decisions are advisory.")

    def _evaluate(self) -> Decision:
        self._refresh_return_estimate()
        health = self._health
        terrain = self._terrain
        if health is None or terrain is None:
            return evaluate(
                self._snapshot(
                    telemetry_fresh=False,
                    link_ok=False,
                    payload_secured=False,
                    fault="",
                    mobility_health_percent=0.0,
                    tide_risk=0.0,
                    seconds_until_unsafe=0.0,
                    corridor_traversable=False,
                ),
                self._policy_config(),
            )
        return evaluate(
            self._snapshot(
                telemetry_fresh=self._telemetry_fresh(),
                link_ok=health.link_ok,
                payload_secured=health.payload_secured,
                fault=health.fault.strip(),
                mobility_health_percent=health.mobility_health_percent,
                tide_risk=terrain.tide_risk,
                seconds_until_unsafe=terrain.seconds_until_corridor_unsafe,
                corridor_traversable=terrain.corridor_traversable,
            ),
            self._policy_config(),
        )

    def _snapshot(
        self,
        *,
        telemetry_fresh: bool,
        link_ok: bool,
        payload_secured: bool,
        fault: str,
        mobility_health_percent: float,
        tide_risk: float,
        seconds_until_unsafe: float,
        corridor_traversable: bool,
    ) -> PolicySnapshot:
        return PolicySnapshot(
            phase=self._phase,
            telemetry_fresh=telemetry_fresh,
            outbound_path_current=self._outbound_path_current(),
            outbound_path_timed_out=self._outbound_path_timed_out(),
            command_fresh=self._command_fresh(),
            return_path_ready=self._return_path_ready(),
            return_path_timed_out=self._return_path_timed_out(),
            link_ok=link_ok,
            payload_secured=payload_secured,
            fault=fault,
            mobility_health_percent=mobility_health_percent,
            tide_risk=tide_risk,
            seconds_until_corridor_unsafe=seconds_until_unsafe,
            corridor_traversable=corridor_traversable,
            estimate=self._estimate,
        )

    def _refresh_return_estimate(self) -> None:
        if self._health is None or self._costmap is None:
            self._estimate = ReturnEstimate(False, "Awaiting raw health and terrain cost map.")
            return
        if not path_ends_at_home(
            self._return_path_points,
            self._home(),
            self._parameter("home_tolerance_m"),
        ):
            self._estimate = ReturnEstimate(False, "Return route does not end within HOME tolerance.")
            return
        self._estimate = estimate_return(
            self._return_path_points,
            self._costmap,
            self._health.battery_percent,
            self._health.mobility_health_percent,
            self._estimator_config(),
        )

    def _outbound_path_current(self) -> bool:
        if self._planned_path_received_at is None:
            return False
        return (
            self._costmap_changed_at is None
            or self._planned_path_received_at >= self._costmap_changed_at
        )

    def _outbound_path_timed_out(self) -> bool:
        return (
            not self._outbound_path_current()
            and self._costmap_changed_at is not None
            and self._now_s() - self._costmap_changed_at
            > self._parameter("path_replan_timeout_s")
        )

    def _return_path_ready(self) -> bool:
        if self._return_path_received_at is None or not self._estimate.valid:
            return False
        if self._phase is not MissionPhase.RETURNING:
            return True
        return (
            self._return_requested_at is not None
            and self._return_path_received_at >= self._return_requested_at
            and (
                self._costmap_changed_at is None
                or self._return_path_received_at >= self._costmap_changed_at
            )
        )

    def _return_path_timed_out(self) -> bool:
        return (
            self._phase is MissionPhase.RETURNING
            and self._return_requested_at is not None
            and not self._return_path_ready()
            and self._now_s() - self._return_requested_at
            > self._parameter("return_path_timeout_s")
        )

    def _telemetry_fresh(self) -> bool:
        if self._last_health_at is None or self._last_terrain_at is None:
            return False
        timeout = self._parameter("telemetry_timeout_s")
        return (
            self._now_s() - self._last_health_at <= timeout
            and self._now_s() - self._last_terrain_at <= timeout
        )

    def _command_fresh(self) -> bool:
        timestamp = (
            self._last_remote_command_at
            if self._remote_enabled else self._last_autonomous_command_at
        )
        return (
            timestamp is not None
            and self._now_s() - timestamp
            <= self._parameter("command_timeout_s")
        )

    def _apply_decision(self, decision: Decision) -> None:
        if decision.return_required and self._phase is not MissionPhase.RETURNING:
            self._request_return(decision.reason)
            return
        self._state = decision.state
        self._reason = decision.reason
        self._publish_status(decision.return_required)

    def _request_return(self, reason: str) -> None:
        if self._phase is not MissionPhase.RETURNING:
            self._phase = MissionPhase.RETURNING
            self._return_requested_at = self._now_s()
        self._state = RETURN
        self._reason = reason
        self._publish_stop()
        self._publish_status(True)

    def _reset(self, reason: str) -> None:
        self._phase = MissionPhase.PRELAUNCH
        self._planned_path_received_at = None
        self._return_path_received_at = None
        self._return_requested_at = None
        self._last_autonomous_command_at = None
        self._last_remote_command_at = None
        self._remote_enabled = False
        self._return_path_points = []
        self._estimate = ReturnEstimate(False, "Awaiting a return route and terrain cost map.")
        self._state = HOLD
        self._reason = reason
        self._publish_stop()
        self._publish_status(False)

    def _limited_command(self, command: Twist, state: str, *, remote: bool = False) -> Twist:
        if remote:
            linear_limit = self._parameter("remote_linear_speed_limit_mps")
            angular_limit = self._parameter("remote_angular_speed_limit_radps")
            approved = Twist()
            approved.linear.x = max(-linear_limit, min(linear_limit, command.linear.x))
            approved.angular.z = max(-angular_limit, min(angular_limit, command.angular.z))
            return approved
        limit_name = {
            CRUISE: "cruise_linear_speed_limit_mps",
            CAUTION: "caution_linear_speed_limit_mps",
            RETURN: "return_linear_speed_limit_mps",
        }[state]
        limit = self._parameter(limit_name)
        approved = Twist()
        approved.linear.x = max(-limit, min(limit, command.linear.x))
        approved.angular.z = command.angular.z
        return approved

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_status(self, return_required: bool) -> None:
        status = SafetyStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = self._state
        status.reason = self._reason
        status.return_required = return_required
        status.estimated_return_energy_percent = self._estimate.estimated_energy_percent
        status.return_margin_percent = self._estimate.margin_percent
        status.estimated_return_time_s = self._estimate.eta_s
        self._status_pub.publish(status)

    def _policy_config(self) -> PolicyConfig:
        return PolicyConfig(
            caution_tide_risk=self._parameter("caution_tide_risk"),
            return_tide_risk=self._parameter("return_tide_risk"),
            hold_tide_risk=self._parameter("hold_tide_risk"),
            caution_margin_percent=self._parameter("caution_return_margin_percent"),
            return_margin_percent=self._parameter("return_margin_percent"),
            hold_margin_percent=self._parameter("hold_return_margin_percent"),
            caution_mobility_percent=self._parameter("caution_mobility_health_percent"),
            return_mobility_percent=self._parameter("return_mobility_health_percent"),
            hold_mobility_percent=self._parameter("hold_mobility_health_percent"),
            tide_return_time_buffer_s=self._parameter("tide_return_time_buffer_s"),
        )

    def _estimator_config(self) -> ReturnEstimatorConfig:
        return ReturnEstimatorConfig(
            sample_spacing_m=self._parameter("route_sample_spacing_m"),
            no_go_cost=int(self._parameter("no_go_cost")),
            track_mode_enabled=bool(self._parameter("track_mode_enabled")),
            track_cost_max=int(self._parameter("track_cost_max")),
            elevated_hover_cost_min=int(self._parameter("elevated_hover_cost_min")),
            track_energy_percent_per_m=self._parameter("track_energy_percent_per_m"),
            hover_energy_percent_per_m=self._parameter("hover_energy_percent_per_m"),
            track_nominal_speed_mps=self._parameter("track_nominal_speed_mps"),
            hover_nominal_speed_mps=self._parameter("hover_nominal_speed_mps"),
            elevated_hover_energy_multiplier=self._parameter("elevated_hover_energy_multiplier"),
            elevated_hover_speed_multiplier=self._parameter("elevated_hover_speed_multiplier"),
            mode_transition_time_s=self._parameter("mode_transition_time_s"),
            mode_transition_energy_percent=self._parameter("mode_transition_energy_percent"),
            terrain_cost_energy_weight=self._parameter("terrain_cost_energy_weight"),
            mobility_degradation_weight=self._parameter("mobility_degradation_weight"),
            minimum_buffer_percent=self._parameter("minimum_return_buffer_percent"),
            contingency_ratio=self._parameter("return_energy_contingency_ratio"),
        )

    def _home(self) -> tuple[float, float]:
        return self._parameter("home_x_m"), self._parameter("home_y_m")

    def _parameter(self, name: str) -> float:
        return self.get_parameter(name).value

    def _now_s(self) -> float:
        # All freshness and timeout decisions use the same clock as the
        # simulation. This avoids false stale-data holds when Gazebo runs
        # slower than wall time.
        return self.get_clock().now().nanoseconds * 1.0e-9


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # rclpy can report a take_message conversion error while launch is
        # tearing down subscriptions. Preserve real runtime failures.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
