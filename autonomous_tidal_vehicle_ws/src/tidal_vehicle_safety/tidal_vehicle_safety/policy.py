"""Pure state-precedence policy for the safety supervisor."""

from __future__ import annotations

from dataclasses import dataclass

from .mission_phase import MissionPhase
from .return_estimator import ReturnEstimate


CRUISE = "CRUISE"
CAUTION = "CAUTION"
HOLD = "HOLD"
RETURN = "RETURN"


@dataclass(frozen=True)
class PolicyConfig:
    caution_tide_risk: float
    return_tide_risk: float
    hold_tide_risk: float
    caution_margin_percent: float
    return_margin_percent: float
    hold_margin_percent: float
    caution_mobility_percent: float
    return_mobility_percent: float
    hold_mobility_percent: float
    tide_return_time_buffer_s: float


@dataclass(frozen=True)
class PolicySnapshot:
    phase: MissionPhase
    telemetry_fresh: bool
    outbound_path_current: bool
    outbound_path_timed_out: bool
    command_fresh: bool
    return_path_ready: bool
    return_path_timed_out: bool
    link_ok: bool
    payload_secured: bool
    fault: str
    mobility_health_percent: float
    tide_risk: float
    seconds_until_corridor_unsafe: float
    corridor_traversable: bool
    estimate: ReturnEstimate


@dataclass(frozen=True)
class Decision:
    state: str
    reason: str
    return_required: bool = False


def evaluate(snapshot: PolicySnapshot, config: PolicyConfig) -> Decision:
    """Apply ordered safety conditions; earlier conditions take precedence."""
    if snapshot.phase is MissionPhase.PRELAUNCH:
        return Decision(HOLD, "Awaiting a mission goal and current telemetry.")
    if not snapshot.telemetry_fresh:
        return Decision(HOLD, "Vehicle health or terrain telemetry is stale.")
    if snapshot.fault:
        return Decision(
            HOLD,
            f"Vehicle controller reported {snapshot.fault}; holding position.",
            snapshot.phase is MissionPhase.RETURNING,
        )

    if snapshot.phase is MissionPhase.RETURNING:
        if not snapshot.corridor_traversable or snapshot.tide_risk >= config.hold_tide_risk:
            return Decision(HOLD, "Return corridor is no longer traversable.", True)
        if snapshot.mobility_health_percent < config.hold_mobility_percent:
            return Decision(HOLD, "Mobility is too degraded to continue returning.", True)
        if not snapshot.return_path_ready:
            reason = (
                "Fresh return route was not received before the timeout."
                if snapshot.return_path_timed_out
                else "Holding until a fresh return route to HOME is validated."
            )
            return Decision(HOLD, reason, True)
        if not snapshot.estimate.valid:
            return Decision(HOLD, snapshot.estimate.reason, True)
        if snapshot.estimate.margin_percent <= config.hold_margin_percent:
            return Decision(HOLD, "Return-energy margin is exhausted.", True)
        return Decision(RETURN, "Following the validated route to HOME.", True)

    if not snapshot.corridor_traversable or snapshot.tide_risk >= config.hold_tide_risk:
        return Decision(HOLD, "No traversable corridor is currently available.")
    if snapshot.mobility_health_percent < config.hold_mobility_percent:
        return Decision(HOLD, "Critical mobility degradation; holding position.")
    if not snapshot.estimate.valid:
        return Decision(HOLD, snapshot.estimate.reason)
    if snapshot.estimate.margin_percent <= config.hold_margin_percent:
        return Decision(HOLD, "Return-energy margin is exhausted.")

    tide_window_closing = (
        snapshot.seconds_until_corridor_unsafe > 0.0
        and snapshot.estimate.eta_s + config.tide_return_time_buffer_s
        >= snapshot.seconds_until_corridor_unsafe
    )
    if (
        not snapshot.link_ok
        or snapshot.estimate.margin_percent <= config.return_margin_percent
        or snapshot.mobility_health_percent < config.return_mobility_percent
        or snapshot.tide_risk >= config.return_tide_risk
        or tide_window_closing
    ):
        return Decision(RETURN, "Return safety margin requires an immediate return.", True)

    if not snapshot.outbound_path_current:
        reason = (
            "Autonomy did not publish a terrain-updated route before the timeout."
            if snapshot.outbound_path_timed_out
            else "Awaiting a route updated for the latest terrain map."
        )
        return Decision(HOLD, reason)
    if not snapshot.command_fresh:
        return Decision(HOLD, "Autonomy proposed command is stale.")
    if snapshot.phase is MissionPhase.OUTBOUND and not snapshot.payload_secured:
        return Decision(HOLD, "Payload is not secured.")
    if (
        snapshot.tide_risk >= config.caution_tide_risk
        or snapshot.estimate.margin_percent <= config.caution_margin_percent
        or snapshot.mobility_health_percent < config.caution_mobility_percent
    ):
        return Decision(CAUTION, "Elevated tide, return-margin or mobility risk; slowing for reassessment.")
    return Decision(CRUISE, "Normal operating margin.")
