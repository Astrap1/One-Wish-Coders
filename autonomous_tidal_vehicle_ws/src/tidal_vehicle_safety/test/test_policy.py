from tidal_vehicle_safety.mission_phase import MissionPhase
from tidal_vehicle_safety.policy import CAUTION, HOLD, RETURN, PolicyConfig, PolicySnapshot, evaluate
from tidal_vehicle_safety.return_estimator import ReturnEstimate


def _config() -> PolicyConfig:
    return PolicyConfig(0.60, 0.75, 0.85, 20.0, 12.0, 0.0, 75.0, 60.0, 35.0, 30.0)


def _snapshot(**changes) -> PolicySnapshot:
    values = {
        "phase": MissionPhase.OUTBOUND,
        "telemetry_fresh": True,
        "outbound_path_current": True,
        "outbound_path_timed_out": False,
        "command_fresh": True,
        "return_path_ready": True,
        "return_path_timed_out": False,
        "link_ok": True,
        "payload_secured": True,
        "fault": "",
        "mobility_health_percent": 100.0,
        "tide_risk": 0.2,
        "seconds_until_corridor_unsafe": 300.0,
        "corridor_traversable": True,
        "estimate": ReturnEstimate(True, "valid", 10.0, 20.0, 40.0, 20.0),
    }
    values.update(changes)
    return PolicySnapshot(**values)


def test_caution_and_return_tide_thresholds():
    assert evaluate(_snapshot(tide_risk=0.60), _config()).state == CAUTION
    decision = evaluate(_snapshot(tide_risk=0.75), _config())
    assert decision.state == RETURN
    assert decision.return_required


def test_time_to_unsafe_forces_return_before_hard_tide_limit():
    decision = evaluate(_snapshot(seconds_until_corridor_unsafe=50.0), _config())

    assert decision.state == RETURN
    assert decision.return_required


def test_invalid_return_path_and_critical_fault_hold_position():
    assert evaluate(_snapshot(estimate=ReturnEstimate(False, "blocked")), _config()).state == HOLD
    assert evaluate(_snapshot(mobility_health_percent=30.0), _config()).state == HOLD


def test_vehicle_controller_fault_holds_position():
    decision = evaluate(_snapshot(fault="hover_not_ready"), _config())

    assert decision.state == HOLD
    assert "hover_not_ready" in decision.reason


def test_returning_holds_until_a_fresh_valid_path_is_available():
    decision = evaluate(
        _snapshot(phase=MissionPhase.RETURNING, return_path_ready=False),
        _config(),
    )

    assert decision.state == HOLD
    assert decision.return_required
