from tidal_vehicle_evaluation.scenario_runner import ScenarioResult, summarize_scenario_result


def test_summarize_scenario_result_reports_mission_metrics() -> None:
    result = ScenarioResult(
        name="normal_delivery",
        success=True,
        duration_s=324.5,
        replans=2,
        minimum_obstacle_clearance_m=1.8,
        safety_interventions=1,
        final_return_margin_percent=18.0,
    )

    summary = summarize_scenario_result(result)

    assert "normal_delivery" in summary
    assert "324.5s" in summary
    assert "2" in summary
    assert "18.0%" in summary
    assert "PASS" in summary
