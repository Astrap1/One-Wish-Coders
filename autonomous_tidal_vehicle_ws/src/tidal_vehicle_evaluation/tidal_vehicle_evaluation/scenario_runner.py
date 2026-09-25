"""Utilities for deterministic mission evaluation and scenario reporting."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ScenarioResult:
    """Aggregated results for one evaluation scenario."""

    name: str
    success: bool
    duration_s: float
    replans: int
    minimum_obstacle_clearance_m: float
    safety_interventions: int
    final_return_margin_percent: float


def summarize_scenario_result(result: ScenarioResult) -> str:
    """Render a concise summary suitable for operator and evaluator reporting."""
    outcome = "PASS" if result.success else "FAIL"
    return (
        f"Scenario {result.name}: {outcome}. "
        f"Duration {result.duration_s:.1f}s, replans {result.replans}, "
        f"minimum clearance {result.minimum_obstacle_clearance_m:.1f}m, "
        f"safety interventions {result.safety_interventions}, "
        f"final return margin {result.final_return_margin_percent:.1f}%"
    )
