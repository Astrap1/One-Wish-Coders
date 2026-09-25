import types

from tidal_vehicle_operator import web_dashboard_node
from tidal_vehicle_operator.operator_dashboard import build_dashboard_payload, summarize_dashboard


def test_browser_dashboard_main_runs_spin_while_serving_dashboard(monkeypatch) -> None:
    calls: list[str] = []

    class FakeNode:
        def __init__(self, *_args, **_kwargs) -> None:
            pass

        def destroy_node(self) -> None:
            calls.append("destroy")

    class FakeThread:
        def __init__(self, target, daemon: bool = False) -> None:
            self.target = target
            self.daemon = daemon

        def start(self) -> None:
            calls.append("server-thread-start")
            self.target()
            calls.append("server-thread-finish")

    fake_rclpy = types.SimpleNamespace(
        init=lambda args=None: calls.append("init"),
        shutdown=lambda: calls.append("shutdown"),
        ok=lambda: True,
        spin=lambda node: calls.append("spin"),
    )

    monkeypatch.setattr(web_dashboard_node, "rclpy", fake_rclpy)
    monkeypatch.setattr(web_dashboard_node, "BrowserDashboardNode", FakeNode)
    monkeypatch.setattr(web_dashboard_node, "serve_dashboard", lambda: calls.append("serve"))
    monkeypatch.setattr(web_dashboard_node.threading, "Thread", FakeThread)

    web_dashboard_node.main([])

    assert "server-thread-start" in calls
    assert "serve" in calls
    assert "spin" in calls


def test_summarize_dashboard_reports_live_mission_state() -> None:
    summary = summarize_dashboard(
        mission_state="CRUISE",
        return_required=False,
        planned_route_points=12,
        return_route_points=7,
        battery_percent=81.0,
        return_margin_percent=23.0,
        reason="Stable corridor; continuing outbound leg.",
    )

    assert "CRUISE" in summary
    assert "81.0%" in summary
    assert "23.0%" in summary
    assert "outbound" in summary.lower()
    assert "return" in summary.lower()


def test_build_dashboard_payload_contains_state_and_summary() -> None:
    payload = build_dashboard_payload(
        mission_state="CRUISE",
        return_required=False,
        planned_route_points=8,
        return_route_points=3,
        battery_percent=72.5,
        return_margin_percent=18.0,
        reason="Proceeding along the approved corridor.",
    )

    assert payload["mission_state"] == "CRUISE"
    assert payload["battery_percent"] == 72.5
    assert "CRUISE" in payload["summary"]
    assert payload["return_required"] is False
