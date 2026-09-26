import types

from tidal_vehicle_operator import web_dashboard, web_dashboard_node
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


def test_build_dashboard_payload_exposes_status_class_for_ui_theming() -> None:
    payload = build_dashboard_payload(
        mission_state="HOLD",
        return_required=False,
        planned_route_points=0,
        return_route_points=0,
        battery_percent=0.0,
        return_margin_percent=0.0,
        reason="Awaiting telemetry.",
    )

    assert payload["status_class"] == "hold"


def test_browser_state_replaces_non_finite_telemetry_with_json_null() -> None:
    web_dashboard.update_dashboard_state(
        estimated_return_energy_percent=float("nan"),
        telemetry={"eta": float("inf")},
    )

    encoded = __import__("json").dumps(web_dashboard.DashboardHandler._state, allow_nan=False)
    assert '"estimated_return_energy_percent": null' in encoded
    assert '"eta": null' in encoded


def test_dashboard_declares_dom_helper_before_remote_handlers() -> None:
    page = web_dashboard.DASHBOARD_PAGE

    assert page.index("const E=id=>document.getElementById(id)") < page.index(
        "E('remote-toggle').onclick"
    )


def test_dashboard_dispatch_goal_is_the_far_riverbank() -> None:
    assert "{kind:'goal',x:104.0,y:0.0}" in web_dashboard.DASHBOARD_PAGE


def test_dashboard_has_one_unified_terrain_and_route_panel() -> None:
    page = web_dashboard.DASHBOARD_PAGE

    assert "Terrain map &amp; mission route" in page
    assert "Terrain cost map" not in page
    assert page.count('id="map"') == 1
    assert 'id="cost-map"' not in page
    assert "BARRIER A" in page and "BARRIER B" in page
