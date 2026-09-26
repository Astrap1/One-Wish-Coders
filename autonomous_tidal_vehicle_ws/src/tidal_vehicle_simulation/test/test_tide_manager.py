"""Regression tests for tide event timing and the corridor terrain profile."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).parents[1] / "scripts" / "tide_manager.py"
SPEC = importlib.util.spec_from_file_location("tide_manager", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
TideManager = MODULE.TideManager
STATIC_OBSTACLES = MODULE.STATIC_OBSTACLES


def _manager() -> tuple[TideManager, list[float]]:
    manager = object.__new__(TideManager)
    now = [0.0]
    manager._now = lambda: now[0]
    manager.duration = 20.0
    manager.initial_level = -2.8
    manager.rise = 2.8
    manager.started_at = 0.0
    manager.hold = False
    manager.held_fraction = 0.0
    manager.last_level = -2.8
    return manager, now


def test_hold_freezes_progress_and_resume_excludes_held_time() -> None:
    manager, now = _manager()
    now[0] = 4.0
    manager._event_callback(SimpleNamespace(data="tide_hold"))
    assert manager._fraction() == pytest.approx(0.2)

    now[0] = 14.0
    assert manager._fraction() == pytest.approx(0.2)
    manager._event_callback(SimpleNamespace(data="tide_resume"))
    assert manager._fraction() == pytest.approx(0.2)

    now[0] = 15.0
    assert manager._fraction() == pytest.approx(0.25)


def test_reset_stops_at_low_tide_until_rise_event() -> None:
    manager, now = _manager()
    now[0] = 8.0
    manager._event_callback(SimpleNamespace(data="reset"))
    now[0] = 18.0
    assert manager._fraction() == 0.0

    manager._event_callback(SimpleNamespace(data="tide_rise"))
    now[0] = 20.0
    assert manager._fraction() == pytest.approx(0.1)


def test_terrain_profile_matches_banks_slopes_and_floor() -> None:
    manager, _ = _manager()
    assert manager._terrain_height(0.0) == pytest.approx(0.0)
    assert manager._terrain_height(4.0) == pytest.approx(0.0)
    assert manager._terrain_height(7.7320508) == pytest.approx(-1.0)
    assert manager._terrain_height(52.0) == pytest.approx(-3.0)
    assert manager._terrain_height(96.2679492) == pytest.approx(-1.0)
    assert manager._terrain_height(100.0) == pytest.approx(0.0)


def test_static_obstacle_footprints_match_world_count() -> None:
    manager, _ = _manager()
    assert len(STATIC_OBSTACLES) == 30
    assert manager._is_static_obstacle(25.0, -12.0)
    assert manager._is_static_obstacle(106.0, -10.0)
    assert not manager._is_static_obstacle(52.0, 0.0)
