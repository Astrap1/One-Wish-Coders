"""Tests for the public mobility modes and transition safeguards."""

import importlib.util
from pathlib import Path

SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "vehicle_mobility_node.py"
)
SPEC = importlib.util.spec_from_file_location("vehicle_mobility_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
ModeMachine = MODULE.ModeMachine


def test_hover_requires_explicit_ready_state() -> None:
    modes = ModeMachine(
        policy="hover_only",
        spinup_s=0.1,
        fold_s=0.1,
        transition_timeout_s=1.0,
    )

    for _ in range(5):
        modes.step(0.1, hover_state="")

    assert modes.mode == "TRANSITION"
    assert modes.drive_mode is None

    modes.step(0.1, hover_state="HOVER")

    assert modes.mode == "HOVER"
    assert modes.drive_mode == "HOVER"


def test_missing_hover_ready_reports_fault_and_keeps_drive_stopped() -> None:
    modes = ModeMachine(
        policy="hover_only",
        spinup_s=0.1,
        fold_s=0.1,
        transition_timeout_s=0.4,
    )

    for _ in range(5):
        modes.step(0.1, hover_state="SPIN_UP")

    assert modes.mode == "TRANSITION"
    assert modes.fault == "hover_not_ready"
    assert modes.drive_mode is None


def test_terrain_auto_uses_three_public_modes() -> None:
    modes = ModeMachine(
        policy="terrain_auto",
        spinup_s=0.1,
        fold_s=0.1,
        firm_dwell_s=0.2,
        settle_s=0.1,
        transition_timeout_s=1.0,
    )
    assert modes.mode == "TRACK"

    modes.step(0.1, terrain="MUD")
    assert modes.mode == "TRANSITION"

    modes.step(0.1, terrain="MUD", hover_state="SPIN_UP")
    modes.step(0.1, terrain="MUD", hover_state="HOVER")
    assert modes.mode == "HOVER"

    modes.step(0.1, terrain="FIRM", hover_state="HOVER")
    modes.step(0.1, terrain="FIRM", hover_state="HOVER")
    assert modes.mode == "TRANSITION"

    modes.step(0.1, terrain="FIRM", hover_state="SPIN_DOWN")
    modes.step(0.1, terrain="FIRM", hover_state="OFF")
    assert modes.mode == "TRACK"
