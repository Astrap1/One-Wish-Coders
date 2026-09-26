"""Tests for the public mobility modes and transition safeguards."""

import importlib.util
import math
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


# --- Version 2: retractable tracks with cushion load sharing ---------------
def _tracks(policy: str = "terrain_auto", **kw) -> "ModeMachine":
    args = dict(
        policy=policy, gear="tracks", retract_position=0.25, fold_s=0.2,
        firm_dwell_s=0.2, settle_s=0.2, transition_timeout_s=2.0,
        track_share_slope=0.6, track_deploy_slope_deg=15.0,
        slope_dwell_s=0.2,
        settle_gap_m=0.03, settle_gap_tolerance_m=0.008,
    )
    args.update(kw)
    return ModeMachine(**args)


def test_tracks_hover_waits_for_ready_before_retracting() -> None:
    modes = _tracks(policy="hover_only")
    assert modes.mode == "TRANSITION"

    modes.step(0.1, hover_state="SPIN_UP", gear_pos=0.0)
    assert modes.hover_enabled and modes.lift_share is None
    assert modes.legs == 0.0          # tracks stay down until hover-ready

    modes.step(0.1, hover_state="HOVER", gear_pos=0.0)
    assert modes.legs == 0.25         # now retract
    assert modes.mode == "TRANSITION"

    modes.step(0.1, hover_state="HOVER", gear_pos=0.25)
    assert modes.mode == "HOVER"
    assert modes.drive_mode == "HOVER"


def _enter_v2_hover(modes: "ModeMachine") -> None:
    modes.step(0.1, terrain="FIRM", hover_state="HOVER", gear_pos=0.0)
    modes.step(0.1, terrain="FIRM", hover_state="HOVER", gear_pos=0.25)
    assert modes.mode == "HOVER"


def test_tracks_start_in_hover_with_tracks_retracted() -> None:
    modes = _tracks()
    assert modes.mode == "TRANSITION" and modes.transition_target == "HOVER"
    _enter_v2_hover(modes)
    assert modes.drive_mode == "HOVER"
    assert modes.hover_enabled and modes.legs == 0.25


def test_tracks_hover_to_track_deploys_then_settles_on_measured_gap() -> None:
    modes = _tracks()
    _enter_v2_hover(modes)
    for _ in range(3):                # sustained 16 degree firm-land climb
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="HOVER", gap=0.05, gear_pos=0.25)
    assert modes.mode == "TRANSITION" and modes.legs == 0.0
    assert modes.lift_share is None   # still fully hovering while deploying
    assert modes.drive_mode is None

    modes.step(0.1, terrain="FIRM", slope_deg=16.0,
               hover_state="HOVER", gap=0.05, gear_pos=0.0)
    assert modes.lift_share == 0.6    # tracks down: lower the lift
    for _ in range(3):                # still hovering: not settled
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="LOAD_SHARE", gap=0.05, gear_pos=0.0)
    assert modes.mode == "TRANSITION"
    for _ in range(3):                # gap at the on-track height for settle_s
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="LOAD_SHARE", gap=0.031, gear_pos=0.0)
    assert modes.mode == "TRACK"


def test_tracks_steep_slope_selects_track_with_slope_share() -> None:
    modes = _tracks(slope_dwell_s=0.2)
    _enter_v2_hover(modes)
    for _ in range(3):
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="HOVER", gap=0.03, gear_pos=0.25)
    assert modes.mode == "TRANSITION" and modes.transition_target == "TRACK"


def test_low_or_non_firm_slope_does_not_leave_hover() -> None:
    modes = _tracks(slope_dwell_s=0.2)
    _enter_v2_hover(modes)
    for _ in range(5):
        modes.step(0.1, terrain="FIRM", slope_deg=14.9,
                   hover_state="HOVER", gear_pos=0.25)
        modes.step(0.1, terrain="MUD", slope_deg=30.0,
                   hover_state="HOVER", gear_pos=0.25)
    assert modes.mode == "HOVER"


def test_tracks_settle_timeout_reports_fault() -> None:
    modes = _tracks(transition_timeout_s=0.6)
    _enter_v2_hover(modes)
    for _ in range(3):
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="HOVER", gap=0.05, gear_pos=0.25)
    for _ in range(8):                # deployed, but never settles on the tracks
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="LOAD_SHARE", gap=0.06, gear_pos=0.0)
    assert modes.fault == "track_settle_timeout"
    assert modes.drive_mode is None


def test_costmap_bands_select_mobility_terrain() -> None:
    terrain_from_costs = MODULE.terrain_from_costs
    assert terrain_from_costs([10, 12, 19]) == "FIRM"
    assert terrain_from_costs([10, 30]) == "MUD"          # hover terrain ahead
    assert terrain_from_costs([10, 95, -1]) == "FIRM"     # no-go / unknown ignored
    assert terrain_from_costs([-1, 100]) is None


def test_costmap_lookahead_samples_along_heading() -> None:
    sample = MODULE.sample_costmap
    # 10 x 1 strip, 1 m cells from x = 0: firm until x = 3, then water
    info = (1.0, 10, 1, 0.0, 0.0)
    data = [10, 10, 10, 30, 30, 30, 30, 30, 30, 30]
    assert sample(info, data, 0.5, 0.5, 0.0, 2.0) == [10, 10, 10, 10, 10]
    assert 30 in sample(info, data, 1.0, 0.5, 0.0, 2.5)   # water 2.5 m ahead
    assert sample(info, data, 1.0, 0.5, math.pi, 2.5) == [10, 10, 10]  # facing away


def test_tracks_retract_after_climb_or_over_water() -> None:
    modes = _tracks(slope_dwell_s=0.1)
    _enter_v2_hover(modes)
    for _ in range(2):
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="HOVER", gap=0.05, gear_pos=0.25)
    modes.step(0.1, terrain="FIRM", slope_deg=16.0,
               hover_state="HOVER", gap=0.05, gear_pos=0.0)
    for _ in range(3):
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="LOAD_SHARE", gap=0.03, gear_pos=0.0)
    assert modes.mode == "TRACK"
    modes.step(0.1, terrain="WATER", slope_deg=16.0, over_water=True,
               hover_state="HOVER", gear_pos=0.0)
    modes.step(0.1, terrain="WATER", slope_deg=16.0, over_water=True,
               hover_state="HOVER", gear_pos=0.25)
    assert modes.mode == "HOVER"


def test_tracks_settle_on_vertical_speed_on_uneven_ground() -> None:
    modes = _tracks()
    _enter_v2_hover(modes)
    for _ in range(3):
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="HOVER", gap=0.05, gear_pos=0.25)
    modes.step(0.1, terrain="FIRM", slope_deg=16.0,
               hover_state="HOVER", gap=0.05, gear_pos=0.0)
    for _ in range(12):               # mean gap reads 4.5 cm on uneven ground, but no vertical motion
        modes.step(0.1, terrain="FIRM", slope_deg=16.0,
                   hover_state="LOAD_SHARE", gap=0.045, gear_pos=0.0, vz=0.002)
    assert modes.mode == "TRACK"
