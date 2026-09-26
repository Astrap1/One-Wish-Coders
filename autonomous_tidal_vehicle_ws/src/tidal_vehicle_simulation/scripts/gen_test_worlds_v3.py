#!/usr/bin/env python3
"""
gen_test_worlds_v3.py — writes the Version 3 (3.0 m, 530 kg, high-speed)
vehicle test worlds. Reuses the Version 2 scenarios (with the Version 3
model) plus a high-speed open-water test.

    python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_simulation/scripts/gen_test_worlds_v3.py
    python3 .../gen_test_worlds_v3.py --no-render-sensors --out /tmp/v3worlds   # headless physics tests

Output: tidal_vehicle_simulation/worlds/vehicle_tests/<name>.sdf

  v3_hover_test, v3_track_test, v3_load_share_test, v3_transition_test,
  v3_turn_test             the Version 2 scenarios, run with hovercraft_v3
  v3_collision_test.sdf    hovers bow-first into a wall at 1.5 m/s, then backs off: the
                           collision monitor must report a front hit on the wall
                           (tools/vehicle_tests/run_collision_test.sh).
  v3_speed_test.sdf        1.5 km of open water: hump crossing, 30 km/h cruise,
                           50 km/h, braking from 50 km/h, turns at 10 and 30 km/h,
                           a pivot, and a 58 km/h command to show the margin.

Each scenario is scripted with hover::ScriptedCommands (timed gz-transport
messages), so each run is repeatable and needs no ROS in the loop.
"""
import argparse
import importlib.util
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _load(name):
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


V1 = _load("gen_test_worlds")
V2 = _load("gen_test_worlds_v2")
MODEL = "hovercraft_v3"
V2.MODEL = MODEL                     # the Version 2 scenarios, with the Version 3 vehicle
V2.world.__defaults__ = ((MODEL,),)  # its floaters default was bound to hovercraft_v2
OUT = HERE.parent / "worlds" / "vehicle_tests"
RETRACTED = 0.25
KMH = 1 / 3.6
CRUISE, TOP, SLOW = 30 * KMH, 50 * KMH, 10 * KMH


def build(sensors):
    W = {}
    # Version 2 scenarios, renamed v3_* (the checks in analyze.py are shared)
    for name, sdf in V2.build(sensors).items():
        W[name.replace("v2_", "v3_", 1)] = sdf.replace(f'<world name="{name}">',
                                                       f'<world name="{name.replace("v2_", "v3_", 1)}">')

    # ---------------- high speed over open water ---------------------------
    m = MODEL
    rep, hv = V2.repeat, V1.hover_vel
    cmds = [V1.hover_on(m, 0.5), V2.tracks(m, 3.0, RETRACTED)]
    cmds += rep(lambda t: hv(m, t, CRUISE), 6.0, 36.0)             # over the hump, to cruise
    cmds += rep(lambda t: hv(m, t, TOP), 36.0, 60.0)               # 50 km/h
    cmds += rep(lambda t: hv(m, t, 0.0), 60.0, 75.0)               # brake from 50 km/h
    cmds += rep(lambda t: hv(m, t, SLOW), 75.0, 90.0)              # 10 km/h straight
    cmds += rep(lambda t: hv(m, t, SLOW, 0.30), 90.0, 110.0)       # 10 km/h turn
    cmds += rep(lambda t: hv(m, t, CRUISE), 110.0, 130.0)          # 30 km/h straight
    cmds += rep(lambda t: hv(m, t, CRUISE, 0.12), 130.0, 150.0)    # 30 km/h turn (~70 m radius)
    cmds += rep(lambda t: hv(m, t, 0.0), 150.0, 165.0)
    cmds += rep(lambda t: hv(m, t, 0.0, 0.6), 165.0, 177.0)        # pivot in place
    cmds += rep(lambda t: hv(m, t, 16.0), 177.0, 207.0)            # 58 km/h: margin above 50
    cmds += rep(lambda t: hv(m, t, 0.0), 207.0, 222.0)
    body = (V1.block("launch", -12, 6, -30, 30, 0.0, "firm") +
            V1.block("bed", 6, 1500, -400, 400, -0.60, "bed") +
            V1.water("open_water", 6, 1500, -400, 400, 0.0) +
            V2.include(m, 0, 0))
    W["v3_speed_test"] = V2.world("v3_speed_test", body,
                                  [("open_water", "WATER", 6, 1500, -400, 400, 0.0)], cmds, sensors)

    # ---------------- collision: bow into a wall ---------------------------
    cmds = [V1.hover_on(m, 0.5), V2.tracks(m, 3.0, RETRACTED)]
    cmds += rep(lambda t: hv(m, t, 1.5), 7.0, 16.0)                # into the wall at x = 10
    cmds += rep(lambda t: hv(m, t, -0.8), 16.0, 20.0)              # back off
    cmds += rep(lambda t: hv(m, t, 0.0), 20.0, 24.0)
    body = (V1.block("pad", -12, 30, -20, 20, 0.0, "firm") +
            V1.block("wall", 10.0, 10.6, -4, 4, 1.2, "firm", thick=1.2) +
            V2.include(m, 0, 0))
    W["v3_collision_test"] = V2.world("v3_collision_test", body, [], cmds, sensors)
    return W


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render-sensors", action="store_true")
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for name, sdf in build(not a.no_render_sensors).items():
        (out / f"{name}.sdf").write_text(sdf.replace("gen_worlds.py", "gen_test_worlds_v3.py"))
        print("wrote", out / f"{name}.sdf")


if __name__ == "__main__":
    main()
