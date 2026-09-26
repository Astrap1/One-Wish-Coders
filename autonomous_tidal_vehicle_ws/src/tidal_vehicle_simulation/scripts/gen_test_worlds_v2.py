#!/usr/bin/env python3
"""
gen_test_worlds_v2.py — writes the Version 2 (tracked, 2.5 m) vehicle test
worlds. Reuses the world helpers of gen_test_worlds.py (Version 1).

    python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_simulation/scripts/gen_test_worlds_v2.py
    python3 .../gen_test_worlds_v2.py --no-render-sensors --out /tmp/worlds   # headless physics tests

Output: tidal_vehicle_simulation/worlds/vehicle_tests/<name>.sdf

  v2_hover_test.sdf        spin up, retract the tracks, hold the cushion gap for
                           20 s, then 2.0 m/s and a 0.5 rad/s turn on cmd_vel_hover.
  v2_track_test.sdf        TRACK mode on firm ground: 1.0 m/s straight, pivot turn,
                           then up a 15 deg ramp onto a plateau.
  v2_load_share_test.sdf   TRACK mode with 60 % cushion load share: the vehicle
                           must stay on its tracks while the cushion carries 60 %.
  v2_transition_test.sdf   HOVER across water and mud, stop on the firm bank,
                           deploy the tracks, reduce lift to 40 % load share, and
                           climb a 12 deg slope on the tracks.
  v2_turn_test.sdf         HOVER turning A/B: a 1.0 rad/s turn at 1.5 m/s and a
                           0.8 rad/s pivot in place, first with fans only
                           (turn_aids off), then with rudders + puff ports.

Each scenario is scripted with hover::ScriptedCommands (timed gz-transport
messages), so each run is repeatable and needs no ROS in the loop.
"""
import argparse
import importlib.util
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("gen_test_worlds", HERE / "gen_test_worlds.py")
V1 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(V1)

OUT = HERE.parent / "worlds" / "vehicle_tests"
MODEL = "hovercraft_v2"
RETRACTED = 0.25               # track retract stroke (m), link_frames.json "tracks.stroke"


def include(name, x, y, yaw=0.0, z=0.002):
    return f"""
    <include>
      <uri>model://{MODEL}</uri>
      <name>{name}</name>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
    </include>"""


def ramp(name, x0, length, y0, y1, angle_deg, thick=0.4):
    """Static slope rising along +x from ground level at x0, plus a plateau
    at the top height. Returns (sdf, top_height, x_top)."""
    a = math.radians(angle_deg)
    h = length * math.sin(a)
    x1 = x0 + length * math.cos(a)
    # top-surface midpoint, then back off along the surface normal by thick/2
    mx, mz = (x0 + x1) / 2, h / 2
    cx, cz = mx + math.sin(a) * thick / 2, mz - math.cos(a) * thick / 2
    rgba, mu = V1.COLORS["firm"]
    sdf = f"""
    <model name="{name}"><static>true</static>
      <pose>{cx:.4f} {(y0 + y1) / 2} {cz:.4f} 0 {-a:.5f} 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{length} {y1 - y0} {thick}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><box><size>{length} {y1 - y0} {thick}</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>
      </link>
    </model>"""
    sdf += V1.block(f"{name}_plateau", x1, x1 + 12, y0, y1, h, "firm", thick=h + 0.3)
    return sdf, h, x1


def tracks(m, t, pos):
    return V1.cmd(t, f"/model/{m}/tracks_cmd", "gz.msgs.Double", f"data: {pos:.3f}")


def lift_share(m, t, share):
    return V1.cmd(t, f"/model/{m}/lift_share", "gz.msgs.Double", f"data: {share}")


def turn_aids(m, t, on):
    return V1.cmd(t, f"/model/{m}/turn_aids", "gz.msgs.Boolean", f"data: {'true' if on else 'false'}")


def repeat(fn, t0, t1, dt=0.2):
    """Re-send a command so the 0.5 s command timeout never stops it."""
    out, t = [], t0
    while t < t1 - 1e-9:
        out.append(fn(round(t, 2)))
        t += dt
    return out


def world(name, body, zones, commands, sensors, floaters=(MODEL,)):
    return V1.world(name, body, zones, commands, sensors, floaters=floaters)


def build(sensors):
    W = {}
    m = MODEL
    # ---------------- HOVER: gap hold, speed, turn ------------------------
    cmds = [V1.hover_on(m, 0.5), tracks(m, 3.0, RETRACTED)]
    cmds += repeat(lambda t: V1.hover_vel(m, t, 2.0), 26.0, 40.0)
    cmds += repeat(lambda t: V1.hover_vel(m, t, 1.0, 0.5), 40.0, 52.0)
    cmds += repeat(lambda t: V1.hover_vel(m, t, 0.0), 52.0, 62.0)
    W["v2_hover_test"] = world(
        "v2_hover_test",
        V1.block("ground", -20, 120, -60, 60, 0.0, "firm") + include(m, 0, 0), [], cmds, sensors)

    # ---------------- HOVER turning: fans only vs rudders + puff ports ----
    cmds = [turn_aids(m, 0.4, False), V1.hover_on(m, 0.5), tracks(m, 3.0, RETRACTED)]
    for t0, aids in ((8.0, False), (36.0, True)):         # turn at speed, A then B
        cmds += [turn_aids(m, t0, aids)]
        cmds += repeat(lambda t: V1.hover_vel(m, t, 1.5), t0, t0 + 10)
        cmds += repeat(lambda t: V1.hover_vel(m, t, 1.5, 1.0), t0 + 10, t0 + 20)
        cmds += repeat(lambda t: V1.hover_vel(m, t, 0.0), t0 + 20, t0 + 28)
    for t0, aids in ((64.0, True), (80.0, False)):        # pivot in place, B then A
        cmds += [turn_aids(m, t0, aids)]
        cmds += repeat(lambda t: V1.hover_vel(m, t, 0.0, 0.8), t0, t0 + 10)
        cmds += repeat(lambda t: V1.hover_vel(m, t, 0.0), t0 + 10, t0 + 16)
    W["v2_turn_test"] = world(
        "v2_turn_test",
        V1.block("ground", -40, 80, -60, 60, 0.0, "firm") + include(m, 0, 0), [], cmds, sensors)

    # ---------------- TRACK: straight, pivot, 15 deg ramp -----------------
    ramp_sdf, h15, _ = ramp("ramp15", 20.0, 8.0, -6, 6, 15.0)
    cmds = []
    cmds += repeat(lambda t: V1.drive(m, t, 1.0), 2.0, 8.0)          # straight 6 s
    cmds += repeat(lambda t: V1.drive(m, t, 0.0, 0.5), 8.0, 14.0)    # pivot turn
    cmds += repeat(lambda t: V1.drive(m, t, 0.0, -0.5), 14.0, 20.0)  # pivot back
    cmds += repeat(lambda t: V1.drive(m, t, 1.0), 20.0, 44.0)        # to and up the ramp
    cmds += repeat(lambda t: V1.drive(m, t, 0.0), 44.0, 48.0)
    W["v2_track_test"] = world(
        "v2_track_test",
        V1.block("ground", -10, 40, -12, 12, 0.0, "firm") + ramp_sdf + include(m, 0, 0),
        [], cmds, sensors)

    # ---------------- TRACK with 60 % cushion load share ------------------
    cmds = [lift_share(m, 0.5, 0.6), V1.hover_on(m, 0.6)]
    cmds += repeat(lambda t: V1.drive(m, t, 1.0), 6.0, 14.0)
    cmds += repeat(lambda t: V1.drive(m, t, 0.0), 14.0, 18.0)
    W["v2_load_share_test"] = world(
        "v2_load_share_test",
        V1.block("ground", -10, 40, -12, 12, 0.0, "firm") + include(m, 0, 0), [], cmds, sensors)

    # ---------------- HOVER water -> mud, then TRACK up the bank ----------
    ramp_sdf, h12, _ = ramp("bank_slope", 30.0, 8.0, -8, 8, 12.0)
    body = (V1.block("launch", -8, 6, -8, 8, 0.0, "firm") +
            V1.block("channel_bed", 6, 16, -8, 8, -0.45, "bed") +
            V1.water("channel_water", 6, 16, -8, 8, 0.0) +
            V1.block("mudflat", 16, 26, -8, 8, 0.0, "mud") +
            V1.block("bank", 26, 30, -8, 8, 0.0, "firm") + ramp_sdf +
            include(m, 0, 0))
    cmds = [V1.hover_on(m, 0.5), tracks(m, 3.0, RETRACTED)]
    cmds += repeat(lambda t: V1.hover_vel(m, t, 1.5), 7.0, 25.0)       # over water and mud
    cmds += repeat(lambda t: V1.hover_vel(m, t, 0.0), 25.0, 33.0)      # stop on the bank
    # HOVER -> TRACK: stopped; deploy tracks while hovering, then lower lift to 40 %
    cmds += [tracks(m, 33.0, 0.0), lift_share(m, 36.0, 0.4)]
    cmds += repeat(lambda t: V1.drive(m, t, 1.0), 40.0, 60.0)          # climb on tracks
    cmds += repeat(lambda t: V1.drive(m, t, 0.0), 60.0, 63.0)
    W["v2_transition_test"] = world(
        "v2_transition_test", body,
        [("channel", "WATER", 6, 16, -8, 8, 0.0), ("mudflat", "MUD", 16, 26, -8, 8)],
        cmds, sensors)
    return W


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render-sensors", action="store_true")
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for name, sdf in build(not a.no_render_sensors).items():
        (out / f"{name}.sdf").write_text(sdf.replace("gen_worlds.py", "gen_test_worlds_v2.py"))
        print("wrote", out / f"{name}.sdf")


if __name__ == "__main__":
    main()
