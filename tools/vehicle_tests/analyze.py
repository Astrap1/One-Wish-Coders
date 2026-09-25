#!/usr/bin/env python3
"""
analyze.py — pass/fail checks and plots from the AirCushion CSV logs.

    python3 tools/vehicle_tests/analyze.py            # analyse everything under results/

Writes results/acceptance.md (a table of checks) and results/plots/*.png.
"""
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "results"
PLOTS = RES / "plots"
TARGET_GAP = 0.04

# Colours: ground mode vs hover mode, used consistently in every plot
C_GROUND, C_HOVER, C_MUD, C_WATER = "#b5651d", "#1f6fb2", "#6b4a2b", "#4aa3c7"


def load(path):
    with open(path) as f:
        header = f.readline().strip().split(",")
        rows = [line.strip().split(",") for line in f if line.strip()]
    cols = {}
    for i, h in enumerate(header):
        vals = [r[i] if i < len(r) else "nan" for r in rows]
        try:
            cols[h] = np.array([float(v) for v in vals])
        except ValueError:
            cols[h] = np.array(vals)
    return cols


def at(d, t, key):
    i = int(np.argmin(np.abs(d["t"] - t)))
    return float(d[key][i])


def style(ax, title, xl, yl):
    ax.set_title(title, loc="left", fontsize=11, fontweight="bold")
    ax.set_xlabel(xl); ax.set_ylabel(yl)
    ax.grid(alpha=0.25)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


checks = []          # (test, check, value, target, pass)


def check(test, name, value, target, ok):
    checks.append((test, name, value, target, bool(ok)))


def empty_test():
    p = RES / "empty_test" / "hover_hovercraft.csv"
    if not p.exists():
        return
    d = load(p)
    z_wheels = at(d, 2.8, "z")
    z_folded = at(d, 6.8, "z")
    z_back = at(d, 9.8, "z")
    leg = at(d, 6.8, "wheel_leg_fl_joint")
    tilt = np.degrees(np.max(np.abs(np.r_[d["roll"], d["pitch"]])))
    check("empty_test", "Sits on wheels: hull origin height", f"{z_wheels:.3f} m", "0.300 m (±0.02)",
          abs(z_wheels - 0.30) < 0.02)
    check("empty_test", "Legs fold on command (leg angle)", f"{math.degrees(leg):.1f}°", "90° (±5)",
          abs(math.degrees(leg) - 90) < 5)
    # skirt collision stops 5 cm above the visual skirt bottom (flexible fingers)
    check("empty_test", "Hull settles onto skirt when legs fold", f"{z_folded:.3f} m", "~0.150 m (±0.02)",
          abs(z_folded - 0.15) < 0.02)
    check("empty_test", "Legs unfold and lift hull back up", f"{z_back:.3f} m", "0.300 m (±0.02)",
          abs(z_back - 0.30) < 0.02)
    check("empty_test", "Stable: max roll/pitch whole run", f"{tilt:.1f}°", "< 10°", tilt < 10)
    dist = at(d, 14.0, "x") - at(d, 10.0, "x")
    check("empty_test", "Ground-mode drive 4 s at 0.6 m/s", f"{dist:.2f} m", "≈2.4 m (±0.4)",
          abs(dist - 2.4) < 0.4)

    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.plot(d["t"], d["z"], color="#333")
    for t, lab in ((3, "fold legs"), (7, "unfold"), (10, "drive")):
        ax.axvline(t, color="#999", ls="--", lw=0.8)
        ax.text(t + 0.1, ax.get_ylim()[1] if False else 0.31, lab, fontsize=8, color="#555")
    style(ax, "Phase 2 — hull height through fold / unfold", "sim time (s)", "hull origin z (m)")
    fig.tight_layout(); fig.savefig(PLOTS / "phase2_fold_unfold.png", dpi=150); plt.close(fig)


def gap_hold():
    p = RES / "gap_hold_test" / "hover_hovercraft.csv"
    if not p.exists():
        return
    d = load(p)
    m = (d["t"] >= 10) & (d["t"] <= 70)
    g = d["gap_mean"][m]
    corners = np.c_[d["gap_fl"][m], d["gap_fr"][m], d["gap_rl"][m], d["gap_rr"][m]]
    dev = np.max(np.abs(corners - TARGET_GAP))
    span = d["t"][m][-1] - d["t"][m][0] if m.any() else 0
    check("gap_hold_test", "Hover gap held (all 4 corners, 60 s)", f"max dev {dev * 100:.2f} cm",
          "±1.5 cm", dev <= 0.015 and span >= 59)
    check("gap_hold_test", "Mean gap", f"{np.mean(g) * 100:.2f} cm", "3–5 cm",
          0.03 <= np.mean(g) <= 0.05)
    osc = np.std(g) * 100
    check("gap_hold_test", "No oscillation (std of mean gap)", f"{osc:.3f} cm", "< 0.3 cm", osc < 0.3)

    fig, ax = plt.subplots(figsize=(8, 3.2))
    ax.axhspan((TARGET_GAP - 0.015) * 100, (TARGET_GAP + 0.015) * 100, color=C_HOVER, alpha=0.08,
               label="±1.5 cm band")
    for k, lab in (("gap_fl", "FL"), ("gap_fr", "FR"), ("gap_rl", "RL"), ("gap_rr", "RR")):
        ax.plot(d["t"], d[k] * 100, lw=1, label=lab)
    ax.set_ylim(0, 12)
    ax.legend(ncol=5, fontsize=8, frameon=False, loc="upper right")
    style(ax, "Hover gap under each skirt corner (target 4 cm)", "sim time (s)", "gap (cm)")
    fig.tight_layout(); fig.savefig(PLOTS / "phase4_gap_hold.png", dpi=150); plt.close(fig)


def hover_drive():
    p = RES / "hover_drive_test" / "hover_hovercraft.csv"
    if not p.exists():
        return
    d = load(p)
    m = (d["t"] > 5) & (d["t"] < 20)
    vmax = float(np.max(d["speed"][m]))
    check("hover_drive_test", "Top speed in hover (2 × 30 N)", f"{vmax:.2f} m/s", "2–3 m/s",
          2.0 <= vmax <= 3.0)
    # Turn radius: fit a circle to the steady part of the turn
    m2 = (d["t"] > 28) & (d["t"] < 40)
    x, y = d["x"][m2], d["y"][m2]
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    b = x ** 2 + y ** 2
    cx, cy, c0 = np.linalg.lstsq(A, b, rcond=None)[0]
    r = math.sqrt(c0 + cx ** 2 + cy ** 2)
    check("hover_drive_test", "Turn radius (30 N / 3 N differential)", f"{r:.2f} m", "≤ 3 m", r <= 3.0)
    tilt = np.degrees(np.max(np.abs(np.r_[d["roll"], d["pitch"]])))
    check("hover_drive_test", "Stable in hard turn: max roll/pitch", f"{tilt:.1f}°", "< 15°", tilt < 15)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.6))
    a1.plot(d["t"], d["speed"], color=C_HOVER)
    style(a1, "Speed (full thrust from 5 s, turn from 20 s)", "sim time (s)", "speed (m/s)")
    a2.plot(d["x"], d["y"], color=C_HOVER)
    a2.add_patch(plt.Circle((cx, cy), r, fill=False, ls="--", color="#999"))
    a2.set_aspect("equal")
    style(a2, f"Path — turn radius {r:.1f} m", "x (m)", "y (m)")
    fig.tight_layout(); fig.savefig(PLOTS / "phase4_speed_turn.png", dpi=150); plt.close(fig)


def transition():
    p = RES / "transition_test" / "hover_hovercraft.csv"
    if not p.exists():
        return
    d = load(p)
    tilt = np.degrees(np.max(np.abs(np.r_[d["roll"], d["pitch"]])))
    xmax = float(np.max(d["x"]))
    over = d["t"] > 5
    gmin = float(np.min(d["gap_mean"][over]))
    check("transition_test", "Crosses water → mud → firm (max x)", f"{xmax:.1f} m",
          "> 23 m (whole hull on the bank)", xmax > 23)
    check("transition_test", "No flip: max roll/pitch", f"{tilt:.1f}°", "< 15°", tilt < 15)
    check("transition_test", "No sinking: min mean gap", f"{gmin * 100:.1f} cm", "> 1 cm", gmin > 0.01)

    fig, ax = plt.subplots(figsize=(8, 3.2))
    for x0, x1, c, lab in ((4, 12, C_WATER, "water"), (12, 22, C_MUD, "mud")):
        ax.axvspan(x0, x1, color=c, alpha=0.15)
        ax.text((x0 + x1) / 2, 11, lab, ha="center", fontsize=9, color=c)
    ax.plot(d["x"], d["gap_mean"] * 100, color=C_HOVER)
    ax.set_ylim(0, 12)
    style(ax, "Hover gap along the course (water → mud → firm)", "x (m)", "mean gap (cm)")
    fig.tight_layout(); fig.savefig(PLOTS / "phase4_transition_gap.png", dpi=150); plt.close(fig)


def debris():
    pl = RES / "debris_test" / "hover_hc_low.csv"
    pt = RES / "debris_test" / "hover_hc_tall.csv"
    if not (pl.exists() and pt.exists()):
        return
    lo, ta = load(pl), load(pt)
    xl, xt = float(np.max(lo["x"])), float(np.max(ta["x"]))
    tilt = np.degrees(np.max(np.abs(np.r_[lo["roll"], lo["pitch"]])))
    check("debris_test", "Hover rides over a 7 cm branch (flexible fingers)", f"max x {xl:.2f} m",
          "> 6 m (past the branch at 5 m)", xl > 6.0)
    check("debris_test", "  ... without tipping", f"{tilt:.1f}°", "< 10°", tilt < 10)
    check("debris_test", "A 20 cm log blocks the skirt (must be avoided)", f"max x {xt:.2f} m",
          "< 5 m", xt < 5.0)


def cmd_vel():
    p = RES / "cmd_vel_test" / "hover_hovercraft.csv"
    if not p.exists():
        return
    d = load(p)
    fwd = d["vx"] * np.cos(d["yaw"]) + d["vy"] * np.sin(d["yaw"])
    m1 = (d["t"] > 8) & (d["t"] < 12)
    e1 = float(np.mean(np.abs(fwd[m1] - 1.5)))
    check("cmd_vel_test", "HOVER speed tracking at 1.5 m/s (mean error)", f"{e1:.3f} m/s", "< 0.15 m/s", e1 < 0.15)
    m2 = (d["t"] > 16) & (d["t"] < 20)
    yawrate = np.gradient(np.unwrap(d["yaw"]), d["t"])
    e2 = float(np.mean(np.abs(yawrate[m2] - 0.5)))
    check("cmd_vel_test", "HOVER yaw-rate tracking at 0.5 rad/s", f"{e2:.3f} rad/s", "< 0.1 rad/s", e2 < 0.1)
    stop = d["t"] > 25
    vs = float(np.max(d["speed"][stop])) if stop.any() else 9
    check("cmd_vel_test", "Stops within 5 s of a zero command", f"{vs:.3f} m/s", "< 0.1 m/s", vs < 0.1)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.4))
    a1.plot(d["t"], fwd, color=C_HOVER, label="measured")
    ref = np.select([d["t"] < 4, d["t"] < 12, d["t"] < 20], [0, 1.5, 1.0], 0)
    a1.plot(d["t"], ref, color="#999", ls="--", label="/cmd_vel")
    a1.legend(frameon=False, fontsize=8)
    style(a1, "Forward speed", "sim time (s)", "m/s")
    a2.plot(d["x"], d["y"], color=C_HOVER); a2.set_aspect("equal")
    style(a2, "Path", "x (m)", "y (m)")
    fig.tight_layout(); fig.savefig(PLOTS / "cmd_vel_tracking.png", dpi=150); plt.close(fig)


def mud_ab():
    pg = RES / "mud_ab_test" / "hover_hc_ground.csv"
    ph = RES / "mud_ab_test" / "hover_hc_hover.csv"
    if not (pg.exists() and ph.exists()):
        return
    g, h = load(pg), load(ph)
    xg, xh = float(g["x"][-1]), float(h["x"][-1])
    check("mud_ab_test", "GROUND mode: stopped inside the mud", f"final x {xg:.2f} m",
          "4 < x < 12 (mud)", 4 < xg < 12)
    late = g["t"] > g["t"][-1] - 3
    vg = float(np.mean(g["speed"][late]))
    check("mud_ab_test", "GROUND mode: bogged down (speed, last 3 s)", f"{vg:.3f} m/s", "< 0.05 m/s", vg < 0.05)
    check("mud_ab_test", "HOVER mode: crossed the mud", f"final x {xh:.2f} m", "> 16 m (finish)", xh > 16)
    dy = float(np.max(np.abs(h["y"] - h["y"][0])))
    check("mud_ab_test", "HOVER mode: stayed in its lane (max sideways drift)", f"{dy:.2f} m", "< 0.8 m", dy < 0.8)
    fig, ax = plt.subplots(figsize=(8, 3.4))
    ax.axhspan(4, 12, color=C_MUD, alpha=0.15)
    ax.text(0.3, 8, "mud patch", color=C_MUD, fontsize=9, va="center")
    ax.axhline(16, color="#2a9d3a", ls="--", lw=1)
    ax.text(0.3, 16.3, "finish", color="#2a9d3a", fontsize=9)
    ax.plot(g["t"], g["x"], color=C_GROUND, lw=2, label="GROUND mode (wheels)")
    ax.plot(h["t"], h["x"], color=C_HOVER, lw=2, label="HOVER mode (cushion + fans)")
    ax.legend(frameon=False, fontsize=9, loc="upper left", bbox_to_anchor=(0.0, 0.92))
    style(ax, "Mud A/B test — same patch, same vehicle, two modes", "sim time (s)", "distance along lane x (m)")
    fig.tight_layout(); fig.savefig(PLOTS / "checkpoint2_mud_ab.png", dpi=150); plt.close(fig)


# --------------------------------------------------------------------------
# Version 2 (hovercraft_v2: retractable tracks + cushion load sharing)
# --------------------------------------------------------------------------
def _v2(test):
    p = RES / test / "hover_hovercraft_v2.csv"
    return load(p) if p.exists() else None


def _win(d, t0, t1):
    return (d["t"] >= t0) & (d["t"] <= t1)


def _yaw_rate(d, t0, t1):
    m = _win(d, t0, t1)
    yaw = np.unwrap(d["yaw"][m])
    return (yaw[-1] - yaw[0]) / (d["t"][m][-1] - d["t"][m][0])


def _tilt(d):
    return math.degrees(max(np.max(np.abs(d["roll"])), np.max(np.abs(d["pitch"]))))


def v2_hover():
    d = _v2("v2_hover_test")
    if d is None:
        return
    t = "v2_hover_test"
    g = d["gap_mean"][_win(d, 6, 25)]
    check(t, "Tracks retracted into the hull before propulsion", f"{at(d, 6, 'track_left_joint'):.3f} m",
          "0.25 m (±0.01)", abs(at(d, 6, "track_left_joint") - 0.25) < 0.01)
    check(t, "Hover gap held (mean, 6-25 s)", f"{np.mean(g) * 100:.2f} cm", "5 cm (±1)",
          abs(np.mean(g) - 0.05) < 0.01)
    check(t, "Gap range", f"{g.min() * 100:.2f}..{g.max() * 100:.2f} cm", "within 3.5..6.5 cm",
          g.min() > 0.035 and g.max() < 0.065)
    v = np.mean(d["speed"][_win(d, 33, 39)])
    check(t, "HOVER speed tracking at 2.0 m/s", f"{v:.3f} m/s", "2.0 m/s (±0.15)", abs(v - 2.0) < 0.15)
    r = _yaw_rate(d, 44, 51)
    check(t, "HOVER yaw-rate tracking at 0.5 rad/s", f"{r:.3f} rad/s", "0.5 rad/s (±0.1)", abs(r - 0.5) < 0.1)
    vs = at(d, 61.9, "speed")
    check(t, "Stops after a zero command", f"{vs:.3f} m/s", "< 0.1 m/s", vs < 0.1)
    check(t, "Stable: max roll/pitch", f"{_tilt(d):.1f}°", "< 5°", _tilt(d) < 5)


def v2_track():
    d = _v2("v2_track_test")
    if d is None:
        return
    t = "v2_track_test"
    v = (at(d, 8, "x") - at(d, 3, "x")) / 5.0
    check(t, "TRACK speed at 1.0 m/s (3-8 s)", f"{v:.3f} m/s", "1.0 m/s (±0.1)", abs(v - 1.0) < 0.1)
    r = _yaw_rate(d, 9, 14)
    check(t, "Pivot turn at 0.5 rad/s", f"{r:.3f} rad/s", "0.5 rad/s (±0.1)", abs(r - 0.5) < 0.1)
    drift = math.hypot(at(d, 14, "x") - at(d, 9, "x"), at(d, 14, "y") - at(d, 9, "y"))
    check(t, "Pivot turn stays in place", f"{drift:.3f} m", "< 0.2 m", drift < 0.2)
    top = 8.0 * math.sin(math.radians(15))
    z = at(d, 47.9, "z") - 0.50
    check(t, "Climbs the 15° ramp onto the plateau", f"hull rise {z:.2f} m", f"≈ {top:.2f} m (±0.2)",
          abs(z - top) < 0.2)


def v2_load_share():
    d = _v2("v2_load_share_test")
    if d is None:
        return
    t = "v2_load_share_test"
    m = _win(d, 4, 14)
    sup, g = np.mean(d["support"][m]), d["gap_mean"][m]
    check(t, "Cushion carries the commanded 60 % share", f"{sup:.3f}", "0.60 (±0.05)", abs(sup - 0.6) < 0.05)
    check(t, "Vehicle stays on its tracks (skirt gap)", f"{g.min() * 100:.2f}..{g.max() * 100:.2f} cm",
          "3 cm (±0.8)", g.min() > 0.022 and g.max() < 0.038)
    v = (at(d, 14, "x") - at(d, 8, "x")) / 6.0
    check(t, "Track drive with load share at 1.0 m/s", f"{v:.3f} m/s", "1.0 m/s (±0.1)", abs(v - 1.0) < 0.1)


def v2_transition():
    d = _v2("v2_transition_test")
    if d is None:
        return
    t = "v2_transition_test"
    xs = at(d, 32, "x")                      # stopped on the bank (bank starts at x = 26)
    check(t, "HOVER across water and mud, stops on the bank", f"x {xs:.1f} m", "> 26 m", xs > 26)
    g = d["gap_mean"][_win(d, 8, 25)]
    check(t, "No sinking over water/mud (min gap)", f"{g.min() * 100:.1f} cm", "> 3 cm", g.min() > 0.03)
    check(t, "Tracks deployed, then 40 % load share", f"support {at(d, 39, 'support'):.2f}, "
          f"gap {at(d, 39, 'gap_mean') * 100:.1f} cm", "0.40 (±0.05), 3 cm (±0.8)",
          abs(at(d, 39, "support") - 0.4) < 0.05 and abs(at(d, 39, "gap_mean") - 0.03) < 0.008)
    top = 8.0 * math.sin(math.radians(12))
    z = at(d, 62.9, "z") - 0.50
    check(t, "Climbs the 12° bank on the tracks", f"hull rise {z:.2f} m", f"≈ {top:.2f} m (±0.2)",
          abs(z - top) < 0.2)
    check(t, "Stable: max roll", f"{math.degrees(np.max(np.abs(d['roll']))):.1f}°", "< 5°",
          math.degrees(np.max(np.abs(d["roll"]))) < 5)


def main():
    PLOTS.mkdir(parents=True, exist_ok=True)
    for f in (empty_test, gap_hold, hover_drive, transition, debris, cmd_vel, mud_ab,
              v2_hover, v2_track, v2_load_share, v2_transition):
        f()
    lines = ["| Test | Check | Result | Target | |", "|---|---|---|---|---|"]
    for t, n, v, tg, ok in checks:
        lines.append(f"| {t} | {n} | {v} | {tg} | {'PASS' if ok else 'FAIL'} |")
    (RES / "acceptance.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    json.dump([dict(test=t, check=n, value=v, target=tg, ok=ok) for t, n, v, tg, ok in checks],
              open(RES / "acceptance.json", "w"), indent=1)


if __name__ == "__main__":
    main()
