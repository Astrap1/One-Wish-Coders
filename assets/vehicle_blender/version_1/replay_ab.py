"""
replay_ab.py — render the Checkpoint 2 mud A/B clip from the Gazebo logs.

Gazebo writes each vehicle's hull pose and joint angles to a CSV
(AirCushion <log_file>). This script rebuilds the same Blender model
twice, drives both copies from those logs frame by frame and renders a
side-by-side lane view with a small overlay label. The motion is exactly
the simulated motion. Blender only does the drawing.

    blender --background --python assets/vehicle_blender/version_1/replay_ab.py -- \
        --ground results/mud_ab_test/hover_hc_ground.csv \
        --hover  results/mud_ab_test/hover_hc_hover.csv \
        --out results/mud_ab_frames --fps 12 --res 960x540 --samples 8
    ffmpeg -framerate 12 -i results/mud_ab_frames/%04d.png -r 24 -pix_fmt yuv420p mud_ab.mp4
"""
import argparse
import math
import runpy
import sys
from pathlib import Path

import bpy
import bmesh
from mathutils import Euler, Matrix, Vector

HERE = Path(__file__).resolve().parent
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
ap = argparse.ArgumentParser()
ap.add_argument("--ground", required=True)
ap.add_argument("--hover", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--fps", type=float, default=12)
ap.add_argument("--res", default="960x540")
ap.add_argument("--samples", type=int, default=8)
ap.add_argument("--t0", type=float, default=0.0)
ap.add_argument("--t1", type=float, default=1e9)
ap.add_argument("--frames", default="", help="optional a:b frame range to render")
args = ap.parse_args(argv)

sys.argv = [sys.argv[0], "--", "--no-render"]
g = runpy.run_path(str(HERE / "build_vehicle.py"), run_name="replay")
eu = g["eu"]
HULL_Z0 = g["HULL_DZ"] + 0.25          # hull link origin height in ground mode


def load(path):
    with open(path) as f:
        head = f.readline().strip().split(",")
        rows = [l.strip().split(",") for l in f if l.strip()]
    out = []
    for r in rows:
        d = {}
        for k, v in zip(head, r):
            try:
                d[k] = float(v)
            except ValueError:
                d[k] = v
        out.append(d)
    return out


def sample(log, t):
    """Linear interpolation of numeric fields at time t."""
    lo, hi = 0, len(log) - 1
    if t <= log[0]["t"]:
        return log[0]
    if t >= log[-1]["t"]:
        return log[-1]
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if log[mid]["t"] <= t:
            lo = mid
        else:
            hi = mid
    a, b = log[lo], log[hi]
    w = (t - a["t"]) / max(b["t"] - a["t"], 1e-9)
    out = {}
    for k, va in a.items():
        vb = b[k]
        if isinstance(va, float) and isinstance(vb, float) and not (math.isnan(va) or math.isnan(vb)):
            if k in ("roll", "pitch", "yaw"):
                d = (vb - va + math.pi) % (2 * math.pi) - math.pi
                out[k] = va + w * d
            else:
                out[k] = va + w * (vb - va)
        else:
            out[k] = va if w < 0.5 else vb
    return out


# ---------------------------------------------------------------- scene -----
eu.clear_scene()
links, tags, root = g["build_all"]()
coll = bpy.data.collections["hovercraft"]


def clone(links, root, suffix):
    new = {}
    for name, ob in links.items():
        c = ob.copy()
        c.name = name + suffix
        coll.objects.link(c)
        new[name] = c
    r = root.copy(); r.name = root.name + suffix; coll.objects.link(r)
    for name, ob in links.items():
        if ob.parent is root:
            new[name].parent = r
        elif ob.parent is not None:
            new[name].parent = new[ob.parent.name]
    return new, r


veh = {"ground": (links, root)}
veh["hover"] = clone(links, root, "_B")
rest = {k: {n: (o.location.copy(), o.rotation_euler.copy()) for n, o in v[0].items()}
        for k, v in veh.items()}

# Ground: same blocks as mud_ab_test.sdf
mats = {}


def mat(name, rgb, rough=0.8, spec=0.3):
    m = bpy.data.materials.new(name)
    bsdf = eu.ensure_nodes(m).nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Roughness"].default_value = rough
    return m


def box(name, x0, x1, y0, y1, z0, z1, m):
    me = bpy.data.meshes.new(name)
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1)
    bmesh.ops.scale(bm, vec=(x1 - x0, y1 - y0, z1 - z0), verts=bm.verts)
    bmesh.ops.translate(bm, vec=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), verts=bm.verts)
    bm.to_mesh(me); bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    me.materials.append(m)
    return ob


firm = mat("firm", (0.16, 0.17, 0.09))
mud = mat("mud", (0.055, 0.03, 0.012), rough=0.25)
white = mat("line", (0.8, 0.8, 0.8))
green = mat("finish", (0.05, 0.5, 0.1))
yellow = mat("lane", (0.8, 0.7, 0.05))
box("firm_start", -6, 4, -5, 5, -0.3, 0.0, firm)
box("mud_patch", 4, 12, -5, 5, -0.3, 0.0, mud)
box("firm_finish", 12, 26, -5, 5, -0.3, 0.0, firm)
box("start_line", 0.76, 0.84, -4, 4, 0, 0.003, white)
box("finish_line", 15.96, 16.04, -4, 4, 0, 0.003, green)
box("lane_divider", -4, 26, -0.04, 0.04, 0, 0.003, yellow)

sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
sun.data.energy = 3.5
sun.rotation_euler = (math.radians(35), 0, math.radians(-40))
bpy.context.scene.collection.objects.link(sun)
world = bpy.data.worlds.new("w")
wnt = eu.ensure_nodes(world)
bg = wnt.nodes.get("Background") or wnt.nodes.new("ShaderNodeBackground")
bg.inputs["Color"].default_value = (0.62, 0.68, 0.75, 1)
bpy.context.scene.world = world

cam = eu.add_camera("cam", (4.0, -9.5, 4.2), (6.0, 0.0, 0.0), lens=28)
bpy.context.scene.camera = cam
eu.setup_render("CYCLES", res=tuple(int(v) for v in args.res.split("x")), samples=args.samples)

# ---------------------------------------------------------------- drive -----
logs = {"ground": load(args.ground), "hover": load(args.hover)}
t_end = min(args.t1, min(l[-1]["t"] for l in logs.values()))
t_start = max(args.t0, 0.0)
n = int((t_end - t_start) * args.fps) + 1
Path(args.out).mkdir(parents=True, exist_ok=True)
lo_f, hi_f = 0, n
if args.frames:
    a, b = args.frames.split(":")
    lo_f, hi_f = int(a), min(int(b), n)

fan_angle = {"ground": 0.0, "hover": 0.0}
thr_angle = {"ground": 0.0, "hover": 0.0}
for f in range(n):
    t = t_start + f / args.fps
    for key, (lk, rt) in veh.items():
        s = sample(logs[key], t)
        R = Euler((s["roll"], s["pitch"], s["yaw"]), "XYZ").to_matrix().to_4x4()
        hull_w = Matrix.Translation((s["x"], s["y"], s["z"])) @ R
        rt.matrix_world = hull_w @ Matrix.Translation((0, 0, -HULL_Z0))
        for c in ("fl", "fr", "rl", "rr"):
            sx = 1 if c[0] == "f" else -1
            leg = lk["wheel_leg_" + c]
            loc0, rot0 = rest[key]["wheel_leg_" + c]
            leg.rotation_euler = (0, sx * s.get(f"wheel_leg_{c}_joint", 0.0), 0)
            sh = lk["wheel_shock_" + c]
            sloc0, _ = rest[key]["wheel_shock_" + c]
            sh.location = sloc0 + Vector((0, 0, s.get(f"wheel_shock_{c}_joint", 0.0)))
            lk["wheel_" + c].rotation_euler = (0, s.get(f"wheel_{c}_joint", 0.0), 0)
        state = s.get("hover_state", "OFF")
        if state not in ("OFF",):
            fan_angle[key] += 60.0 / args.fps
        thr = 0.5 * (s.get("thrust_left", 0.0) + s.get("thrust_right", 0.0))
        thr_angle[key] += thr * 2.0 / args.fps
        lk["lift_fan"].rotation_euler = (0, 0, fan_angle[key])
        lk["thrust_fan_left"].rotation_euler = (thr_angle[key], 0, 0)
        lk["thrust_fan_right"].rotation_euler = (thr_angle[key], 0, 0)
    if lo_f <= f < hi_f:
        # camera tracks the midpoint of the two vehicles (smoothly)
        xs = [sample(logs[k], t)["x"] for k in logs]
        cx = max(3.0, min(14.0, 0.5 * (xs[0] + xs[1])))
        cam.location = (cx - 2.0, -9.5, 4.2)
        cam.rotation_euler = (Vector((cx, 0.0, 0.0)) - cam.location).to_track_quat("-Z", "Y").to_euler()
        bpy.context.view_layer.update()
        eu.render_to(cam, Path(args.out) / f"{f:04d}.png")
print("frames:", n)
import os
os._exit(0)
