"""
build_heavy_load.py — presentation variant of Vehicle Version 3: the same
hovercraft carrying a heavy, strapped deck load, with military camouflage and
markings. VISUAL ONLY: it builds the Version 3 model with build_vehicle.py,
adds the load and markings as extra objects parented to the hull, and renders.
It never writes the simulation meshes, link_frames.json, SDF or URDF, so the
Gazebo model (100 kg rated payload, olive paint) is unchanged.

Run headless from the repo root (Blender 4.2+, or the pip `bpy` 4.2 module):

    blender --background --python assets/vehicle_blender/version_3/build_heavy_load.py
    blender --background --python assets/vehicle_blender/version_3/build_heavy_load.py -- --no-render
    ... -- --no-camo            keep the plain olive paint, markings and load only

Outputs:
    assets/vehicle_blender/version_3/hovercraft_v3_heavy_load.blend
    assets/vehicle_blender/version_3/renders/heavy_load/*.png
    assets/vehicle_blender/version_3/renders/heavy_load/load_log.txt

The markings are generic (team hull number "OWC-03", tactical chevrons, a
military load class disc, propeller warnings); no real national insignia or
unit badge. Every load item stays below the LiDAR's lowest beam and clear of
the lift-fan intake, the engine-cover louvres and the fuel filler.
"""
import math
import sys
from pathlib import Path

import bpy
import bmesh
from mathutils import Matrix, Vector

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
import build_vehicle as bv          # noqa: E402  (also imports export_utils as bv.eu)

eu = bv.eu
T, R, bm_box, bm_cyl, bm_tube = bv.T, bv.R, bv.bm_box, bv.bm_cyl, bv.bm_tube
DECK = 0.728                        # top of the anti-slip deck panels
DECAL_LIFT = 0.0015                 # decals float this far off the surface
TAU = 2 * math.pi

EXTRA_MATS = {
    "stencil":      ((0.720, 0.720, 0.690), 0.70, 0.0),
    "hazard":       ((0.850, 0.560, 0.010), 0.50, 0.0),
    "black_paint":  ((0.010, 0.010, 0.010), 0.60, 0.0),
    "cerise":       ((0.950, 0.060, 0.220), 0.55, 0.0),   # VS-17 air-recognition panel
    "canvas":       ((0.080, 0.078, 0.040), 0.90, 0.0),
    "strap":        ((0.260, 0.200, 0.100), 0.80, 0.0),
    "net":          ((0.025, 0.030, 0.016), 0.85, 0.0),
    "wood":         ((0.220, 0.130, 0.060), 0.80, 0.0),
    "can_green":    ((0.050, 0.070, 0.028), 0.55, 0.1),
}

# Load items: (description, estimated kg). The rated payload is the 100 kg box.
LOAD = [
    ("sealed payload box (rated payload)", 100),
    ("stacked equipment case", 45),
    ("3 x 20 L jerry cans, full", 72),
    ("4 x ration / water crates", 100),
    ("2 x rolled kit bags", 30),
]


def make_extra_materials():
    for name, (rgb, rough, metal) in EXTRA_MATS.items():
        m = bpy.data.materials.new(name)
        bsdf = eu.ensure_nodes(m).nodes.get("Principled BSDF")
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Roughness"].default_value = rough
        bsdf.inputs["Metallic"].default_value = metal
        m.diffuse_color = (*rgb, 1.0)
        bv.MATS[name] = m


def camouflage(mat_name="olive"):
    """Three-colour woodland pattern on the hull paint (object-space noise,
    hard colour steps), replacing the plain olive."""
    m = bv.MATS[mat_name]
    nt = m.node_tree
    bsdf = nt.nodes.get("Principled BSDF")
    tc = nt.nodes.new("ShaderNodeTexCoord")
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 1.6
    noise.inputs["Detail"].default_value = 1.5
    noise.inputs["Distortion"].default_value = 0.8
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.interpolation = 'CONSTANT'
    els = ramp.color_ramp.elements
    els[0].position, els[0].color = 0.0, (0.030, 0.045, 0.018, 1)      # dark green
    els[1].position, els[1].color = 0.40, (0.115, 0.135, 0.040, 1)     # olive
    e = els.new(0.56)
    e.color = (0.085, 0.060, 0.030, 1)                                 # brown
    e = els.new(0.68)
    e.color = (0.018, 0.018, 0.014, 1)                                 # black
    nt.links.new(tc.outputs["Object"], noise.inputs["Vector"])
    nt.links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])


# --------------------------------------------------------------------------
# Surface helpers
# --------------------------------------------------------------------------
def surface(origin, direction):
    """(point, normal) where a ray from outside hits the vehicle."""
    dg = bpy.context.evaluated_depsgraph_get()
    hit, loc, nrm, *_ = bpy.context.scene.ray_cast(dg, Vector(origin), Vector(direction))
    if not hit:
        raise RuntimeError(f"no surface along {origin} -> {direction}")
    return loc, nrm.normalized()


def decal_frame(point, normal):
    """Matrix whose local XY plane lies on the surface (text reads upright
    from outside: x = up x normal) and whose +Z is the surface normal."""
    n = normal.normalized()
    up = Vector((0, 0, 1))
    x = up.cross(n)
    if x.length < 1e-6:                      # horizontal surface
        x = Vector((1, 0, 0))
    x.normalize()
    y = n.cross(x)
    M = Matrix((x, y, n)).transposed().to_4x4()
    M.translation = point + n * DECAL_LIFT
    return M


def text_bm(body, size):
    """Flat, filled text as a bmesh in the XY plane, centred on the origin."""
    cu = bpy.data.curves.new(f"txt_{body}", 'FONT')
    cu.body = body
    cu.size = size
    cu.align_x, cu.align_y = 'CENTER', 'CENTER'
    cu.fill_mode = 'BOTH'
    ob = bpy.data.objects.new(f"txt_{body}", cu)
    bpy.context.scene.collection.objects.link(ob)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    pb = bmesh.new()
    pb.from_mesh(me)
    bpy.data.objects.remove(ob)
    bpy.data.curves.remove(cu)
    bpy.data.meshes.remove(me)
    return pb


def poly_bm(pts):
    """One flat polygon (XY) from a list of (x, y) points."""
    pb = bmesh.new()
    pb.faces.new([pb.verts.new((x, y, 0.0)) for x, y in pts])
    return pb


def chevron_bm(w, h, t):
    """Inverted-V tactical chevron, width w, height h, stroke t."""
    return poly_bm([(-w / 2, h / 2), (0, -h / 2 + t * 1.4), (w / 2, h / 2),
                    (w / 2 - t, h / 2), (0, -h / 2 + t * 1.4 + t * 1.3), (-w / 2 + t, h / 2)])


def arc_band_bm(r, x0, x1, a0, a1, n=4):
    """Part of a cylindrical band about the X axis (radius r, x0..x1, angles a0..a1)."""
    pb = bmesh.new()
    rows = []
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        rows.append((pb.verts.new((x0, r * math.cos(a), r * math.sin(a))),
                     pb.verts.new((x1, r * math.cos(a), r * math.sin(a)))))
    for (a, b), (c, d) in zip(rows, rows[1:]):
        pb.faces.new((a, b, d, c))
    return pb


# --------------------------------------------------------------------------
# Deck load
# --------------------------------------------------------------------------
def strap(L, pts, w=0.035):
    """Flat ratchet strap through the given points (a chain of thin boxes)."""
    for a, b in zip(pts, pts[1:]):
        a, b = Vector(a), Vector(b)
        d = b - a
        pb = bm_box(d.length, w, 0.006)
        rot = d.to_track_quat('X', 'Z').to_matrix().to_4x4()
        L.add(pb, "strap", T(*((a + b) / 2)) @ rot)


def jerry_can(L, x, y):
    w, t, h = 0.345, 0.165, 0.47
    z = DECK + h / 2
    L.add(bm_box(w, t, h, bevel=0.018), "can_green", T(x, y, z))
    for sy in (1, -1):                      # the pressed "X" on both faces
        for ang in (32, -32):
            L.add(bm_box(0.36, 0.006, 0.03, bevel=0.002), "can_green",
                  T(x, y + sy * t / 2, z - 0.02) @ R('Y', ang))
    for dx in (-0.09, 0.0, 0.09):           # three carry handles
        L.add(bm_tube([(x + dx - 0.03, y, DECK + h), (x + dx - 0.02, y, DECK + h + 0.035),
                       (x + dx + 0.02, y, DECK + h + 0.035), (x + dx + 0.03, y, DECK + h)],
                      0.007, seg=6), "can_green")
    L.add(bm_cyl(0.025, 0.03, seg=12), "gunmetal", T(x + w / 2 - 0.05, y, DECK + h + 0.015))


def crate(L, x, y, z0, label):
    w, d, h = 0.36, 0.26, 0.24
    L.add(bm_box(w, d, h, bevel=0.01), "can_green", T(x, y, z0 + h / 2))
    L.add(bm_box(w + 0.006, d + 0.006, 0.02, bevel=0.004), "gunmetal", T(x, y, z0 + h - 0.02))
    for sx in (1, -1):                      # end handles and side latches
        L.add(bm_box(0.012, 0.10, 0.025, bevel=0.003), "gunmetal", T(x + sx * (w / 2 + 0.006), y, z0 + h * 0.6))
    L.add(bm_box(0.05, 0.012, 0.05, bevel=0.003), "gunmetal", T(x, y - d / 2 - 0.004, z0 + h - 0.04))
    # stencil on the outboard face (normal -Y)
    L.add(text_bm(label, 0.06), "stencil",
          T(x, y - d / 2 - 0.0035, z0 + h * 0.42) @ R('X', 90))
    return z0 + h


def kit_bag(L, x, y, length=0.50, r=0.12):
    z = DECK + r
    L.add(bm_cyl(r, length, seg=20, bevel=0.03), "canvas", T(x, y, z) @ R('Y', 90))
    for dx in (-0.15, 0.15):
        L.add(bm_cyl(r + 0.004, 0.03, seg=20), "strap", T(x + dx, y, z) @ R('Y', 90))


def cargo_net(L, x0, x1, y_in, y_out, z_top, z_bot, pitch=0.09):
    """Net over the top and down the outboard face of a stack."""
    r = 0.004
    ys = [y_in + (y_out - y_in) * k / 6 for k in range(7)]
    nx = max(2, int(abs(x1 - x0) / pitch))
    for k in range(nx + 1):                 # cords running inboard -> over -> down
        x = x0 + (x1 - x0) * k / nx
        L.add(bm_tube([(x, y_in, z_top + 0.012), (x, y_out, z_top + 0.012),
                       (x, y_out + math.copysign(0.012, y_out), z_bot)], r, seg=5), "net")
    for y in ys:                            # cords along the top
        L.add(bm_tube([(x0, y, z_top + 0.014), (x1, y, z_top + 0.014)], r, seg=5), "net")
    zs = [z_bot + (z_top - z_bot) * k / 5 for k in range(5)]
    for z in zs:                            # cords along the outboard face
        L.add(bm_tube([(x0, y_out + math.copysign(0.013, y_out), z),
                       (x1, y_out + math.copysign(0.013, y_out), z)], r, seg=5), "net")


def build_load():
    L = bv.Link("heavy_load", (0, 0, 0))
    top = {}

    # 1) equipment case stacked on the sealed payload box, on two battens,
    #    with an air-recognition panel on top
    px = bv.PAY_X
    lid = 1.09
    for dx in (-0.27, 0.27):
        L.add(bm_box(0.06, 0.56, 0.06, bevel=0.005), "wood", T(px + dx, 0, lid + 0.03))
    case_z0, case_h = lid + 0.06, 0.26
    L.add(bm_box(0.70, 0.50, case_h, bevel=0.025), "can_green", T(px, 0, case_z0 + case_h / 2))
    L.add(bm_box(0.705, 0.505, 0.012), "black_paint", T(px, 0, case_z0 + case_h * 0.72))
    for sy in (1, -1):
        for dx in (-0.2, 0.2):
            L.add(bm_box(0.05, 0.02, 0.05, bevel=0.004), "gunmetal",
                  T(px + dx, sy * 0.255, case_z0 + case_h * 0.72))
    case_top = case_z0 + case_h
    L.add(bm_box(0.60, 0.40, 0.004), "cerise", T(px, 0, case_top + 0.002))
    for dx in (-0.20, 0.20):                # two straps round box + case, to the deck
        strap(L, [(px + dx, -0.40, 0.73), (px + dx, -0.30, case_top + 0.006),
                  (px + dx, 0.30, case_top + 0.006), (px + dx, 0.40, 0.73)])
    top["stacked case"] = (px, case_top + 0.006)

    # 2) three jerry cans along the port deck edge, strapped down
    can_y = 0.62
    for x in (-0.40, -0.02, 0.36):
        jerry_can(L, x, can_y)
    strap(L, [(-0.62, can_y, 0.73), (-0.58, can_y, DECK + 0.30),
              (0.58, can_y, DECK + 0.30), (0.62, can_y, 0.73)])
    for x in (-0.40, -0.02, 0.36):
        strap(L, [(x, 0.50, 0.73), (x, can_y - 0.085, DECK + 0.49),
                  (x, can_y + 0.085, DECK + 0.49), (x, 0.74, 0.73)], w=0.03)
    top["jerry cans"] = (0.0, DECK + 0.47 + 0.035)

    # 3) four ration / water crates, two high, starboard deck, under a net
    cy = -0.60
    z1 = crate(L, 0.04, cy, DECK, "RATIONS")
    crate(L, 0.42, cy, DECK, "WATER")
    z2 = crate(L, 0.04, cy, z1, "RATIONS")
    crate(L, 0.42, cy, z1, "WATER")
    cargo_net(L, -0.16, 0.62, -0.46, -0.745, z2, DECK + 0.04)
    top["crates"] = (0.23, z2 + 0.02)

    # 4) rolled kit bags on the bow deck, both sides
    for sy in (1, -1):
        kit_bag(L, 0.85, sy * 0.60)
    top["kit bags"] = (0.85, DECK + 0.24)
    return L, top


# --------------------------------------------------------------------------
# Markings
# --------------------------------------------------------------------------
def build_markings():
    L = bv.Link("army_markings", (0, 0, 0))

    def on(origin, direction, pb, mat, lift=0.0):
        p, n = surface(origin, direction)
        L.add(pb, mat, T(*(n * lift)) @ decal_frame(p, n))

    for sy in (1, -1):
        # hull number and tactical chevron on the flared topsides
        on((-0.33, sy * 3, 0.655), (0, -sy, 0), text_bm("OWC-03", 0.085), "stencil")
        on((0.33, sy * 3, 0.655), (0, -sy, 0), chevron_bm(0.16, 0.08, 0.02), "stencil")
        # propeller warning on the outboard side of each thrust duct
        on((-1.17, sy * 3, bv.THRUST_Z), (0, -sy, 0), text_bm("DANGER  PROPELLER", 0.032), "hazard")
    # bow: hull number and a yellow military load class disc (MLC 1: < 1 t)
    on((3, -0.40, 0.50), (-1, 0, 0), text_bm("OWC-03", 0.05), "stencil")
    on((3, 0.40, 0.50), (-1, 0, 0), bm_cyl(0.058, 0.002, seg=32), "hazard")
    on((3, 0.40, 0.50), (-1, 0, 0), text_bm("1", 0.075), "black_paint", lift=0.003)
    # stern transom
    on((-3, 0.0, 0.53), (1, 0, 0), text_bm("OWC-03", 0.045), "stencil")
    # yellow / black warning band round the intake lip of each thrust duct
    r = bv.DUCT_R_OUT + 0.003
    x1 = bv.DUCT_X1
    n = 24
    for sy in (1, -1):
        for k in range(n):
            pb = arc_band_bm(r, x1 - 0.05, x1 - 0.005, TAU * k / n, TAU * (k + 1) / n)
            L.add(pb, "hazard" if k % 2 == 0 else "black_paint",
                  T(0, sy * bv.THRUST_Y, bv.THRUST_Z))
    # "NO STEP" on both duct tops (horizontal: reads from the stern)
    for sy in (1, -1):
        p, nrm = surface((-1.14, sy * bv.THRUST_Y, 3), (0, 0, -1))
        M = Matrix.Translation(p + nrm * DECAL_LIFT) @ R('Z', 90)
        L.add(text_bm("NO STEP", 0.05), "stencil", M)
    return L


def lidar_clearance(top):
    """The LiDAR's lowest beams point 15 deg down from the mast: every item
    must stay below that cone."""
    lines = []
    ok = True
    for name, (x, z) in top.items():
        beam = bv.LIDAR_Z - abs(x - bv.LIDAR_X) * math.tan(math.radians(15))
        clear = z < beam
        ok &= clear
        lines.append(f"  {name:14s} top {z:.2f} m at x {x:+.2f}; lowest LiDAR beam there "
                     f"{beam:.2f} m -> {'clear' if clear else 'BLOCKS LIDAR'}")
    return lines, ok


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    do_render = "--no-render" not in argv
    eu.clear_scene()
    links, _tags, root = bv.build_all()
    bv.animate_mode_switch(links, root)
    make_extra_materials()
    if "--no-camo" not in argv:
        camouflage("olive")

    coll = bpy.data.collections.new("heavy_load_variant")
    bpy.context.scene.collection.children.link(coll)
    bpy.context.scene.frame_set(1)
    markings = build_markings().finish(coll)      # ray casts need the bare vehicle
    load, top = build_load()
    load = load.finish(coll)
    hull = links["hull"]
    for ob in (markings, load):
        ob.parent = hull
        ob.matrix_parent_inverse = hull.matrix_world.inverted()

    out = _HERE / "renders" / "heavy_load"
    out.mkdir(parents=True, exist_ok=True)
    total = sum(kg for _, kg in LOAD)
    mass = sum(bv.MASS.values()) - bv.MASS.get("payload_box", 100) + total
    lines = ["Vehicle Version 3, heavy-load presentation variant (visual only)", "",
             "Deck load (estimated):"]
    lines += [f"  {n:36s} {kg:4d} kg" for n, kg in LOAD]
    lines += [f"  {'total':36s} {total:4d} kg  ({total / 100:.1f}x the 100 kg rated payload)",
              "", f"All-up mass with this load: about {mass:.0f} kg (rated: 530 kg)",
              f"Cushion pressure rises by {mass / 530:.2f}x to about {1.0 * mass / 530:.1f} kPa; "
              f"static thrust-to-weight falls from 0.18 to about {940 / (mass * 9.81):.2f}.",
              "An overload demonstration only; the Gazebo model keeps the 100 kg payload.",
              "", "LiDAR clearance:"]
    cl, ok = lidar_clearance(top)
    lines += cl + [f"OVERALL: {'load clear of the LiDAR beams' if ok else 'LOAD BLOCKS THE LIDAR'}"]
    (out / "load_log.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))

    bv.build_stage()
    cams = bv.add_preview_cameras()
    cams["cargo"] = eu.add_camera("cam_cargo", (2.2, -2.6, 2.3), (-0.1, -0.1, 0.95), lens=35)
    cams["bow"] = eu.add_camera("cam_bow", (3.4, 1.2, 1.0), (1.2, 0.0, 0.62), lens=45)
    if do_render:
        eu.setup_render("CYCLES", res=(1280, 800), samples=32)
        for name in ("front_34", "rear_34", "side", "top", "cargo", "bow"):
            bpy.context.scene.frame_set(1)
            eu.render_to(cams[name], out / f"{name}.png")
        bpy.context.scene.frame_set(100)                 # HOVER mode
        eu.render_to(cams["front_34"], out / "hover_mode.png")
        bpy.context.scene.frame_set(1)
    bpy.context.scene.camera = cams["front_34"]
    bpy.ops.wm.save_as_mainfile(filepath=str(_HERE / "hovercraft_v3_heavy_load.blend"))
    print("Saved", _HERE / "hovercraft_v3_heavy_load.blend")


if __name__ == "__main__":
    main()
    if not bpy.app.binary_path:          # pip bpy: avoid the glTF shutdown segfault
        import os
        sys.stdout.flush()
        os._exit(0)
