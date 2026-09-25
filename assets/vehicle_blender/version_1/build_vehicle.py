"""
build_vehicle.py — procedural Blender model of the hover_recon amphibious
hovercraft (Phase 1 of the build plan).

Run headless from the repo root (WSL, or Windows Blender pointed at the WSL
path \\wsl.localhost\<distro>\home\<you>\One-Wish-Coders\...):

    blender --background --python assets/vehicle_blender/version_1/build_vehicle.py
    blender --background --python assets/vehicle_blender/version_1/build_vehicle.py -- --no-render

Outputs (all regenerated on every run, so the script is idempotent):
    assets/vehicle_blender/version_1/hovercraft.blend      editable model
    assets/vehicle_blender/version_1/renders/*.png         preview renders
    assets/vehicle_blender/version_1/renders/dimensions.txt  bbox / spec log
    autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/models/hovercraft/
        meshes/<link>.dae, meshes/<link>.glb               one file per link
        link_frames.json                                   link origins, joints,
                                                           masses, collisions

Conventions: metres, kilograms, REP-103 axes (+X forward, +Y left, +Z up).
Model frame origin = ground plane directly under the hull centre, with the
vehicle standing on its wheels (ground mode: legs down, suspension at static
sag). Hull-mounted parts are modelled in a "hull design frame" and lifted by
HULL_DZ in ground mode; in hover mode (4 cm cushion gap) the hull sits
HULL_DZ + SKIRT_BOTTOM_Z - 0.04 lower, which puts the Section 2 heights
(hull top ~0.45 m, LiDAR ~0.60 m) back where the plan specifies them.

Every Gazebo link is ONE Blender object whose origin is that link's joint
pivot (fan hub, wheel axle, leg pivot, suspension slider, rudder hinge,
sensor optical centre).
All rotation/scale is baked into the mesh; only the location is kept.
"""
import math
import sys
from pathlib import Path

import bpy
import bmesh
from mathutils import Matrix, Vector

# Make export_utils importable when run via `blender --python`.
try:
    _HERE = Path(__file__).resolve().parent
    sys.path.insert(0, str(_HERE))
    import export_utils as eu
except NameError:          # pasted into a live Blender session (no __file__)
    _HERE = None
    eu = globals().get("_EU_MODULE")   # optional: export_utils injected by caller

TAU = 2 * math.pi
MODEL_SUBDIR = "autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/models/hovercraft"

# ==========================================================================
# 1. PARAMETERS — Section 2 of the build plan. Change numbers here only.
# ==========================================================================
# Skirt / footprint (outer skirt = hull footprint 1.2 m x 0.7 m)
SKIRT_L, SKIRT_W = 1.20, 0.70
SKIRT_BOTTOM_Z = 0.05          # skirt bottom in the hull design frame
HULL_DZ = 0.05                 # hull lift in ground mode -> 0.10 m ground clearance
HOVER_GAP = 0.04               # target cushion gap in hover mode
HOVER_DROP = HULL_DZ + SKIRT_BOTTOM_Z - HOVER_GAP   # ground -> hover height change
BAG_R = 0.065                  # bag-skirt tube radius
BAG_Z = 0.14                   # bag-skirt tube centre height
BAG_CORNER_R = 0.16            # plan-view corner radius of bag centreline
N_FINGERS = 64                 # skirt finger segments around the perimeter
SKIRT_TOP_Z = BAG_Z + BAG_R    # top of the bag (0.205)
FINGER_FLEX = 0.05             # bottom band of flexible fingers: no collision

# Rigid hull
DECK_Z = 0.32                  # main deck height
BAY_TOP_Z = 0.45               # top of bow electronics bay = "hull top ~0.45 m"

# Lift fan (centreline, vertical axis, ducted)
LIFT_X = 0.14
LIFT_Z = 0.34                  # rotor plane
LIFT_ROTOR_R = 0.135

# Thrust fans (rear, horizontal axis, ducted)
THRUST_X = -0.475              # rotor plane
THRUST_Y = 0.175               # +/- from centreline
THRUST_Z = 0.45                # duct axis height
THRUST_ROTOR_R = 0.105
DUCT_X0, DUCT_X1 = -0.575, -0.415

# Rudders (hinge axis vertical, just behind each thrust duct)
RUDDER_X = -0.60

# Retractable wheel legs (swing-up, Sealegs-style) with coil-over suspension.
# All values below are in the GROUND-MODE model frame (not the hull frame).
WHEEL_R = 0.15                 # 0.30 m balloon tyres (low ground pressure on mud)
WHEEL_W = 0.08
LEG_X = 0.44                   # +/- fore/aft position of the leg pivots
LEG_Y = 0.38                   # leg plane (outboard of the 0.35 m skirt bulge)
WHEEL_Y = 0.45                 # wheel centre plane
LEG_PIVOT_Z = 0.36             # pivot height (just under the deck edge)
LEG_RETRACT = math.pi / 2      # 90 deg: fronts fold back, rears fold forward
SUSP_DROOP = 0.03              # suspension extension below static sag
SUSP_BUMP = 0.05               # suspension compression above static sag
SUSP_K = 1700.0                # N/m  -> ~0.03 m static sag at ~5 kg per corner
SUSP_C = 75.0                  # N s/m (~0.4 of critical damping)

# Payload box (sealed, orange)
PAY_X = -0.215                 # box centre, x
PAY_BASE_Z = 0.34              # box underside (sits on cradle rails)
PAY_L, PAY_W, PAY_H = 0.35, 0.25, 0.20

# Sensors
LIDAR_X, LIDAR_Z = 0.40, 0.70  # 16-ch 3D LiDAR optical centre (mast raised
                               # +0.10 m so the rear ducts/payload don't block
                               # the lower beams; 0.69 m in hover mode)
LIDAR_R, LIDAR_H = 0.0515, 0.0717
CAM_X, CAM_Z = 0.552, 0.39     # front RGB camera optical centre
RANGER_XY = (0.42, 0.20)       # 4 downward rangers under the hull corners
RANGER_Z = 0.15

# Masses (kg) — sum = 25.0 kg incl. 4 kg payload (Section 2)
MASS = {
    "hull": 10.52, "skirt": 1.50, "lift_fan": 0.80,
    "thrust_fan_left": 0.50, "thrust_fan_right": 0.50,
    "rudder_left": 0.10, "rudder_right": 0.10,
    "wheel_leg_fl": 0.40, "wheel_leg_fr": 0.40, "wheel_leg_rl": 0.40, "wheel_leg_rr": 0.40,
    "wheel_shock_fl": 0.20, "wheel_shock_fr": 0.20, "wheel_shock_rl": 0.20, "wheel_shock_rr": 0.20,
    "wheel_fl": 0.90, "wheel_fr": 0.90, "wheel_rl": 0.90, "wheel_rr": 0.90,
    "payload_box": 4.00, "lidar": 0.83, "camera": 0.15,
}

def MASS_SPRUNG_PER_CORNER():
    unsprung = sum(v for k, v in MASS.items() if k.startswith(("wheel_shock", "wheel_f", "wheel_r")))
    return (sum(MASS.values()) - unsprung) / 4


# ==========================================================================
# 2. MATERIALS — plain Principled BSDF colours only
# ==========================================================================
MAT_DEFS = {
    # name:        (linear RGB,               roughness, metallic)
    "olive":       ((0.115, 0.135, 0.040),     0.65, 0.0),
    "dark_green":  ((0.035, 0.055, 0.022),     0.60, 0.0),
    "rubber":      ((0.012, 0.012, 0.012),     0.85, 0.0),
    "gunmetal":    ((0.045, 0.047, 0.050),     0.40, 0.7),
    "aluminium":   ((0.550, 0.550, 0.560),     0.35, 1.0),
    "blade":       ((0.030, 0.030, 0.032),     0.45, 0.0),
    "orange":      ((1.000, 0.230, 0.010),     0.45, 0.0),
    "label":       ((0.800, 0.800, 0.780),     0.60, 0.0),
    "spring":      ((0.600, 0.450, 0.020),     0.35, 0.8),
    "bungee":      ((0.030, 0.034, 0.018),     0.80, 0.0),
    "lens":        ((0.005, 0.010, 0.030),     0.05, 0.0),
    "lamp":        ((0.900, 0.850, 0.600),     0.10, 0.0),
}
MATS = {}


def eu_ensure_nodes(idblock):
    if eu:
        return eu.ensure_nodes(idblock)
    idblock.use_nodes = True
    return idblock.node_tree


def make_materials():
    for name, (rgb, rough, metal) in MAT_DEFS.items():
        m = bpy.data.materials.new(name)
        bsdf = eu_ensure_nodes(m).nodes.get("Principled BSDF")
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Roughness"].default_value = rough
        bsdf.inputs["Metallic"].default_value = metal
        m.diffuse_color = (*rgb, 1.0)          # viewport solid colour
        MATS[name] = m


# ==========================================================================
# 3. GEOMETRY PRIMITIVES (each returns a fresh bmesh in local coordinates)
# ==========================================================================
def T(x=0.0, y=0.0, z=0.0):
    return Matrix.Translation(Vector((x, y, z)))


def R(axis, deg):
    return Matrix.Rotation(math.radians(deg), 4, axis)


def bm_box(sx, sy, sz, bevel=0.0):
    pb = bmesh.new()
    bmesh.ops.create_cube(pb, size=1.0)
    bmesh.ops.scale(pb, vec=(sx, sy, sz), verts=pb.verts)
    if bevel > 0:
        bmesh.ops.bevel(pb, geom=list(pb.edges), offset=bevel, segments=3,
                        affect='EDGES', profile=0.5, clamp_overlap=True)
    return pb


def bm_cyl(r, depth, seg=24, r2=None, bevel=0.0):
    """Cylinder (or cone if r2 given) along Z, centred on the origin."""
    pb = bmesh.new()
    bmesh.ops.create_cone(pb, cap_ends=True, segments=seg, radius1=r,
                          radius2=r if r2 is None else r2, depth=depth)
    if bevel > 0:   # round only the two rim edges (horizontal edges)
        rims = [e for e in pb.edges
                if abs(e.verts[0].co.z - e.verts[1].co.z) < 1e-6]
        bmesh.ops.bevel(pb, geom=rims, offset=bevel, segments=3,
                        affect='EDGES', profile=0.5, clamp_overlap=True)
    return pb


def bm_ring(r_in, r_out, z0, z1, seg=32):
    """Thick-walled tube (duct) along Z."""
    pb = bmesh.new()
    rings = []
    for i in range(seg):
        a = TAU * i / seg
        c, s = math.cos(a), math.sin(a)
        rings.append([pb.verts.new((r_in * c, r_in * s, z0)),
                      pb.verts.new((r_out * c, r_out * s, z0)),
                      pb.verts.new((r_out * c, r_out * s, z1)),
                      pb.verts.new((r_in * c, r_in * s, z1))])
    for i in range(seg):
        a, b = rings[i], rings[(i + 1) % seg]
        for k in range(4):
            pb.faces.new((a[k], b[k], b[(k + 1) % 4], a[(k + 1) % 4]))
    bmesh.ops.recalc_face_normals(pb, faces=pb.faces)
    return pb


def bm_tube(points, radius, seg=8, closed=False):
    """Round tube swept along a polyline (parallel-transport frames)."""
    pts = [Vector(p) for p in points]
    n = len(pts)
    tangents = []
    for i in range(n):
        if closed:
            t = pts[(i + 1) % n] - pts[i - 1]
        else:
            t = pts[min(i + 1, n - 1)] - pts[max(i - 1, 0)]
        tangents.append(t.normalized())
    ref = Vector((0, 0, 1)) if abs(tangents[0].z) < 0.9 else Vector((1, 0, 0))
    nrm = (ref - ref.dot(tangents[0]) * tangents[0]).normalized()
    pb = bmesh.new()
    rings = []
    for i in range(n):
        t = tangents[i]
        nrm = (nrm - nrm.dot(t) * t).normalized()
        bi = t.cross(nrm)
        rings.append([pb.verts.new(pts[i] + radius * (math.cos(TAU * k / seg) * nrm
                                                     + math.sin(TAU * k / seg) * bi))
                      for k in range(seg)])
    last = n if closed else n - 1
    for i in range(last):
        a, b = rings[i], rings[(i + 1) % n]
        for k in range(seg):
            pb.faces.new((a[k], a[(k + 1) % seg], b[(k + 1) % seg], b[k]))
    if not closed:
        pb.faces.new(rings[0])
        pb.faces.new(list(reversed(rings[-1])))
    bmesh.ops.recalc_face_normals(pb, faces=pb.faces)
    return pb


def circle_pts(r, n=24, plane="XY"):
    pts = []
    for i in range(n):
        a = TAU * i / n
        c, s = r * math.cos(a), r * math.sin(a)
        pts.append({"XY": (c, s, 0), "YZ": (0, c, s), "XZ": (c, 0, s)}[plane])
    return pts


def rr_outline(xmin, xmax, hy, rc, nc=6, nsx=6, nsy=3):
    """Rounded-rectangle outline in plan view, CCW. Returns [(p2d, n2d)].
    Point count is fixed by (nc, nsx, nsy) so outlines can be lofted."""
    rc = min(rc, hy - 1e-4, (xmax - xmin) / 2 - 1e-4)
    out = []

    def straight(p0, p1, nrm, ns):
        for i in range(ns):
            t = i / ns
            out.append((p0.lerp(p1, t), nrm))

    def corner(c, a0):
        for i in range(nc):
            a = math.radians(a0 + 90.0 * i / nc)
            nrm = Vector((math.cos(a), math.sin(a)))
            out.append((c + rc * nrm, nrm))

    V = lambda x, y: Vector((x, y))
    straight(V(xmax, -(hy - rc)), V(xmax, hy - rc), V(1, 0), nsy)
    corner(V(xmax - rc, hy - rc), 0)
    straight(V(xmax - rc, hy), V(xmin + rc, hy), V(0, 1), nsx)
    corner(V(xmin + rc, hy - rc), 90)
    straight(V(xmin, hy - rc), V(xmin, -(hy - rc)), V(-1, 0), nsy)
    corner(V(xmin + rc, -(hy - rc)), 180)
    straight(V(xmin + rc, -hy), V(xmax - rc, -hy), V(0, -1), nsx)
    corner(V(xmax - rc, -(hy - rc)), 270)
    return out


def resample_closed(outline, n):
    """Evenly spaced points (with normals) along a closed outline."""
    pts = [p for p, _ in outline] + [outline[0][0]]
    seglen = [(pts[i + 1] - pts[i]).length for i in range(len(pts) - 1)]
    total = sum(seglen)
    res, i, acc = [], 0, 0.0
    for k in range(n):
        target = total * k / n
        while acc + seglen[i] < target:
            acc += seglen[i]
            i += 1
        t = (target - acc) / seglen[i]
        p = pts[i].lerp(pts[i + 1], t)
        tan = (pts[i + 1] - pts[i]).normalized()
        res.append((p, Vector((tan.y, -tan.x))))   # outward normal (CCW)
    return res


def bm_loft(profiles):
    """Loft closed rounded-rect profiles [(z, xmin, xmax, hy, rc), ...]
    bottom→top into a capped solid."""
    pb = bmesh.new()
    loops = []
    for z, xmin, xmax, hy, rc in profiles:
        loops.append([pb.verts.new((p.x, p.y, z))
                      for p, _ in rr_outline(xmin, xmax, hy, rc)])
    m = len(loops[0])
    for a, b in zip(loops[:-1], loops[1:]):
        for k in range(m):
            pb.faces.new((a[k], a[(k + 1) % m], b[(k + 1) % m], b[k]))
    pb.faces.new(list(reversed(loops[0])))
    pb.faces.new(loops[-1])
    bmesh.ops.recalc_face_normals(pb, faces=pb.faces)
    return pb


def bm_blade(r0, r1, c0, c1, thick, pitch_deg):
    """Fan blade lying along +X (radial) in the XY plane, pitched about X.
    The rotor axis is +Z."""
    pb = bmesh.new()
    vs = []
    for r, c in ((r0, c0), (r1, c1)):
        for y, z in ((-c / 2, -thick / 2), (c / 2, -thick / 2),
                     (c / 2, thick / 2), (-c / 2, thick / 2)):
            vs.append(pb.verts.new((r, y, z)))
    quads = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
             (2, 6, 7, 3), (3, 7, 4, 0)]
    for q in quads:
        pb.faces.new([vs[i] for i in q])
    # twist: tip pitched a bit less than root, like a real propeller
    for v in vs:
        frac = (v.co.x - r0) / (r1 - r0)
        ang = math.radians(pitch_deg * (1.0 - 0.35 * frac))
        y, z = v.co.y, v.co.z
        v.co.y = y * math.cos(ang) - z * math.sin(ang)
        v.co.z = y * math.sin(ang) + z * math.cos(ang)
    bmesh.ops.recalc_face_normals(pb, faces=pb.faces)
    return pb


# ==========================================================================
# 4. LINK BUILDER — accumulates parts into one mesh per Gazebo link
# ==========================================================================
class Link:
    def __init__(self, name, origin, dz=HULL_DZ):
        """dz lifts the whole link: hull-mounted links are authored in the
        hull design frame (dz = HULL_DZ); the wheel assembly is authored
        directly in the ground-mode frame (dz = 0)."""
        self.name = name
        self.dz = dz
        self.origin = Vector(origin) + Vector((0, 0, dz))
        self.bm = bmesh.new()
        self.mats = []
        self.tags = {}          # tag -> (min, max) world bbox, for the log

    def add(self, pb, mat, M=Matrix.Identity(4), tag=None):
        """Copy part bmesh `pb` (local coords) into this link, transformed by
        world matrix M."""
        if mat not in self.mats:
            self.mats.append(mat)
        mi = self.mats.index(mat)
        M = T(0, 0, self.dz) @ M
        vmap = {}
        for v in pb.verts:
            vmap[v] = self.bm.verts.new(M @ v.co)
        for f in pb.faces:
            nf = self.bm.faces.new([vmap[v] for v in f.verts])
            nf.material_index = mi
        if tag:
            co = [vmap[v].co for v in pb.verts]
            lo = Vector([min(c[i] for c in co) for i in range(3)])
            hi = Vector([max(c[i] for c in co) for i in range(3)])
            if tag in self.tags:
                plo, phi = self.tags[tag]
                lo = Vector(map(min, lo, plo))
                hi = Vector(map(max, hi, phi))
            self.tags[tag] = (lo, hi)
        pb.free()

    def finish(self, collection):
        """Create the Blender object: mesh re-centred on the link origin."""
        bmesh.ops.remove_doubles(self.bm, verts=self.bm.verts, dist=1e-6)
        bmesh.ops.transform(self.bm, matrix=T(*(-self.origin)),
                            verts=self.bm.verts)
        me = bpy.data.meshes.new(self.name)
        self.bm.to_mesh(me)
        self.bm.free()
        for m in self.mats:
            me.materials.append(MATS[m])
        me.shade_smooth()
        me.set_sharp_from_angle(angle=math.radians(35))
        ob = bpy.data.objects.new(self.name, me)
        collection.objects.link(ob)
        ob.location = self.origin
        return ob


# ==========================================================================
# 5. THE VEHICLE
# ==========================================================================
def build_skirt():
    L = Link("skirt", (0, 0, BAG_Z))
    hx = SKIRT_L / 2 - BAG_R
    hy = SKIRT_W / 2 - BAG_R
    # Bag skirt: a tube swept around the rounded-rect perimeter
    outline = rr_outline(-hx, hx, hy, BAG_CORNER_R, nc=10, nsx=14, nsy=6)
    pb = bmesh.new()
    seg = 14
    rings = []
    for p, n in outline:
        n3 = Vector((n.x, n.y, 0))
        c = Vector((p.x, p.y, BAG_Z))
        rings.append([pb.verts.new(c + BAG_R * (math.cos(TAU * k / seg) * n3
                                                + math.sin(TAU * k / seg) * Vector((0, 0, 1))))
                      for k in range(seg)])
    for i in range(len(rings)):
        a, b = rings[i], rings[(i + 1) % len(rings)]
        for k in range(seg):
            pb.faces.new((a[k], b[k], b[(k + 1) % seg], a[(k + 1) % seg]))
    bmesh.ops.recalc_face_normals(pb, faces=pb.faces)
    L.add(pb, "rubber", tag="bag")

    # Segmented fingers hanging under the outer edge of the bag
    for p, n in resample_closed(outline, N_FINGERS):
        n3 = Vector((n.x, n.y, 0))
        t3 = Vector((-n.y, n.x, 0))
        c = Vector((p.x, p.y, 0))
        top, bot = c + 0.045 * n3 + Vector((0, 0, 0.11)), \
            c + 0.020 * n3 + Vector((0, 0, SKIRT_BOTTOM_Z))
        fb = bmesh.new()
        vs = []
        for base, w in ((bot, 0.040), (top, 0.048)):
            for dt, dn in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                vs.append(fb.verts.new(base + dt * w / 2 * t3 + dn * 0.004 * n3))
        for q in [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
                  (2, 6, 7, 3), (3, 7, 4, 0)]:
            fb.faces.new([vs[i] for i in q])
        bmesh.ops.recalc_face_normals(fb, faces=fb.faces)
        L.add(fb, "rubber", tag="fingers")
    return L


def build_hull():
    L = Link("hull", (0, 0, 0.25))

    # --- main rigid hull (buoyancy / plenum body), raked bow ---------------
    L.add(bm_loft([
        (0.170, -0.500, 0.480, 0.270, 0.14),
        (0.210, -0.565, 0.555, 0.318, 0.16),
        (0.290, -0.575, 0.575, 0.330, 0.17),
        (DECK_Z, -0.565, 0.565, 0.320, 0.16),
    ]), "olive", tag="hull_body")
    # rub rail around the gunwale
    L.add(bm_tube([(p.x, p.y, 0.255) for p, _ in
                   rr_outline(-0.578, 0.572, 0.331, 0.17, nc=8, nsx=8, nsy=4)],
                  0.011, seg=8, closed=True), "gunmetal")
    # bow towing eye
    L.add(bm_box(0.012, 0.05, 0.035, bevel=0.003), "gunmetal", T(0.566, 0, 0.225))
    L.add(bm_tube(circle_pts(0.016, 16, "XZ"), 0.004, seg=6, closed=True),
          "gunmetal", T(0.585, 0, 0.225))

    # --- bow electronics bay (battery, computer), top = "hull top" ---------
    L.add(bm_loft([
        (DECK_Z - 0.01, 0.310, 0.555, 0.220, 0.05),
        (BAY_TOP_Z, 0.340, 0.465, 0.200, 0.04),
    ]), "dark_green", tag="bow_bay")
    for s in (1, -1):   # IR / work lamps on the sloped nose
        L.add(bm_cyl(0.018, 0.02, seg=16), "lamp",
              T(0.530, s * 0.12, 0.36) @ R('Y', 90))
        L.add(bm_ring(0.018, 0.022, -0.012, 0.012, seg=16), "gunmetal",
              T(0.530, s * 0.12, 0.36) @ R('Y', 90))
    # sensor mast for the LiDAR
    mast_top = LIDAR_Z - LIDAR_H / 2
    L.add(bm_cyl(0.016, mast_top - BAY_TOP_Z, seg=16), "gunmetal",
          T(LIDAR_X, 0, (BAY_TOP_Z + mast_top) / 2), tag="mast")
    L.add(bm_cyl(0.045, 0.006, seg=24), "gunmetal", T(LIDAR_X, 0, mast_top - 0.003))
    L.add(bm_cyl(0.03, 0.03, seg=16, r2=0.016), "gunmetal", T(LIDAR_X, 0, BAY_TOP_Z + 0.015))
    # GNSS puck + comms whip antenna
    L.add(bm_cyl(0.032, 0.016, seg=20, bevel=0.005), "blade", T(0.37, 0.12, BAY_TOP_Z + 0.008))
    L.add(bm_cyl(0.009, 0.03, seg=12), "gunmetal", T(0.37, -0.12, BAY_TOP_Z + 0.015))
    L.add(bm_cyl(0.0035, 0.25, seg=8), "blade", T(0.37, -0.12, BAY_TOP_Z + 0.155),
          tag="antenna")

    # --- lift fan duct (static parts) ---------------------------------------
    L.add(bm_ring(0.140, 0.152, 0.30, 0.385, seg=40), "olive", T(LIFT_X, 0, 0), tag="lift_duct")
    L.add(bm_tube(circle_pts(0.146, 40), 0.008, seg=8, closed=True), "gunmetal",
          T(LIFT_X, 0, 0.385))
    # guard grille over the intake
    for r in (0.100, 0.055):
        L.add(bm_tube(circle_pts(r, 32), 0.003, seg=6, closed=True), "blade",
              T(LIFT_X, 0, 0.393))
    for k in range(8):
        L.add(bm_cyl(0.003, 0.118, seg=6), "blade",
              T(LIFT_X, 0, 0.393) @ R('Z', 45 * k) @ T(0.087, 0, 0) @ R('Y', 90))
    L.add(bm_cyl(0.028, 0.006, seg=16), "blade", T(LIFT_X, 0, 0.393))
    # motor can + stator struts under the rotor
    L.add(bm_cyl(0.030, 0.035, seg=20), "gunmetal", T(LIFT_X, 0, 0.3075))
    for k in range(3):
        L.add(bm_box(0.112, 0.006, 0.015), "gunmetal",
              T(LIFT_X, 0, 0.31) @ R('Z', 120 * k + 30) @ T(0.085, 0, 0))

    # --- thrust fan ducts (static parts) ------------------------------------
    duct_len = DUCT_X1 - DUCT_X0
    for s in (1, -1):
        c = Vector((0, s * THRUST_Y, THRUST_Z))
        ax = T(*c) @ R('Y', 90)                       # local Z -> world +X
        L.add(bm_ring(0.110, 0.122, DUCT_X0, DUCT_X1, seg=40), "olive", ax,
              tag="thrust_ducts")
        L.add(bm_tube(circle_pts(0.116, 40, "YZ"), 0.008, seg=8, closed=True),
              "gunmetal", T(DUCT_X1, c.y, c.z))
        for r in (0.075, 0.040):                      # intake guard
            L.add(bm_tube(circle_pts(r, 32, "YZ"), 0.003, seg=6, closed=True),
                  "blade", T(DUCT_X1 + 0.01, c.y, c.z))
        for k in range(8):
            L.add(bm_cyl(0.003, 0.096, seg=6), "blade",
                  T(DUCT_X1 + 0.01, c.y, c.z) @ R('X', 45 * k) @ T(0, 0.068, 0) @ R('X', 90))
        L.add(bm_cyl(0.022, 0.006, seg=16), "blade",
              T(DUCT_X1 + 0.01, c.y, c.z) @ R('Y', 90))
        # motor can + stators behind the rotor
        L.add(bm_cyl(0.028, 0.04, seg=20), "gunmetal", T(-0.515, c.y, c.z) @ R('Y', 90))
        for k in range(3):
            L.add(bm_box(0.006, 0.084, 0.015), "gunmetal",
                  T(-0.52, c.y, c.z) @ R('X', 120 * k + 90) @ T(0, 0.069, 0))
        # saddle mount on the deck
        L.add(bm_box(0.11, 0.07, 0.03, bevel=0.004), "dark_green",
              T((DUCT_X0 + DUCT_X1) / 2, c.y, DECK_Z + 0.008))
        # rudder hinge brackets (top & bottom)
        for z in (THRUST_Z + 0.115, THRUST_Z - 0.115):
            L.add(bm_box(0.035, 0.016, 0.010), "gunmetal",
                  T(DUCT_X0 - 0.015, c.y, z))
    # cross-brace between the two ducts
    L.add(bm_box(0.06, 2 * (THRUST_Y - 0.118), 0.03, bevel=0.004), "dark_green",
          T((DUCT_X0 + DUCT_X1) / 2, 0, THRUST_Z + 0.03))

    # --- wheel-leg pivot mounts: bracket + rotary retract actuator ---------
    pz = LEG_PIVOT_Z - HULL_DZ                  # pivot height in hull frame
    for sx in (1, -1):
        for sy in (1, -1):
            x = sx * LEG_X
            L.add(bm_box(0.10, 0.05, 0.09, bevel=0.005), "dark_green",
                  T(x, sy * 0.335, pz - 0.015), tag="leg_mounts")
            L.add(bm_cyl(0.042, 0.022, seg=24, bevel=0.003), "gunmetal",
                  T(x, sy * 0.352, pz) @ R('X', 90))          # actuator drum
            L.add(bm_cyl(0.011, 0.075, seg=12), "aluminium",
                  T(x, sy * (LEG_Y - 0.005), pz) @ R('X', 90))  # pivot pin

    # --- lifting handles (2 per side, clear of the folded legs) ------------
    for sy in (1, -1):
        for hx in (-0.10, 0.10):
            pts = [(hx - 0.055, 0.322, 0.278), (hx - 0.055, 0.350, 0.278),
                   (hx - 0.045, 0.362, 0.278), (hx + 0.045, 0.362, 0.278),
                   (hx + 0.055, 0.350, 0.278), (hx + 0.055, 0.322, 0.278)]
            L.add(bm_tube([(p[0], sy * p[1], p[2]) for p in pts], 0.007, seg=8),
                  "gunmetal")

    # --- payload cradle: rails + corner locating posts ----------------------
    for sy in (1, -1):
        L.add(bm_box(PAY_L + 0.03, 0.03, 0.02), "gunmetal",
              T(PAY_X, sy * 0.09, DECK_Z + 0.01))
        for sx in (1, -1):
            L.add(bm_box(0.02, 0.02, 0.06, bevel=0.002), "gunmetal",
                  T(PAY_X + sx * (PAY_L / 2 + 0.01), sy * (PAY_W / 2 + 0.01), DECK_Z + 0.03))
        # battery access hatches beside the payload
        L.add(bm_box(0.30, 0.07, 0.006, bevel=0.002), "dark_green",
              T(PAY_X, sy * 0.24, DECK_Z + 0.003))

    # --- payload tie-down: two elastic bungee straps over the lid ----------
    # Each strap runs deck D-ring -> up the box side -> across the lid ->
    # down the other side -> D-ring, and is hooked at both ends.
    top = PAY_BASE_Z + PAY_H
    for dx in (-0.11, 0.11):
        x = PAY_X + dx
        ya, yb = PAY_W / 2 + 0.035, PAY_W / 2 + 0.009     # anchor, box side
        zt = top + 0.012                                   # over the lid ribs
        zb = PAY_BASE_Z + 0.03
        pts = [(-ya + 0.012, DECK_Z + 0.024), (-yb - 0.002, zb), (-yb, zt - 0.03),
               (-yb, zt - 0.008), (-yb + 0.012, zt), (yb - 0.012, zt),
               (yb, zt - 0.008), (yb, zt - 0.03), (yb + 0.002, zb), (ya - 0.012, DECK_Z + 0.024)]
        L.add(bm_tube([(x, y, z) for y, z in pts], 0.0065, seg=8), "bungee",
              tag="straps")
        for sy in (1, -1):
            # deck D-ring on a small plate
            L.add(bm_box(0.04, 0.03, 0.004), "gunmetal", T(x, sy * ya, DECK_Z + 0.002))
            L.add(bm_tube(circle_pts(0.012, 12, "YZ"), 0.0025, seg=6, closed=True),
                  "gunmetal", T(x, sy * ya, DECK_Z + 0.016) @ R('Z', 90))
            # hook at the strap end
            L.add(bm_tube([(x, sy * (ya - 0.012), DECK_Z + 0.022),
                           (x, sy * (ya - 0.002), DECK_Z + 0.030),
                           (x, sy * (ya + 0.006), DECK_Z + 0.022),
                           (x, sy * (ya + 0.004), DECK_Z + 0.012)], 0.003, seg=6),
                  "aluminium")

    # --- downward range sensors (cushion altimeters) under hull corners -----
    for sx in (1, -1):
        for sy in (1, -1):
            L.add(bm_cyl(0.015, 0.02, seg=12), "blade",
                  T(sx * RANGER_XY[0], sy * RANGER_XY[1], 0.17 - 0.01))
    return L


def build_lift_fan():
    L = Link("lift_fan", (LIFT_X, 0, LIFT_Z))
    M = T(LIFT_X, 0, LIFT_Z)
    L.add(bm_cyl(0.032, 0.028, seg=20, bevel=0.004), "aluminium", M)
    L.add(bm_cyl(0.020, 0.012, seg=16, r2=0.006), "aluminium", M @ T(0, 0, 0.02))
    for k in range(7):
        L.add(bm_blade(0.028, LIFT_ROTOR_R, 0.050, 0.035, 0.004, 28), "blade",
              M @ R('Z', 360 / 7 * k))
    return L


def build_thrust_fan(side):
    s = 1 if side == "left" else -1
    L = Link(f"thrust_fan_{side}", (THRUST_X, s * THRUST_Y, THRUST_Z))
    M = T(THRUST_X, s * THRUST_Y, THRUST_Z) @ R('Y', 90)   # rotor axis -> +X
    L.add(bm_cyl(0.030, 0.030, seg=20, bevel=0.004), "aluminium", M)
    L.add(bm_cyl(0.028, 0.022, seg=16, r2=0.006), "aluminium", M @ T(0, 0, 0.026))
    for k in range(5):
        L.add(bm_blade(0.026, THRUST_ROTOR_R, 0.045, 0.030, 0.004, 32), "blade",
              M @ R('Z', 72 * k))
    return L


def build_rudder(side):
    s = 1 if side == "left" else -1
    L = Link(f"rudder_{side}", (RUDDER_X, s * THRUST_Y, THRUST_Z))
    y = s * THRUST_Y
    chord, span = 0.067, 0.21
    L.add(bm_box(chord, 0.008, span, bevel=0.003), "dark_green",
          T(RUDDER_X - chord / 2 + 0.012, y, THRUST_Z), tag="vane")
    L.add(bm_cyl(0.005, 0.236, seg=8), "aluminium", T(RUDDER_X, y, THRUST_Z))
    return L


def _corner(corner):
    sx = 1 if corner[0] == "f" else -1
    sy = 1 if corner[1] == "l" else -1
    return sx, sy


def build_leg(corner):
    """Upper leg: pivots on the hull side (retract joint). Carries the upper
    shock tube and the coil spring."""
    sx, sy = _corner(corner)
    x, y = sx * LEG_X, sy * LEG_Y
    axle_z = WHEEL_R
    L = Link(f"wheel_leg_{corner}", (x, y, LEG_PIVOT_Z), dz=0)
    L.add(bm_cyl(0.032, 0.032, seg=20, bevel=0.004), "olive",
          T(x, y, LEG_PIVOT_Z) @ R('X', 90))                    # pivot boss
    top, bot = LEG_PIVOT_Z - 0.02, axle_z + 0.095
    L.add(bm_cyl(0.019, top - bot, seg=16), "dark_green", T(x, y, (top + bot) / 2),
          tag="leg")                                              # upper tube
    L.add(bm_cyl(0.034, 0.008, seg=20), "gunmetal", T(x, y, bot + 0.07))   # spring seat
    turns, z0, z1, rr = 6, bot + 0.066, bot - 0.035, 0.029
    helix = [(x + rr * math.cos(TAU * t / 16), y + rr * math.sin(TAU * t / 16),
              z0 + (z1 - z0) * t / (16 * turns)) for t in range(16 * turns + 1)]
    L.add(bm_tube(helix, 0.0042, seg=6), "spring", tag="spring")  # coil spring
    return L


def build_shock(corner):
    """Lower leg: slides along the leg axis (suspension joint) and carries the
    stub axle. Joint origin = axle centre at static sag."""
    sx, sy = _corner(corner)
    x, y = sx * LEG_X, sy * LEG_Y
    az = WHEEL_R
    L = Link(f"wheel_shock_{corner}", (x, y, az), dz=0)
    L.add(bm_cyl(0.014, 0.16, seg=16), "aluminium", T(x, y, az + 0.08))     # slider
    L.add(bm_cyl(0.034, 0.008, seg=20), "gunmetal", T(x, y, az + 0.058))    # lower seat
    L.add(bm_box(0.045, 0.034, 0.06, bevel=0.005), "gunmetal", T(x, y, az + 0.01))
    L.add(bm_cyl(0.013, WHEEL_Y - LEG_Y, seg=12), "aluminium",
          T(x, sy * (LEG_Y + WHEEL_Y) / 2, az) @ R('X', 90))              # stub axle
    return L


def build_wheel(corner):
    sx, sy = _corner(corner)
    c = Vector((sx * LEG_X, sy * WHEEL_Y, WHEEL_R))
    L = Link(f"wheel_{corner}", c, dz=0)
    M = T(*c) @ R('X', 90)                              # wheel axis -> Y
    L.add(bm_cyl(WHEEL_R - 0.009, WHEEL_W, seg=36, bevel=0.024), "rubber", M,
          tag="tyre")                                   # balloon carcass
    for k in range(24):                                 # chevron tread lugs
        for side in (-1, 1):
            L.add(bm_box(0.026, 0.009, WHEEL_W * 0.42), "rubber",
                  M @ R('Z', 15 * k + 7.5 * (side > 0)) @ T(0, WHEEL_R - 0.0045, side * WHEEL_W * 0.2)
                  @ R('Y', 18 * side), tag="tyre")
    L.add(bm_cyl(0.088, WHEEL_W + 0.004, seg=28, bevel=0.004), "aluminium", M)  # rim face
    L.add(bm_cyl(0.028, WHEEL_W + 0.010, seg=16, bevel=0.003), "gunmetal", M)
    for k in range(6):                                  # wheel nuts
        L.add(bm_cyl(0.006, WHEEL_W + 0.004, seg=6), "aluminium",
              M @ R('Z', 60 * k) @ T(0.045, 0, 0))
    return L


def build_payload():
    L = Link("payload_box", (PAY_X, 0, PAY_BASE_Z))
    body_h, gasket_h = 0.160, 0.006
    lid_h = PAY_H - body_h - gasket_h
    z_body = PAY_BASE_Z + body_h / 2
    z_gasket = PAY_BASE_Z + body_h + gasket_h / 2
    z_lid = PAY_BASE_Z + body_h + gasket_h + lid_h / 2
    top = PAY_BASE_Z + PAY_H
    L.add(bm_box(PAY_L, PAY_W, body_h, bevel=0.012), "orange", T(PAY_X, 0, z_body),
          tag="box_core")
    L.add(bm_box(PAY_L + 0.002, PAY_W + 0.002, gasket_h), "rubber",
          T(PAY_X, 0, z_gasket), tag="box_core")
    L.add(bm_box(PAY_L + 0.004, PAY_W + 0.004, lid_h, bevel=0.010), "orange",
          T(PAY_X, 0, z_lid), tag="box_core")
    # lid reinforcing ribs
    for dy in (-0.06, 0.06):
        L.add(bm_box(PAY_L - 0.06, 0.012, 0.006, bevel=0.002), "orange",
              T(PAY_X, dy, top + 0.002))
    # carry handle
    hx = 0.07
    pts = [(-hx, 0, top), (-hx, 0, top + 0.018), (-hx + 0.01, 0, top + 0.026),
           (hx - 0.01, 0, top + 0.026), (hx, 0, top + 0.018), (hx, 0, top)]
    L.add(bm_tube([(PAY_X + p[0], p[1], p[2]) for p in pts], 0.006, seg=8), "rubber")
    # hinges on the rear edge
    zh = PAY_BASE_Z + body_h + gasket_h / 2
    for dy in (-0.07, 0.07):
        L.add(bm_cyl(0.008, 0.05, seg=12), "gunmetal",
              T(PAY_X - PAY_L / 2 - 0.006, dy, zh) @ R('X', 90))
    # draw latches: two front, one each side
    for dy in (-0.07, 0.07):
        L.add(bm_box(0.012, 0.035, 0.045, bevel=0.002), "gunmetal",
              T(PAY_X + PAY_L / 2 + 0.006, dy, zh))
    for sy in (1, -1):
        L.add(bm_box(0.035, 0.012, 0.045, bevel=0.002), "gunmetal",
              T(PAY_X, sy * (PAY_W / 2 + 0.006), zh))
        # ID label plate
        L.add(bm_box(0.07, 0.003, 0.05), "label",
              T(PAY_X + 0.055, sy * (PAY_W / 2 + 0.001), PAY_BASE_Z + 0.08))
    # pressure-equalisation valve (sealed box detail)
    L.add(bm_cyl(0.012, 0.010, seg=12), "gunmetal", T(PAY_X + 0.148, 0.08, top + 0.004))
    return L


def build_lidar():
    L = Link("lidar", (LIDAR_X, 0, LIDAR_Z))
    b = LIDAR_Z - LIDAR_H / 2
    L.add(bm_cyl(LIDAR_R, 0.020, seg=32, bevel=0.003), "aluminium", T(LIDAR_X, 0, b + 0.010))
    L.add(bm_cyl(LIDAR_R - 0.002, 0.032, seg=32), "lens", T(LIDAR_X, 0, b + 0.036))
    L.add(bm_cyl(LIDAR_R, 0.020, seg=32, bevel=0.003), "aluminium", T(LIDAR_X, 0, b + 0.062))
    for k in range(12):   # cooling fins on the cap
        L.add(bm_box(0.004, 0.03, 0.006), "aluminium",
              T(LIDAR_X, 0, b + LIDAR_H + 0.001) @ R('Z', 15 * k))
    return L


def build_camera():
    L = Link("camera", (CAM_X, 0, CAM_Z))
    L.add(bm_box(0.05, 0.075, 0.05, bevel=0.005), "blade", T(CAM_X - 0.037, 0, CAM_Z))
    L.add(bm_cyl(0.016, 0.014, seg=20), "gunmetal", T(CAM_X - 0.007, 0, CAM_Z) @ R('Y', 90))
    L.add(bm_cyl(0.012, 0.002, seg=20), "lens", T(CAM_X - 0.001, 0, CAM_Z) @ R('Y', 90))
    L.add(bm_box(0.035, 0.085, 0.004), "blade", T(CAM_X - 0.03, 0, CAM_Z + 0.029))
    return L


# ==========================================================================
# 6. JOINT / FRAME TABLE  (drives link_frames.json -> model.sdf in Phase 2)
# ==========================================================================
def joint_table():
    J = {}
    J["skirt"] = ("hull", "fixed", None, None)
    J["lift_fan"] = ("hull", "continuous", (0, 0, 1), None)
    for side in ("left", "right"):
        J[f"thrust_fan_{side}"] = ("hull", "continuous", (1, 0, 0), None)
        J[f"rudder_{side}"] = ("hull", "revolute", (0, 0, 1),
                               (-math.radians(30), math.radians(30)))
    for c in ("fl", "fr", "rl", "rr"):
        sx, _ = _corner(c)
        # Retract: positive angle folds the leg up. Fronts fold backwards
        # (+Y axis), rears fold forwards (-Y axis) so the wheels tuck in
        # alongside the hull, clear of each other.
        J[f"wheel_leg_{c}"] = ("hull", "revolute", (0, sx, 0), (0.0, LEG_RETRACT),
                               {"control": "position", "note": "0 = leg down (GROUND mode); "
                                f"{LEG_RETRACT:.4f} = folded (HOVER mode)"})
        # Suspension: slider along the leg, held by a spring-damper.
        # +q = compression. q = 0 is static sag; the spring is relaxed at
        # q = -SUSP_DROOP, so at q = 0 it pushes with k * SUSP_DROOP ~ the
        # corner load.
        J[f"wheel_shock_{c}"] = (f"wheel_leg_{c}", "prismatic", (0, 0, 1),
                                 (-SUSP_DROOP, SUSP_BUMP),
                                 {"control": "spring_damper", "stiffness": SUSP_K,
                                  "damping": SUSP_C, "spring_reference": -SUSP_DROOP,
                                  "note": "+ = compression; 0 = static sag"})
        J[f"wheel_{c}"] = (f"wheel_shock_{c}", "continuous", (0, 1, 0), None)
    J["payload_box"] = ("hull", "fixed", None, None)
    J["lidar"] = ("hull", "fixed", None, None)
    J["camera"] = ("hull", "fixed", None, None)
    return {k: (v + (None,))[:5] for k, v in J.items()}


def collision_table():
    """Simple SDF primitives in each link's own frame (do NOT collide with the
    visual meshes)."""
    h = math.pi / 2
    C = {
        "hull": {"type": "box", "size": [1.15, 0.66, 0.15], "pose": [0, 0, -0.005, 0, 0, 0]},
        # Collision covers the bag only, not the bottom FINGER_FLEX of
        # flexible fingers. Debris lower than gap + FINGER_FLEX (~9 cm in
        # hover) brushes the fingers and passes under, like a real segmented
        # skirt; taller obstacles hit the bag and must be avoided.
        "skirt": {"type": "box",
                  "size": [SKIRT_L, SKIRT_W, SKIRT_TOP_Z - SKIRT_BOTTOM_Z - FINGER_FLEX],
                  "pose": [0, 0, (SKIRT_TOP_Z + SKIRT_BOTTOM_Z + FINGER_FLEX) / 2 - BAG_Z, 0, 0, 0]},
        "lift_fan": {"type": "cylinder", "radius": LIFT_ROTOR_R, "length": 0.03,
                     "pose": [0, 0, 0, 0, 0, 0]},
        "payload_box": {"type": "box", "size": [PAY_L, PAY_W, PAY_H],
                        "pose": [0, 0, PAY_H / 2, 0, 0, 0]},
        "lidar": {"type": "cylinder", "radius": LIDAR_R, "length": LIDAR_H,
                  "pose": [0, 0, 0, 0, 0, 0]},
        "camera": {"type": "box", "size": [0.05, 0.075, 0.05],
                   "pose": [-0.037, 0, 0, 0, 0, 0]},
    }
    for side in ("left", "right"):
        C[f"thrust_fan_{side}"] = {"type": "cylinder", "radius": THRUST_ROTOR_R,
                                   "length": 0.03, "pose": [0, 0, 0, 0, h, 0]}
        C[f"rudder_{side}"] = {"type": "box", "size": [0.067, 0.008, 0.21],
                               "pose": [-0.0215, 0, 0, 0, 0, 0]}
    top, bot = LEG_PIVOT_Z - 0.02, WHEEL_R + 0.095
    for c in ("fl", "fr", "rl", "rr"):
        C[f"wheel_leg_{c}"] = {"type": "cylinder", "radius": 0.019, "length": top - bot,
                               "pose": [0, 0, (top + bot) / 2 - LEG_PIVOT_Z, 0, 0, 0]}
        C[f"wheel_shock_{c}"] = {"type": "cylinder", "radius": 0.014, "length": 0.16,
                                 "pose": [0, 0, 0.08, 0, 0, 0]}
        C[f"wheel_{c}"] = {"type": "cylinder", "radius": WHEEL_R, "length": WHEEL_W,
                           "pose": [0, 0, 0, h, 0, 0]}
    return C


def sensor_table():
    S = {
        "imu": {"parent": "hull", "xyz": [0.0, 0.0, 0.25 + HULL_DZ], "rpy": [0, 0, 0],
                "type": "imu"},
        "navsat": {"parent": "hull", "xyz": [0.37, 0.12, BAY_TOP_Z + 0.016 + HULL_DZ],
                   "rpy": [0, 0, 0], "type": "navsat"},
        "lidar_3d": {"parent": "lidar", "xyz": [LIDAR_X, 0, LIDAR_Z + HULL_DZ], "rpy": [0, 0, 0],
                     "type": "gpu_lidar", "channels": 16, "hfov_deg": 360,
                     "vfov_deg": [-15, 15], "range_m": [0.3, 30.0], "rate_hz": 10},
        "front_camera": {"parent": "camera", "xyz": [CAM_X, 0, CAM_Z + HULL_DZ], "rpy": [0, 0, 0],
                         "type": "camera"},
    }
    for c, (sx, sy) in {"fl": (1, 1), "fr": (1, -1), "rl": (-1, 1), "rr": (-1, -1)}.items():
        S[f"ranger_{c}"] = {"parent": "hull",
                            "xyz": [sx * RANGER_XY[0], sy * RANGER_XY[1], RANGER_Z + HULL_DZ],
                            "rpy": [0, math.pi / 2, 0],   # +X of sensor points down
                            "type": "gpu_lidar (1 sample, downward)"}
    return S


# ==========================================================================
# 7. ASSEMBLY, ANIMATION, EXPORT, RENDERS
# ==========================================================================
def build_all():
    make_materials()
    coll = bpy.data.collections.new("hovercraft")
    bpy.context.scene.collection.children.link(coll)
    builders = [build_hull, build_skirt, build_lift_fan,
                lambda: build_thrust_fan("left"), lambda: build_thrust_fan("right"),
                lambda: build_rudder("left"), lambda: build_rudder("right")]
    builders += [lambda c=c: build_leg(c) for c in ("fl", "fr", "rl", "rr")]
    builders += [lambda c=c: build_shock(c) for c in ("fl", "fr", "rl", "rr")]
    builders += [lambda c=c: build_wheel(c) for c in ("fl", "fr", "rl", "rr")]
    builders += [build_payload, build_lidar, build_camera]

    links, tags = {}, {}
    for b in builders:
        L = b()
        tags[L.name] = dict(L.tags)
        links[L.name] = L.finish(coll)
    for ob in links.values():
        if eu:
            eu.apply_transforms(ob)
    bpy.context.view_layer.update()

    # Parent hierarchy mirrors the SDF joint tree (keeps world transforms)
    root = bpy.data.objects.new("hovercraft_root", None)
    root.empty_display_type = 'ARROWS'
    root.empty_display_size = 0.3
    coll.objects.link(root)
    for name, (parent, *_rest) in joint_table().items():
        ob, p = links[name], links[parent]
        ob.parent = p
        ob.matrix_parent_inverse = p.matrix_world.inverted()
        bpy.context.view_layer.update()
    links["hull"].parent = root
    bpy.context.view_layer.update()
    return links, tags, root


def animate_mode_switch(links, root):
    """Frames 1-30 ground mode; 30-50 lift fan spin-up; 50-60 the cushion
    takes the weight and the suspension droops; 60-90 legs fold up and the
    hull settles to a 4 cm gap; 90-140 thrust fans run. Cosmetic only —
    handy for pitch videos. Frame 1 = rest pose."""
    scene = bpy.context.scene
    scene.frame_start, scene.frame_end = 1, 140
    scene.render.fps = 24

    def key(ob, path, frame, value, idx):
        getattr(ob, path)[idx] = value
        ob.keyframe_insert(path, index=idx, frame=frame)

    for c in ("fl", "fr", "rl", "rr"):
        sx, _ = _corner(c)
        sh = links[f"wheel_shock_{c}"]
        z0 = sh.location.z
        key(sh, "location", 50, z0, 2)
        key(sh, "location", 60, z0 - SUSP_DROOP, 2)
        sh.location.z = z0
        leg = links[f"wheel_leg_{c}"]
        key(leg, "rotation_euler", 60, 0.0, 1)
        key(leg, "rotation_euler", 90, sx * LEG_RETRACT, 1)   # fronts back, rears fwd
        leg.rotation_euler[1] = 0.0
    key(root, "location", 60, 0.0, 2)
    key(root, "location", 90, -HOVER_DROP, 2)
    root.location.z = 0.0
    lf = links["lift_fan"]
    key(lf, "rotation_euler", 30, 0.0, 2)
    key(lf, "rotation_euler", 140, 50 * TAU, 2)
    for side in ("left", "right"):
        tf = links[f"thrust_fan_{side}"]
        key(tf, "rotation_euler", 90, 0.0, 0)
        key(tf, "rotation_euler", 140, 25 * TAU, 0)
    for ob in list(links.values()) + [root]:
        ad = ob.animation_data
        if ad and ad.action:
            fcurves = eu.iter_fcurves(ad.action) if eu else ad.action.fcurves
            for fc in fcurves:
                for kp in fc.keyframe_points:
                    spin = fc.data_path == "rotation_euler" and ob.name.startswith(
                        ("lift_fan", "thrust_fan"))
                    kp.interpolation = 'LINEAR' if spin else 'BEZIER'
    scene.frame_set(1)


def build_stage():
    """Preview-only scenery (not exported): grid floor, lights, world."""
    stage = bpy.data.collections.new("preview_stage")
    bpy.context.scene.collection.children.link(stage)
    me = bpy.data.meshes.new("floor")
    pb = bmesh.new()
    bmesh.ops.create_grid(pb, x_segments=1, y_segments=1, size=6.0)
    pb.to_mesh(me)
    pb.free()
    floor = bpy.data.objects.new("floor", me)
    stage.objects.link(floor)
    m = bpy.data.materials.new("floor_grid")
    nt = eu.ensure_nodes(m)
    bsdf = nt.nodes["Principled BSDF"]
    bsdf.inputs["Roughness"].default_value = 0.9
    chk = nt.nodes.new("ShaderNodeTexChecker")
    chk.inputs["Color1"].default_value = (0.42, 0.43, 0.44, 1)
    chk.inputs["Color2"].default_value = (0.36, 0.37, 0.38, 1)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nt.links.new(tc.outputs["Object"], chk.inputs["Vector"])
    chk.inputs["Scale"].default_value = 10.0          # object coords -> 0.1 m squares
    nt.links.new(chk.outputs["Color"], bsdf.inputs["Base Color"])
    me.materials.append(m)

    sun_d = bpy.data.lights.new("sun", 'SUN')
    sun_d.energy = 3.5
    sun_d.angle = math.radians(8)
    sun = bpy.data.objects.new("sun", sun_d)
    sun.rotation_euler = (math.radians(40), math.radians(-10), math.radians(-35))
    stage.objects.link(sun)
    fill_d = bpy.data.lights.new("fill", 'AREA')
    fill_d.energy = 250
    fill_d.size = 3
    fill = bpy.data.objects.new("fill", fill_d)
    fill.location = (-2.0, 2.5, 2.0)
    fill.rotation_euler = (Vector((0, 0, 0.3)) - fill.location).to_track_quat('-Z', 'Y').to_euler()
    stage.objects.link(fill)

    world = bpy.data.worlds.new("preview_world")
    wnt = eu.ensure_nodes(world)
    bg = wnt.nodes.get("Background") or wnt.nodes.new("ShaderNodeBackground")
    out = wnt.nodes.get("World Output") or wnt.nodes.new("ShaderNodeOutputWorld")
    if not out.inputs["Surface"].is_linked:
        wnt.links.new(bg.outputs["Background"], out.inputs["Surface"])
    bg.inputs["Color"].default_value = (0.62, 0.66, 0.72, 1)
    bg.inputs["Strength"].default_value = 0.8
    bpy.context.scene.world = world
    return stage


def measure(links, tags):
    """Bounding boxes + Section 2 checks. Returns (log lines, all_pass)."""
    lines, ok_all = [], True
    obs = list(links.values())
    lines.append("=== Per-link world bounding boxes (ground mode, metres) ===")
    total_tris = 0
    for name, ob in links.items():
        lo, hi = eu.world_bbox([ob])
        tris = eu.tri_count(ob)
        total_tris += tris
        lines.append(f"{name:18s} origin {eu.fmt_v(ob.matrix_world.translation)}  "
                     f"size {eu.fmt_v(hi - lo)}  tris {tris}")
    lo, hi = eu.world_bbox(obs)
    lines.append(f"\nOVERALL bbox min {eu.fmt_v(lo)} max {eu.fmt_v(hi)} size {eu.fmt_v(hi - lo)}")
    lines.append(f"TOTAL triangles: {total_tris}")

    def check(label, value, target, tol, unit="m"):
        nonlocal ok_all
        ok = abs(value - target) <= tol
        ok_all &= ok
        lines.append(f"[{'PASS' if ok else 'FAIL'}] {label:42s} {value:8.3f} {unit}"
                     f"  (spec {target} ± {tol})")

    lines.append("\n=== Section 2 spec checks ===")
    slo, shi = eu.world_bbox([links["skirt"]])
    check("Hull/skirt footprint length", shi.x - slo.x, SKIRT_L, 0.01)
    check("Hull/skirt footprint width", shi.y - slo.y, SKIRT_W, 0.01)
    check("Hull top (bow bay) height, hover mode", tags["hull"]["bow_bay"][1].z - HOVER_DROP,
          0.45, 0.02)
    check("LiDAR centre height, hover mode (mast +0.10)",
          links["lidar"].matrix_world.translation.z - HOVER_DROP, 0.70, 0.02)
    wlo, whi = eu.world_bbox([links["wheel_fl"]])
    check("Wheel diameter (larger, per team request)", whi.z - wlo.z, 2 * WHEEL_R, 0.005)
    wmin = min(eu.world_bbox([links[f"wheel_{c}"]])[0].z for c in ("fl", "fr", "rl", "rr"))
    check("Wheels touch ground (min z)", wmin, 0.0, 0.002)
    plo, phi = tags["payload_box"]["box_core"]
    check("Payload box length (core)", phi.x - plo.x, PAY_L, 0.01)
    check("Payload box width (core)", phi.y - plo.y, PAY_W, 0.01)
    check("Payload box height (core)", phi.z - plo.z, PAY_H, 0.01)
    check("Ground clearance (skirt bottom on wheels)", slo.z, SKIRT_BOTTOM_Z + HULL_DZ, 0.005)
    check("Suspension travel (droop + bump)", SUSP_DROOP + SUSP_BUMP, 0.08, 0.0)
    check("Static sag = corner load / k", MASS_SPRUNG_PER_CORNER() * 9.81 / SUSP_K,
          SUSP_DROOP, 0.004)
    mass = sum(MASS.values())
    check("Total mass", mass, 25.0, 0.01, "kg")
    tri_ok = total_tris < 50000
    ok_all &= tri_ok
    lines.append(f"[{'PASS' if tri_ok else 'FAIL'}] {'Triangle budget':42s} {total_tris:8d} tri"
                 f"  (spec < 50000)")

    # Derived numbers for the pitch
    area = SKIRT_L * SKIRT_W - (4 - math.pi) * (BAG_CORNER_R + BAG_R) ** 2
    lines.append("\n=== Derived ===")
    lines.append(f"Cushion area (rounded footprint)   {area:.3f} m^2   "
                 f"(plan's rectangle estimate 0.84 m^2)")
    lines.append(f"Cushion pressure  25 kg * 9.81 / A = {25 * 9.81 / area:.0f} Pa "
                 f"(plan ~290 Pa)")
    # Hover pose (frame 100): legs folded, hull 4 cm above ground
    scene = bpy.context.scene
    scene.frame_set(100)
    hs_lo, _ = eu.world_bbox([links["skirt"]])
    wb = {c: eu.world_bbox([links[f"wheel_{c}"]]) for c in ("fl", "fr", "rl", "rr")}
    wheel_bot = min(b[0].z for b in wb.values())
    gap_fr = wb["rl"][0].x - wb["fl"][1].x if False else wb["fl"][0].x - wb["rl"][1].x
    scene.frame_set(1)
    lines.append(f"Hover mode: skirt gap {hs_lo.z:.3f} m; folded wheels' lowest point "
                 f"{wheel_bot - hs_lo.z:+.3f} m above skirt bottom (must be > 0)")
    lines.append(f"Hover mode: fore/aft gap between folded front and rear wheels "
                 f"{gap_fr:.3f} m (must be > 0)")
    ok_all &= wheel_bot > hs_lo.z and gap_fr > 0
    lines.append(f"Ground mode: payload top z = {tags['payload_box']['box_core'][1].z:.3f} m, "
                 f"LiDAR beam plane z = {links['lidar'].matrix_world.translation.z:.3f} m")
    return lines, ok_all


def export_all(links, tags, repo):
    model_dir = repo / MODEL_SUBDIR
    mesh_dir = model_dir / "meshes"
    if mesh_dir.exists():                         # idempotent: wipe old meshes
        for f in mesh_dir.iterdir():
            if f.suffix in (".dae", ".glb"):
                f.unlink()
    J, C = joint_table(), collision_table()
    data = {
        "generated_by": "assets/vehicle_blender/version_1/build_vehicle.py",
        "blender_version": bpy.app.version_string,
        "units": "metres, kilograms, radians",
        "axes": "REP-103: +X forward, +Y left, +Z up",
        "model_frame": "ground plane under hull centre, vehicle standing on "
                       "wheels (ground mode: all joints at position 0, i.e. legs "
                       "down and suspension at static sag)",
        "note": "origin_xyz = link origin (joint pivot) in the model frame. "
                "origin_in_parent = pose to use for the SDF joint/link "
                "(no rotation anywhere: all link frames are axis-aligned).",
        "total_mass_kg": round(sum(MASS.values()), 3),
        "links": {}, "sensors": sensor_table(),
    }
    for name, ob in links.items():
        o = ob.matrix_world.translation
        entry = {"origin_xyz": [round(c, 4) for c in o], "mass_kg": MASS[name],
                 "mesh": {}, "collision": C[name]}
        lo, hi = eu.world_bbox([ob])
        entry["bbox_local"] = {"min": [round(c, 4) for c in lo - o],
                               "max": [round(c, 4) for c in hi - o]}
        if name in J:
            parent, jtype, axis, limits, extra = J[name]
            po = links[parent].matrix_world.translation
            entry["parent"] = parent
            entry["origin_in_parent"] = [round(c, 4) for c in o - po]
            entry["joint"] = {"name": f"{name}_joint", "type": jtype}
            if axis:
                entry["joint"]["axis"] = list(axis)
            if limits:
                entry["joint"]["lower"], entry["joint"]["upper"] = \
                    round(limits[0], 4), round(limits[1], 4)
            if extra:
                entry["joint"].update(extra)
        else:
            entry["parent"] = None
            entry["note"] = "root link (base_link)"
        for p in eu.export_link(ob, mesh_dir):
            entry["mesh"][p.suffix[1:]] = f"meshes/{p.name}"
        data["links"][name] = entry
    eu.write_json(model_dir / "link_frames.json", data)
    return model_dir


def add_preview_cameras():
    cams = {
        "front_34": eu.add_camera("cam_front_34", (2.05, -1.55, 1.05), (0.02, 0.0, 0.26), lens=50),
        "side": eu.add_camera("cam_side", (0.0, -4.0, 0.40), (0.0, 0.0, 0.40), ortho_scale=1.75),
        "top": eu.add_camera("cam_top", (0.0, 0.0, 4.0), (0.0, 0.0, 0.0), ortho_scale=1.55),
    }
    cams["top"].rotation_euler = (0, 0, 0)       # +X right, +Y up in image
    bpy.context.scene.camera = cams["front_34"]
    return cams


def render_previews(cams, renders_dir):
    eu.setup_render("CYCLES", res=(1280, 800), samples=40)
    for name, cam in cams.items():
        bpy.context.scene.frame_set(1)
        eu.render_to(cam, renders_dir / f"{name}.png")
    bpy.context.scene.frame_set(100)             # hover mode, wheels up
    eu.render_to(cams["front_34"], renders_dir / "hover_mode.png")
    bpy.context.scene.frame_set(1)
    bpy.context.scene.camera = cams["front_34"]


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    do_render = "--no-render" not in argv

    if eu:
        eu.clear_scene()
    links, tags, root = build_all()
    animate_mode_switch(links, root)

    if eu is None or _HERE is None:              # live session: just build
        return links

    # Repo root = first ancestor holding AGENTS.md (robust to the version_N/ nesting).
    repo = next(p for p in _HERE.parents if (p / "AGENTS.md").exists())
    renders_dir = _HERE / "renders"
    lines, ok = measure(links, tags)
    model_dir = export_all(links, tags, repo)
    lines.append(f"\nMeshes + link_frames.json written to {model_dir.relative_to(repo)}")
    if not eu.has_collada():
        lines.append("NOTE: this Blender has no COLLADA exporter (removed in 5.0) - "
                     "only .glb meshes were written; use those in the SDF.")
    lines.append(f"OVERALL: {'ALL CHECKS PASS' if ok else 'SOME CHECKS FAILED'}")
    renders_dir.mkdir(parents=True, exist_ok=True)
    (renders_dir / "dimensions.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))

    build_stage()
    cams = add_preview_cameras()
    if do_render:
        render_previews(cams, renders_dir)
    bpy.ops.wm.save_as_mainfile(filepath=str(_HERE / "hovercraft.blend"))
    print("Saved", _HERE / "hovercraft.blend")
    return links


if __name__ == "__main__":
    main()
    # When run with the pip `bpy` module (no Blender binary), its glTF add-on
    # can segfault during interpreter shutdown. Everything is already written
    # and flushed at this point, so exit cleanly. Inside real Blender
    # (`blender --background --python ...`) binary_path is set and this is skipped.
    if not bpy.app.binary_path:
        import os
        sys.stdout.flush()
        os._exit(0)
