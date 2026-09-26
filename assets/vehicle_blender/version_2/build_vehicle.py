"""
build_vehicle.py — procedural Blender model of Vehicle Version 2: a
hovercraft-dominant amphibious vehicle with a complete retractable tracked
undercarriage and controlled air-cushion load sharing (see AGENTS.md,
"Vehicle Version 2").

Run headless from the repo root (Blender 4.2+; on Windows point Blender at
\\wsl.localhost\<distro>\home\<you>\One-Wish-Coders\...):

    blender --background --python assets/vehicle_blender/version_2/build_vehicle.py
    blender --background --python assets/vehicle_blender/version_2/build_vehicle.py -- --no-render

It also runs with the pip `bpy` module (`python3 build_vehicle.py`).

Outputs (all regenerated on every run, so the script is idempotent):
    assets/vehicle_blender/version_2/hovercraft_v2.blend    editable model
    assets/vehicle_blender/version_2/renders/*.png          preview renders
    assets/vehicle_blender/version_2/renders/dimensions.txt bbox / spec log
    autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/models/hovercraft_v2/
        meshes/<link>.dae, meshes/<link>.glb                one file per link
        link_frames.json                                    link origins, joints,
                                                            masses, collisions
Then run tidal_vehicle_description/scripts/gen_description_v2.py to build the
SDF and URDF.

Conventions: metres, kilograms, REP-103 axes (+X forward, +Y left, +Z up).
Model frame origin = ground plane directly under the hull centre, with the
vehicle standing on its deployed tracks (TRACK mode: every joint at 0).
In HOVER mode the cushion lifts the whole vehicle by HOVER_RISE and the
tracks retract TRACK_STROKE up into wells in the hull.

Every Gazebo link is ONE Blender object whose origin is that link's joint
pivot (fan hub, track centre, rudder hinge, sensor optical centre).
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
MODEL_NAME = "hovercraft_v2"
MODEL_SUBDIR = f"autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/models/{MODEL_NAME}"

# ==========================================================================
# 1. PARAMETERS — AGENTS.md "Vehicle Version 2". Change numbers here only.
# ==========================================================================
# Every part is authored directly in the model frame (TRACK mode), so the
# Link builder's vertical offset is zero.
HULL_DZ = 0.0

# Overall envelope: 2.5 m long, 1.5 m wide, 1.5 m high (incl. LiDAR mast)
SKIRT_L, SKIRT_W = 2.50, 1.50          # skirt = outer footprint
OVERALL_H = 1.50

# Air cushion
SKIRT_BOTTOM_Z = 0.03          # skirt finger tips above ground in TRACK mode
HOVER_GAP = 0.05               # target cushion gap in HOVER mode
HOVER_RISE = HOVER_GAP - SKIRT_BOTTOM_Z    # TRACK -> HOVER height change (+)
BAG_R = 0.09                   # bag-skirt tube radius (slim so the tracks fit inboard)
BAG_Z = 0.23                   # bag-skirt tube centre height
BAG_CORNER_R = 0.30            # plan-view corner radius of the bag centreline
N_FINGERS = 96                 # skirt finger segments around the perimeter
SKIRT_TOP_Z = BAG_Z + BAG_R    # 0.32
FINGER_FLEX = 0.05             # bottom band of flexible fingers: no collision

# Hull
HULL_BOTTOM_Z = 0.30
DECK_Z = 0.72                  # main deck
BAY_TOP_Z = 0.95               # top of the bow electronics bay = "hull top" (<= 1.0 m)

# Tracks: two inboard rubber tracks inside the skirt footprint
TRACK_Y = 0.42                 # track centre plane (gauge 0.84 m)
TRACK_W = 0.28                 # belt width
TRACK_R = 0.14                 # sprocket / idler radius incl. belt -> loop height 0.28 m
TRACK_HALF = 0.70              # sprocket / idler centres at x = +/-0.70 (1.40 m contact)
TRACK_T = 0.022                # belt thickness
GROUSER_H = 0.016              # cleat height on the outside of the belt
TRACK_STROKE = 0.25            # vertical retract travel into the hull wells
TRACK_LIMIT_MARGIN = 0.02      # joint-limit clearance beyond 0 and TRACK_STROKE

# Lift fan (offset forward so the LiDAR mast sits on the centreline)
LIFT_X = 0.45
LIFT_Z = 0.74                  # rotor plane
LIFT_ROTOR_R = 0.28

# Rear ducted thrust fans + rudders
THRUST_X = -0.93               # rotor plane
THRUST_Y = 0.35                # +/- from centreline
THRUST_Z = 1.03                # duct axis height
THRUST_ROTOR_R = 0.25
DUCT_R_IN, DUCT_R_OUT = 0.265, 0.29
DUCT_X0, DUCT_X1 = -1.15, -0.80
RUDDER_X = -1.17
RUDDER_CHORD, RUDDER_SPAN = 0.10, 0.50
RUDDER_MAX_DEG = 25            # commanded deflection limit (joint limit is 30 deg)

# Puff ports: four side vents fed from the cushion plenum, one on each side
# at the bow and at the stern. A bow vent on one side plus a stern vent on
# the other make a yaw couple at any speed; same-side pairs push sideways.
PUFF_X = 0.74                  # +/- vent centre, x (on the straight topsides)
PUFF_Z = 0.51                  # vent centre height, between chine and rub rail
PUFF_W, PUFF_H = 0.24, 0.12    # outlet opening (m)
PUFF_CD = 0.8                  # discharge coefficient of the louvred outlet

# Payload box (sealed, orange): rated 30 kg
PAY_X = -0.42                  # box centre, x
PAY_BASE_Z = DECK_Z + 0.02     # box underside (sits on cradle rails)
PAY_L, PAY_W, PAY_H = 0.60, 0.45, 0.35

# Sensors
LIDAR_X, LIDAR_Z = 0.0, 1.44   # 16-ch 3D LiDAR on a centre mast, above base_link
LIDAR_R, LIDAR_H = 0.0515, 0.0717
CAM_X, CAM_Z = 1.07, 0.80      # front RGB camera optical centre (bow bay nose)
RANGER_XY = (1.00, 0.50)       # 4 downward rangers under the hull corners
RANGER_Z = HULL_BOTTOM_Z

# Masses (kg) — sum = 300 kg incl. the 30 kg rated payload
MASS = {
    "hull": 147.8, "skirt": 18.0, "lift_fan": 12.0,
    "thrust_fan_left": 10.0, "thrust_fan_right": 10.0,
    "rudder_left": 1.5, "rudder_right": 1.5,
    "track_left": 34.0, "track_right": 34.0,
    "payload_box": 30.0, "lidar": 1.0, "camera": 0.2,
}
# Hull mass is mostly the 10 kWh battery and electronics, carried low
HULL_COM_Z = 0.42


# ==========================================================================
# 2. MATERIALS — plain Principled BSDF colours only
# ==========================================================================
MAT_DEFS = {
    # name:        (linear RGB,               roughness, metallic)
    "olive":       ((0.115, 0.135, 0.040),     0.65, 0.0),
    "dark_green":  ((0.035, 0.055, 0.022),     0.60, 0.0),
    "rubber":      ((0.012, 0.012, 0.012),     0.85, 0.0),
    "neoprene":    ((0.018, 0.019, 0.020),     0.55, 0.0),    # coated skirt fabric
    "tail_lamp":   ((0.600, 0.020, 0.010),     0.15, 0.0),
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
        """dz lifts the whole link. Version 2 authors every part directly in
        the TRACK-mode model frame, so dz = HULL_DZ = 0."""
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



def bm_stadium_band(half_len, r_out, thick, width, n_arc=16):
    """Track belt: a closed stadium loop in the XZ plane (straight runs
    between x = +/-half_len, semicircular ends of radius r_out), `thick`
    deep radially, extruded `width` along Y. Open on the +/-Y faces, like a
    real belt."""
    def loop(r):
        pts = []
        for i in range(n_arc + 1):                         # front end, bottom -> top
            a = -math.pi / 2 + math.pi * i / n_arc
            pts.append((half_len + r * math.cos(a), r * math.sin(a)))
        for i in range(n_arc + 1):                         # rear end, top -> bottom
            a = math.pi / 2 + math.pi * i / n_arc
            pts.append((-half_len + r * math.cos(a), r * math.sin(a)))
        return pts
    outer, inner = loop(r_out), loop(r_out - thick)
    pb = bmesh.new()
    rings = []
    for (xo, zo), (xi, zi) in zip(outer, inner):
        rings.append([pb.verts.new((xo, -width / 2, zo)), pb.verts.new((xo, width / 2, zo)),
                      pb.verts.new((xi, width / 2, zi)), pb.verts.new((xi, -width / 2, zi))])
    n = len(rings)
    for i in range(n):
        a, b = rings[i], rings[(i + 1) % n]
        for k in range(4):
            pb.faces.new((a[k], b[k], b[(k + 1) % 4], a[(k + 1) % 4]))
    bmesh.ops.recalc_face_normals(pb, faces=pb.faces)
    return pb


def stadium_path(half_len, r, n):
    """n evenly spaced (x, z, outward-normal-angle) points around a stadium."""
    straight = 2 * half_len
    arc = math.pi * r
    total = 2 * straight + 2 * arc
    out = []
    for k in range(n):
        s = total * k / n
        if s < straight:                                   # bottom run, rear -> front
            out.append((-half_len + s, -r, -math.pi / 2))
            continue
        s -= straight
        if s < arc:                                        # front end, bottom -> top
            a = -math.pi / 2 + s / r
            out.append((half_len + r * math.cos(a), r * math.sin(a), a))
            continue
        s -= arc
        if s < straight:                                   # top run, front -> rear
            out.append((half_len - s, r, math.pi / 2))
            continue
        s -= straight                                      # rear end, top -> bottom
        a = math.pi / 2 + s / r
        out.append((-half_len + r * math.cos(a), r * math.sin(a), a))
    return out


# ==========================================================================
# 5. THE VEHICLE
# ==========================================================================
def build_skirt():
    L = Link("skirt", (0, 0, BAG_Z))
    hx = SKIRT_L / 2 - BAG_R
    hy = SKIRT_W / 2 - BAG_R
    # Bag skirt: a tube swept around the rounded-rect perimeter
    outline = rr_outline(-hx, hx, hy, BAG_CORNER_R, nc=12, nsx=24, nsy=12)
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
    L.add(pb, "neoprene", tag="bag")

    # Segmented fingers hanging from the underside of the bag. Each finger is
    # a curved, tapered plate: it leaves the bag just below its widest point,
    # follows the bag's outer curve and flares slightly outward at the tip,
    # like the loop-and-finger skirts of light hovercraft.
    n_seg = 5
    # Fingers overlap their neighbours slightly (no see-through gaps);
    # alternate ones sit 3 mm further out so the overlaps don't z-fight.
    for i_f, (p, n) in enumerate(resample_closed(outline, N_FINGERS)):
        n3 = Vector((n.x, n.y, 0))
        t3 = Vector((-n.y, n.x, 0))
        c = Vector((p.x, p.y, 0))
        stagger = -0.003 if i_f % 2 else 0.0
        fb = bmesh.new()
        rows = []
        for i in range(n_seg + 1):
            f = i / n_seg                                   # 0 = top, 1 = tip
            z = (BAG_Z - 0.02) + (SKIRT_BOTTOM_Z - (BAG_Z - 0.02)) * f
            out = 0.074 + stagger + 0.010 * math.sin(math.pi * f * 0.8)   # stays < BAG_R
            w = 0.086 - 0.008 * f                             # taper towards the tip
            th = 0.009 - 0.004 * f
            base = c + out * n3 + Vector((0, 0, z))
            rows.append([fb.verts.new(base + dt * w / 2 * t3 + dn * th / 2 * n3)
                         for dt, dn in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        for a, b in zip(rows[:-1], rows[1:]):
            for k in range(4):
                fb.faces.new((a[k], a[(k + 1) % 4], b[(k + 1) % 4], b[k]))
        fb.faces.new(list(reversed(rows[0])))
        fb.faces.new(rows[-1])
        bmesh.ops.recalc_face_normals(fb, faces=fb.faces)
        L.add(fb, "rubber", tag="fingers")
    return L


def build_hull():
    L = Link("hull", (0, 0, 0.50))

    # --- main rigid hull (buoyancy / plenum body), raked bow ---------------
    # The topsides start on the skirt-attachment flange directly over the bag
    # and tuck in slightly towards the deck, so hull and skirt read as one
    # craft instead of a tub standing on a ring.
    L.add(bm_loft([
        (HULL_BOTTOM_Z, -1.10, 1.02, 0.56, 0.26),
        (SKIRT_TOP_Z + 0.02, -1.19, 1.17, 0.665, 0.33),
        (0.42, -1.205, 1.195, 0.68, 0.34),
        (0.60, -1.19, 1.17, 0.665, 0.33),
        (0.67, -1.17, 1.14, 0.645, 0.31),
        (DECK_Z, -1.14, 1.10, 0.615, 0.29),
    ]), "olive", tag="hull_body")
    # skirt-attachment flange: a plate over the top of the bag, out to the
    # footprint edge, with the bag clamped underneath by a bolted strip
    fl_hx, fl_hy, fl_rc = SKIRT_L / 2 - 0.012, SKIRT_W / 2 - 0.012, BAG_CORNER_R + BAG_R - 0.01
    L.add(bm_loft([
        (SKIRT_TOP_Z - 0.012, -fl_hx, fl_hx, fl_hy, fl_rc),
        (SKIRT_TOP_Z + 0.022, -fl_hx, fl_hx, fl_hy, fl_rc),
    ]), "dark_green", tag="skirt_flange")
    clamp = rr_outline(-fl_hx + 0.004, fl_hx - 0.004, fl_hy - 0.004, fl_rc,
                       nc=10, nsx=20, nsy=10)
    L.add(bm_tube([(p.x, p.y, SKIRT_TOP_Z + 0.005) for p, _ in clamp],
                  0.008, seg=8, closed=True), "gunmetal")
    for p, n in resample_closed(clamp, 64):                   # clamp-strip bolts
        L.add(bm_cyl(0.008, 0.008, seg=8), "aluminium",
              T(p.x - 0.03 * n.x, p.y - 0.03 * n.y, SKIRT_TOP_Z + 0.026))
    # rub rail around the gunwale
    L.add(bm_tube([(p.x, p.y, 0.60) for p, _ in
                   rr_outline(-1.195, 1.175, 0.668, 0.33, nc=8, nsx=14, nsy=6)],
                  0.018, seg=8, closed=True), "gunmetal")
    # lower chine strake: breaks up the topsides and throws spray outward
    L.add(bm_tube([(p.x, p.y, 0.42) for p, _ in
                   rr_outline(-1.21, 1.20, 0.683, 0.34, nc=8, nsx=14, nsy=6)],
                  0.010, seg=6, closed=True), "dark_green")
    # battery-bay vent louvres on both sides (below the rub rail)
    for sy in (1, -1):
        y = sy * 0.676
        L.add(bm_box(0.46, 0.012, 0.10, bevel=0.004), "dark_green", T(-0.10, y, 0.505))
        for k in range(5):
            L.add(bm_box(0.42, 0.016, 0.008, bevel=0.002), "gunmetal",
                  T(-0.10, y + sy * 0.004, 0.468 + 0.019 * k))
        # registration / warning placard
        L.add(bm_box(0.16, 0.006, 0.06), "label", T(0.40, sy * 0.681, 0.51))
        # puff ports (bow and stern): framed outlet, dark duct mouth and
        # angled shutter vanes that open to one side or close the vent
        for px in (PUFF_X, -PUFF_X):
            L.add(bm_box(PUFF_W + 0.03, 0.014, PUFF_H + 0.03, bevel=0.005), "gunmetal",
                  T(px, sy * 0.679, PUFF_Z), tag="puff_ports")
            L.add(bm_box(PUFF_W, 0.012, PUFF_H), "blade", T(px, sy * 0.682, PUFF_Z))
            for k in range(5):
                L.add(bm_box(0.034, 0.005, PUFF_H - 0.008, bevel=0.001), "aluminium",
                      T(px - PUFF_W / 2 + 0.024 + k * (PUFF_W - 0.048) / 4,
                        sy * 0.686, PUFF_Z) @ R('Z', 35), tag="puff_ports")
    # stern: tail lamps and a transom step
    for sy in (1, -1):
        L.add(bm_box(0.012, 0.10, 0.04, bevel=0.004), "tail_lamp",
              T(-1.203, sy * 0.30, 0.53))
    L.add(bm_box(0.05, 0.46, 0.04, bevel=0.008), "gunmetal", T(-1.215, 0, 0.47))
    # anti-slip walkway panels along both deck edges
    for sy in (1, -1):
        L.add(bm_box(0.90, 0.16, 0.008), "dark_green", T(-0.05, sy * 0.49, DECK_Z + 0.004))
    # bow towing eye
    L.add(bm_box(0.02, 0.09, 0.06, bevel=0.005), "gunmetal", T(1.16, 0, 0.50))
    L.add(bm_tube(circle_pts(0.03, 16, "XZ"), 0.007, seg=6, closed=True),
          "gunmetal", T(1.19, 0, 0.50))

    # --- bow electronics bay (computer, comms), top = "hull top" -----------
    L.add(bm_loft([
        (DECK_Z - 0.01, 0.80, 1.10, 0.46, 0.10),
        (BAY_TOP_Z, 0.86, 1.00, 0.40, 0.08),
    ]), "dark_green", tag="bow_bay")
    # tinted side windows (status displays behind), following the bay's side slope
    side_tilt = math.degrees(math.atan2(0.06, BAY_TOP_Z - DECK_Z + 0.01))   # ~14 deg
    for s in (1, -1):
        L.add(bm_box(0.13, 0.006, 0.09, bevel=0.003), "lens",
              T(0.93, s * 0.432, 0.83) @ R('X', s * side_tilt))
        L.add(bm_box(0.15, 0.004, 0.11, bevel=0.003), "gunmetal",
              T(0.93, s * 0.429, 0.83) @ R('X', s * side_tilt))
    # camera bezel + sun visor on the sloped nose, around the front camera
    nose_tilt = -math.degrees(math.atan2(0.10, BAY_TOP_Z - DECK_Z + 0.01))
    L.add(bm_box(0.014, 0.17, 0.12, bevel=0.006), "gunmetal",
          T(CAM_X - 0.012, 0, CAM_Z) @ R('Y', nose_tilt))
    L.add(bm_box(0.05, 0.17, 0.010, bevel=0.003), "dark_green",
          T(CAM_X - 0.02, 0, CAM_Z + 0.066) @ R('Y', 8))
    # roof grab rail
    for s in (1, -1):
        L.add(bm_tube([(0.89, s * 0.33, BAY_TOP_Z), (0.89, s * 0.33, BAY_TOP_Z + 0.035),
                       (0.97, s * 0.33, BAY_TOP_Z + 0.035), (0.97, s * 0.33, BAY_TOP_Z)],
                      0.008, seg=8), "gunmetal")
    for s in (1, -1):   # IR / work lamps on the sloped nose
        L.add(bm_cyl(0.035, 0.03, seg=16), "lamp",
              T(1.07, s * 0.26, 0.82) @ R('Y', 65))
        L.add(bm_ring(0.035, 0.042, -0.018, 0.018, seg=16), "gunmetal",
              T(1.07, s * 0.26, 0.82) @ R('Y', 65))
    # GNSS puck + comms whip antenna on the bay roof
    L.add(bm_cyl(0.06, 0.03, seg=24, bevel=0.008), "blade", T(0.93, 0.25, BAY_TOP_Z + 0.015))
    L.add(bm_cyl(0.015, 0.05, seg=12), "gunmetal", T(0.93, -0.25, BAY_TOP_Z + 0.025))
    L.add(bm_cyl(0.005, 0.45, seg=8), "blade", T(0.93, -0.25, BAY_TOP_Z + 0.275),
          tag="antenna")

    # --- centre sensor mast for the LiDAR (directly above base_link) ------
    mast_top = LIDAR_Z - LIDAR_H / 2
    base_top = DECK_Z + 0.12
    # sloped base housing (mast electronics, cable gland)
    L.add(bm_loft([
        (DECK_Z - 0.005, LIDAR_X - 0.13, LIDAR_X + 0.13, 0.11, 0.05),
        (DECK_Z + 0.05, LIDAR_X - 0.12, LIDAR_X + 0.12, 0.10, 0.05),
        (base_top, LIDAR_X - 0.07, LIDAR_X + 0.07, 0.065, 0.05),
    ]), "dark_green", tag="mast")
    L.add(bm_box(0.004, 0.10, 0.04, bevel=0.002), "gunmetal",
          T(LIDAR_X + 0.123, 0, DECK_Z + 0.03))                     # service plate
    # faired, tapered mast
    L.add(bm_cyl(0.045, mast_top - 0.03 - base_top, seg=28, r2=0.034), "dark_green",
          T(LIDAR_X, 0, (base_top + mast_top - 0.03) / 2), tag="mast")
    for z, r in ((base_top + 0.005, 0.05), (base_top + 0.26, 0.043)):   # collars
        L.add(bm_cyl(r, 0.018, seg=28, bevel=0.004), "gunmetal", T(LIDAR_X, 0, z))
    # cable conduit up the back of the mast
    L.add(bm_tube([(LIDAR_X - 0.052, 0, base_top), (LIDAR_X - 0.045, 0, base_top + 0.25),
                   (LIDAR_X - 0.040, 0, mast_top - 0.04)], 0.008, seg=8), "blade")
    # head: flared neck and a bolted aluminium mounting plate under the LiDAR
    L.add(bm_cyl(0.034, 0.02, seg=28, r2=0.05), "gunmetal", T(LIDAR_X, 0, mast_top - 0.02))
    L.add(bm_cyl(0.056, 0.008, seg=32, bevel=0.002), "aluminium",
          T(LIDAR_X, 0, mast_top - 0.004))
    for k in range(4):
        L.add(bm_cyl(0.005, 0.004, seg=8), "gunmetal",
              T(LIDAR_X, 0, mast_top + 0.001) @ R('Z', 45 + 90 * k) @ T(0.046, 0, 0))

    # --- lift fan duct (static parts), offset forward ----------------------
    L.add(bm_ring(LIFT_ROTOR_R + 0.04, LIFT_ROTOR_R + 0.10, DECK_Z - 0.005, DECK_Z + 0.025,
                  seg=48), "dark_green", T(LIFT_X, 0, 0))    # raised deck plinth
    L.add(bm_ring(LIFT_ROTOR_R + 0.02, LIFT_ROTOR_R + 0.04, DECK_Z - 0.10, DECK_Z + 0.08, seg=48),
          "olive", T(LIFT_X, 0, 0), tag="lift_duct")
    L.add(bm_tube(circle_pts(LIFT_ROTOR_R + 0.03, 48), 0.014, seg=8, closed=True), "gunmetal",
          T(LIFT_X, 0, DECK_Z + 0.08))
    for r in (0.22, 0.14, 0.07):          # guard grille over the intake
        L.add(bm_tube(circle_pts(r, 40), 0.005, seg=6, closed=True), "blade",
              T(LIFT_X, 0, DECK_Z + 0.095))
    for k in range(10):
        L.add(bm_cyl(0.005, 0.26, seg=6), "blade",
              T(LIFT_X, 0, DECK_Z + 0.095) @ R('Z', 36 * k) @ T(0.16, 0, 0) @ R('Y', 90))
    L.add(bm_cyl(0.05, 0.01, seg=16), "blade", T(LIFT_X, 0, DECK_Z + 0.095))
    L.add(bm_cyl(0.06, 0.07, seg=20), "gunmetal", T(LIFT_X, 0, LIFT_Z - 0.06))   # motor
    for k in range(3):
        L.add(bm_box(0.24, 0.012, 0.03), "gunmetal",
              T(LIFT_X, 0, LIFT_Z - 0.06) @ R('Z', 120 * k + 30) @ T(0.17, 0, 0))

    # --- thrust fan ducts on pylons (static parts) --------------------------
    for s in (1, -1):
        c = Vector((0, s * THRUST_Y, THRUST_Z))
        ax = T(*c) @ R('Y', 90)                       # local Z -> world +X
        L.add(bm_ring(DUCT_R_IN, DUCT_R_OUT, DUCT_X0, DUCT_X1, seg=48), "olive", ax,
              tag="thrust_ducts")
        L.add(bm_tube(circle_pts((DUCT_R_IN + DUCT_R_OUT) / 2, 48, "YZ"), 0.014, seg=8,
                      closed=True), "gunmetal", T(DUCT_X1, c.y, c.z))
        for r in (0.19, 0.11):                        # intake guard
            L.add(bm_tube(circle_pts(r, 36, "YZ"), 0.005, seg=6, closed=True),
                  "blade", T(DUCT_X1 + 0.015, c.y, c.z))
        for k in range(8):
            L.add(bm_cyl(0.005, 0.23, seg=6), "blade",
                  T(DUCT_X1 + 0.015, c.y, c.z) @ R('X', 45 * k) @ T(0, 0.15, 0) @ R('X', 90))
        L.add(bm_cyl(0.045, 0.01, seg=16), "blade",
              T(DUCT_X1 + 0.015, c.y, c.z) @ R('Y', 90))
        # motor can + stators behind the rotor
        L.add(bm_cyl(0.06, 0.08, seg=20), "gunmetal", T(THRUST_X - 0.08, c.y, c.z) @ R('Y', 90))
        for k in range(3):
            L.add(bm_box(0.012, 0.21, 0.03), "gunmetal",
                  T(THRUST_X - 0.09, c.y, c.z) @ R('X', 120 * k + 90) @ T(0, 0.155, 0))
        # pylon from the deck up to the duct
        L.add(bm_box(0.24, 0.10, THRUST_Z - DUCT_R_OUT - DECK_Z + 0.03, bevel=0.01),
              "dark_green", T((DUCT_X0 + DUCT_X1) / 2, c.y,
                              (DECK_Z + THRUST_Z - DUCT_R_OUT + 0.03) / 2))
        # rudder hinge brackets (top & bottom)
        for z in (THRUST_Z + RUDDER_SPAN / 2 + 0.01, THRUST_Z - RUDDER_SPAN / 2 - 0.01):
            L.add(bm_box(0.06, 0.03, 0.016), "gunmetal", T(DUCT_X0 - 0.02, c.y, z))
    # cross-brace between the two ducts
    L.add(bm_box(0.10, 2 * (THRUST_Y - DUCT_R_OUT) + 0.02, 0.05, bevel=0.006), "dark_green",
          T((DUCT_X0 + DUCT_X1) / 2, 0, THRUST_Z + 0.06))

    # --- lifting handles along both sides ----------------------------------
    for sy in (1, -1):
        for hx in (-0.55, 0.0, 0.55):
            pts = [(hx - 0.09, 0.615, 0.66), (hx - 0.09, 0.665, 0.66),
                   (hx - 0.07, 0.685, 0.66), (hx + 0.07, 0.685, 0.66),
                   (hx + 0.09, 0.665, 0.66), (hx + 0.09, 0.615, 0.66)]
            L.add(bm_tube([(p[0], sy * p[1], p[2]) for p in pts], 0.012, seg=8),
                  "gunmetal")

    # --- payload cradle: rails + corner locating posts ----------------------
    for sy in (1, -1):
        L.add(bm_box(PAY_L + 0.05, 0.05, 0.02), "gunmetal",
              T(PAY_X, sy * 0.15, DECK_Z + 0.01))
        for sx in (1, -1):
            L.add(bm_box(0.03, 0.03, 0.10, bevel=0.003), "gunmetal",
                  T(PAY_X + sx * (PAY_L / 2 + 0.017), sy * (PAY_W / 2 + 0.017), DECK_Z + 0.05))
    # battery access hatches ahead of the payload
    for sy in (1, -1):
        L.add(bm_box(0.30, 0.22, 0.008, bevel=0.003), "dark_green",
              T(-0.02, sy * 0.30, DECK_Z + 0.004))

    # --- payload tie-down: two ratchet straps over the lid ------------------
    top = PAY_BASE_Z + PAY_H
    for dx in (-0.17, 0.17):
        x = PAY_X + dx
        ya, yb = PAY_W / 2 + 0.06, PAY_W / 2 + 0.012     # anchor, box side
        zt = top + 0.016
        zb = PAY_BASE_Z + 0.05
        pts = [(-ya + 0.02, DECK_Z + 0.03), (-yb - 0.003, zb), (-yb, zt - 0.05),
               (-yb, zt - 0.012), (-yb + 0.018, zt), (yb - 0.018, zt),
               (yb, zt - 0.012), (yb, zt - 0.05), (yb + 0.003, zb), (ya - 0.02, DECK_Z + 0.03)]
        L.add(bm_tube([(x, y, z) for y, z in pts], 0.009, seg=8), "bungee", tag="straps")
        for sy in (1, -1):
            L.add(bm_box(0.06, 0.05, 0.006), "gunmetal", T(x, sy * ya, DECK_Z + 0.003))
            L.add(bm_tube(circle_pts(0.018, 12, "YZ"), 0.004, seg=6, closed=True),
                  "gunmetal", T(x, sy * ya, DECK_Z + 0.022) @ R('Z', 90))
        L.add(bm_box(0.05, 0.035, 0.07, bevel=0.004), "gunmetal", T(x, yb + 0.012, zb + 0.08))

    # --- track-well sleeves on the hull bottom (retract actuators) ----------
    for sy in (1, -1):
        for sx in (1, -1):
            L.add(bm_cyl(0.035, 0.06, seg=16), "gunmetal",
                  T(sx * 0.45, sy * (TRACK_Y - 0.19), HULL_BOTTOM_Z - 0.02))

    # --- downward range sensors (cushion altimeters) under hull corners -----
    for sx in (1, -1):
        for sy in (1, -1):
            L.add(bm_cyl(0.022, 0.03, seg=12), "blade",
                  T(sx * RANGER_XY[0], sy * RANGER_XY[1], RANGER_Z - 0.015))
    return L


def build_track(side):
    """One complete track unit: rubber belt with grousers, drive sprocket
    (rear), idler (front), dual road wheels, return rollers, frame and the
    two retract rods that slide into the hull sleeves. Origin = track centre
    (the prismatic retract joint slides it straight up)."""
    s = 1 if side == "left" else -1
    y0 = s * TRACK_Y
    zc = TRACK_R                                       # track centre height, TRACK mode
    L = Link(f"track_{side}", (0, y0, zc))
    M = T(0, y0, zc)
    # TRACK_R is the grouser-tip radius (= collision radius, touches the ground)
    belt_r = TRACK_R - GROUSER_H
    L.add(bm_stadium_band(TRACK_HALF, belt_r, TRACK_T, TRACK_W, n_arc=18), "rubber", M,
          tag="belt")
    # grousers (cleats) around the outside of the belt
    for x, z, a in stadium_path(TRACK_HALF, belt_r + GROUSER_H / 2, 44):
        L.add(bm_box(0.03, TRACK_W - 0.01, GROUSER_H, bevel=0.003), "rubber",
              M @ T(x, 0, z) @ R('Y', -math.degrees(a) - 90))
    # sprocket (rear, driven) and idler (front): dual discs either side of the guide horns
    r_in = belt_r - TRACK_T
    for x, mat, teeth in ((-TRACK_HALF, "gunmetal", 12), (TRACK_HALF, "aluminium", 0)):
        for dy in (-0.075, 0.075):
            L.add(bm_cyl(r_in - 0.004, 0.07, seg=32, bevel=0.006), mat,
                  M @ T(x, dy, 0) @ R('X', 90))
        L.add(bm_cyl(0.045, TRACK_W - 0.02, seg=20), "gunmetal", M @ T(x, 0, 0) @ R('X', 90))
        for k in range(teeth):
            L.add(bm_box(0.03, 0.05, 0.022), "gunmetal",
                  M @ T(x, 0, 0) @ R('Y', 360 / teeth * k) @ T(0, 0, r_in - 0.012))
    # dual road wheels on the bottom run + return rollers on the top run
    for x in (-0.42, -0.14, 0.14, 0.42):
        for dy in (-0.075, 0.075):
            L.add(bm_cyl(0.07, 0.065, seg=24, bevel=0.006), "rubber",
                  M @ T(x, dy, -r_in + 0.07) @ R('X', 90))
            L.add(bm_cyl(0.035, 0.068, seg=16), "aluminium",
                  M @ T(x, dy, -r_in + 0.07) @ R('X', 90))
    for x in (-0.30, 0.30):
        L.add(bm_cyl(0.035, TRACK_W - 0.06, seg=16), "gunmetal",
              M @ T(x, 0, r_in - 0.035) @ R('X', 90))
    # frame: longitudinal beam between the dual wheels
    L.add(bm_box(2 * TRACK_HALF, 0.05, 0.06, bevel=0.006), "dark_green", M @ T(0, 0, 0.0),
          tag="frame")
    # retract rods: outrigger arms leave through the open inboard side of the
    # belt loop, then rise into the hull sleeves
    yr = -s * 0.19
    for x in (-0.45, 0.45):
        L.add(bm_box(0.06, 0.19, 0.05, bevel=0.005), "dark_green", M @ T(x, yr / 2, 0.0))
        L.add(bm_cyl(0.025, 0.30, seg=16), "aluminium", M @ T(x, yr, 0.15), tag="rods")
    return L


def build_lift_fan():
    L = Link("lift_fan", (LIFT_X, 0, LIFT_Z))
    M = T(LIFT_X, 0, LIFT_Z)
    L.add(bm_cyl(0.06, 0.05, seg=24, bevel=0.006), "aluminium", M)
    L.add(bm_cyl(0.04, 0.03, seg=16, r2=0.012), "aluminium", M @ T(0, 0, 0.04))
    for k in range(9):
        L.add(bm_blade(0.055, LIFT_ROTOR_R, 0.10, 0.07, 0.008, 28), "blade",
              M @ R('Z', 360 / 9 * k))
    return L


def build_thrust_fan(side):
    s = 1 if side == "left" else -1
    L = Link(f"thrust_fan_{side}", (THRUST_X, s * THRUST_Y, THRUST_Z))
    M = T(THRUST_X, s * THRUST_Y, THRUST_Z) @ R('Y', 90)   # rotor axis -> +X
    L.add(bm_cyl(0.055, 0.05, seg=24, bevel=0.006), "aluminium", M)
    L.add(bm_cyl(0.05, 0.04, seg=16, r2=0.012), "aluminium", M @ T(0, 0, 0.045))
    for k in range(7):
        L.add(bm_blade(0.05, THRUST_ROTOR_R, 0.09, 0.06, 0.008, 32), "blade",
              M @ R('Z', 360 / 7 * k))
    return L


def build_rudder(side):
    s = 1 if side == "left" else -1
    L = Link(f"rudder_{side}", (RUDDER_X, s * THRUST_Y, THRUST_Z))
    y = s * THRUST_Y
    L.add(bm_box(RUDDER_CHORD, 0.012, RUDDER_SPAN, bevel=0.004), "dark_green",
          T(RUDDER_X - RUDDER_CHORD / 2 + 0.012, y, THRUST_Z), tag="vane")
    L.add(bm_cyl(0.008, RUDDER_SPAN + 0.04, seg=8), "aluminium", T(RUDDER_X, y, THRUST_Z))
    return L


def build_payload():
    L = Link("payload_box", (PAY_X, 0, PAY_BASE_Z))
    body_h, gasket_h = 0.27, 0.01
    lid_h = PAY_H - body_h - gasket_h
    z_body = PAY_BASE_Z + body_h / 2
    z_gasket = PAY_BASE_Z + body_h + gasket_h / 2
    z_lid = PAY_BASE_Z + body_h + gasket_h + lid_h / 2
    top = PAY_BASE_Z + PAY_H
    L.add(bm_box(PAY_L, PAY_W, body_h, bevel=0.02), "orange", T(PAY_X, 0, z_body),
          tag="box_core")
    L.add(bm_box(PAY_L + 0.003, PAY_W + 0.003, gasket_h), "rubber",
          T(PAY_X, 0, z_gasket), tag="box_core")
    L.add(bm_box(PAY_L + 0.006, PAY_W + 0.006, lid_h, bevel=0.016), "orange",
          T(PAY_X, 0, z_lid), tag="box_core")
    for dy in (-0.11, 0.11):              # lid reinforcing ribs
        L.add(bm_box(PAY_L - 0.10, 0.02, 0.01, bevel=0.003), "orange",
              T(PAY_X, dy, top + 0.003))
    hx = 0.12                             # carry handle
    pts = [(-hx, 0, top), (-hx, 0, top + 0.03), (-hx + 0.02, 0, top + 0.045),
           (hx - 0.02, 0, top + 0.045), (hx, 0, top + 0.03), (hx, 0, top)]
    L.add(bm_tube([(PAY_X + p[0], p[1], p[2]) for p in pts], 0.01, seg=8), "rubber")
    zh = PAY_BASE_Z + body_h + gasket_h / 2
    for dy in (-0.13, 0.13):              # hinges on the rear edge
        L.add(bm_cyl(0.013, 0.08, seg=12), "gunmetal",
              T(PAY_X - PAY_L / 2 - 0.01, dy, zh) @ R('X', 90))
    for dy in (-0.13, 0.13):              # draw latches: two front, one each side
        L.add(bm_box(0.02, 0.06, 0.07, bevel=0.003), "gunmetal",
              T(PAY_X + PAY_L / 2 + 0.01, dy, zh))
    for sy in (1, -1):
        L.add(bm_box(0.06, 0.02, 0.07, bevel=0.003), "gunmetal",
              T(PAY_X + 0.20, sy * (PAY_W / 2 + 0.01), zh))
        L.add(bm_box(0.12, 0.004, 0.08), "label",
              T(PAY_X - 0.05, sy * (PAY_W / 2 + 0.002), PAY_BASE_Z + 0.14))
    L.add(bm_cyl(0.02, 0.016, seg=12), "gunmetal", T(PAY_X + 0.24, 0.15, top + 0.006))
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
    L.add(bm_box(0.075, 0.11, 0.075, bevel=0.008), "blade", T(CAM_X - 0.055, 0, CAM_Z))
    L.add(bm_cyl(0.024, 0.02, seg=20), "gunmetal", T(CAM_X - 0.01, 0, CAM_Z) @ R('Y', 90))
    L.add(bm_cyl(0.018, 0.003, seg=20), "lens", T(CAM_X - 0.0015, 0, CAM_Z) @ R('Y', 90))
    L.add(bm_box(0.05, 0.125, 0.006), "blade", T(CAM_X - 0.045, 0, CAM_Z + 0.043))
    return L


# ==========================================================================
# 6. JOINT / FRAME TABLE  (drives link_frames.json -> model.sdf)
# ==========================================================================
def joint_table():
    J = {}
    J["skirt"] = ("hull", "fixed", None, None)
    J["lift_fan"] = ("hull", "continuous", (0, 0, 1), None)
    for side in ("left", "right"):
        J[f"thrust_fan_{side}"] = ("hull", "continuous", (1, 0, 0), None)
        J[f"rudder_{side}"] = ("hull", "revolute", (0, 0, 1),
                               (-math.radians(30), math.radians(30)))
        # Retract: the whole track unit slides straight up into its hull well.
        # The limits leave TRACK_LIMIT_MARGIN either side of 0 and TRACK_STROKE:
        # a velocity-controlled DART joint resting on a limit can stick, so the
        # controller holds 0 / TRACK_STROKE without touching the stops.
        J[f"track_{side}"] = ("hull", "prismatic", (0, 0, 1),
                              (-TRACK_LIMIT_MARGIN, TRACK_STROKE + TRACK_LIMIT_MARGIN),
                              {"control": "position",
                               "note": f"0 = deployed (TRACK mode); {TRACK_STROKE} = "
                                       "retracted into the hull (HOVER mode)"})
    J["payload_box"] = ("hull", "fixed", None, None)
    J["lidar"] = ("hull", "fixed", None, None)
    J["camera"] = ("hull", "fixed", None, None)
    return {k: (v + (None,))[:5] for k, v in J.items()}


def collision_table():
    """Simple SDF primitives in each link's own frame (do NOT collide with the
    visual meshes). A link may list several primitives."""
    h = math.pi / 2
    C = {
        "hull": {"type": "box", "size": [2.30, 1.20, DECK_Z - HULL_BOTTOM_Z - 0.02],
                 "pose": [0, 0, (DECK_Z + HULL_BOTTOM_Z) / 2 - 0.50, 0, 0, 0]},
        # Collision covers the bag only, not the bottom FINGER_FLEX of the
        # flexible fingers: low debris brushes the fingers and passes under,
        # like a real segmented skirt; taller obstacles hit the bag.
        "skirt": {"type": "box",
                  "size": [SKIRT_L, SKIRT_W, SKIRT_TOP_Z - SKIRT_BOTTOM_Z - FINGER_FLEX],
                  "pose": [0, 0, (SKIRT_TOP_Z + SKIRT_BOTTOM_Z + FINGER_FLEX) / 2 - BAG_Z,
                           0, 0, 0]},
        "lift_fan": {"type": "cylinder", "radius": LIFT_ROTOR_R, "length": 0.04,
                     "pose": [0, 0, 0, 0, 0, 0]},
        "payload_box": {"type": "box", "size": [PAY_L, PAY_W, PAY_H],
                        "pose": [0, 0, PAY_H / 2, 0, 0, 0]},
        "lidar": {"type": "cylinder", "radius": LIDAR_R, "length": LIDAR_H,
                  "pose": [0, 0, 0, 0, 0, 0]},
        "camera": {"type": "box", "size": [0.075, 0.11, 0.075],
                   "pose": [-0.055, 0, 0, 0, 0, 0]},
    }
    for side in ("left", "right"):
        C[f"thrust_fan_{side}"] = {"type": "cylinder", "radius": THRUST_ROTOR_R,
                                   "length": 0.04, "pose": [0, 0, 0, 0, h, 0]}
        C[f"rudder_{side}"] = {"type": "box", "size": [RUDDER_CHORD, 0.012, RUDDER_SPAN],
                               "pose": [-RUDDER_CHORD / 2 + 0.012, 0, 0, 0, 0, 0]}
        # Track: a box for the straight runs plus a cylinder at each end, so
        # the track rolls onto kerbs and slopes instead of catching an edge.
        C[f"track_{side}"] = [
            {"type": "box", "size": [2 * TRACK_HALF, TRACK_W, 2 * TRACK_R],
             "pose": [0, 0, 0, 0, 0, 0]},
            {"type": "cylinder", "radius": TRACK_R, "length": TRACK_W,
             "pose": [TRACK_HALF, 0, 0, h, 0, 0]},
            {"type": "cylinder", "radius": TRACK_R, "length": TRACK_W,
             "pose": [-TRACK_HALF, 0, 0, h, 0, 0]},
        ]
    return C


def sensor_table():
    S = {
        "imu": {"parent": "hull", "xyz": [0.0, 0.0, 0.50], "rpy": [0, 0, 0], "type": "imu"},
        "navsat": {"parent": "hull", "xyz": [0.93, 0.25, BAY_TOP_Z + 0.03],
                   "rpy": [0, 0, 0], "type": "navsat"},
        "lidar_3d": {"parent": "lidar", "xyz": [LIDAR_X, 0, LIDAR_Z], "rpy": [0, 0, 0],
                     "type": "gpu_lidar", "channels": 16, "hfov_deg": 360,
                     "vfov_deg": [-15, 15], "range_m": [0.3, 30.0], "rate_hz": 10},
        "front_camera": {"parent": "camera", "xyz": [CAM_X, 0, CAM_Z], "rpy": [0, 0, 0],
                         "type": "camera"},
    }
    for c, (sx, sy) in {"fl": (1, 1), "fr": (1, -1), "rl": (-1, 1), "rr": (-1, -1)}.items():
        S[f"ranger_{c}"] = {"parent": "hull",
                            "xyz": [sx * RANGER_XY[0], sy * RANGER_XY[1], RANGER_Z],
                            "rpy": [0, math.pi / 2, 0],   # +X of sensor points down
                            "type": "gpu_lidar (narrow, downward)"}
    return S


# ==========================================================================
# 7. ASSEMBLY, ANIMATION, EXPORT, RENDERS
# ==========================================================================
def build_all():
    make_materials()
    coll = bpy.data.collections.new(MODEL_NAME)
    bpy.context.scene.collection.children.link(coll)
    builders = [build_hull, build_skirt, build_lift_fan,
                lambda: build_thrust_fan("left"), lambda: build_thrust_fan("right"),
                lambda: build_rudder("left"), lambda: build_rudder("right"),
                lambda: build_track("left"), lambda: build_track("right"),
                build_payload, build_lidar, build_camera]

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
    root = bpy.data.objects.new(f"{MODEL_NAME}_root", None)
    root.empty_display_type = 'ARROWS'
    root.empty_display_size = 0.5
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
    """TRACK -> HOVER, cosmetic only (handy for pitch videos).
    Frames 1-30 TRACK mode on the tracks; 30-50 lift fan spin-up; 50-60 the
    cushion takes the weight and lifts the vehicle HOVER_RISE; 60-90 the
    tracks retract into the hull; 90-140 the thrust fans run."""
    scene = bpy.context.scene
    scene.frame_start, scene.frame_end = 1, 140
    scene.render.fps = 24

    def key(ob, path, frame, value, idx):
        getattr(ob, path)[idx] = value
        ob.keyframe_insert(path, index=idx, frame=frame)

    for side in ("left", "right"):
        tr = links[f"track_{side}"]
        z0 = tr.location.z
        key(tr, "location", 60, z0, 2)
        key(tr, "location", 90, z0 + TRACK_STROKE, 2)
        tr.location.z = z0
    key(root, "location", 50, 0.0, 2)
    key(root, "location", 60, HOVER_RISE, 2)
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
    bmesh.ops.create_grid(pb, x_segments=1, y_segments=1, size=10.0)
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
    chk.inputs["Scale"].default_value = 4.0           # object coords -> 0.25 m squares
    nt.links.new(chk.outputs["Color"], bsdf.inputs["Base Color"])
    me.materials.append(m)

    sun_d = bpy.data.lights.new("sun", 'SUN')
    sun_d.energy = 3.5
    sun_d.angle = math.radians(8)
    sun = bpy.data.objects.new("sun", sun_d)
    sun.rotation_euler = (math.radians(40), math.radians(-10), math.radians(-35))
    stage.objects.link(sun)
    fill_d = bpy.data.lights.new("fill", 'AREA')
    fill_d.energy = 900
    fill_d.size = 5
    fill = bpy.data.objects.new("fill", fill_d)
    fill.location = (-4.0, 5.0, 4.0)
    fill.rotation_euler = (Vector((0, 0, 0.6)) - fill.location).to_track_quat('-Z', 'Y').to_euler()
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


def cushion_pressure():
    """Cushion pressure (Pa) that carries the whole vehicle on the rounded footprint."""
    area = SKIRT_L * SKIRT_W - (4 - math.pi) * (BAG_CORNER_R + BAG_R) ** 2
    return sum(MASS.values()) * 9.81 / area


def puff_port_force():
    """Side force (N) of one fully open puff port: jet momentum 2·Cd·p·A."""
    return 2 * PUFF_CD * cushion_pressure() * PUFF_W * PUFF_H


def vehicle_com(links):
    """Whole-vehicle centre of mass (TRACK mode): each link at its bbox centre,
    except the hull, whose battery-heavy mass sits at HULL_COM_Z."""
    m_tot, acc = 0.0, Vector((0, 0, 0))
    for name, ob in links.items():
        lo, hi = eu.world_bbox([ob])
        c = (lo + hi) / 2
        if name == "hull":
            c = Vector((c.x, c.y, HULL_COM_Z))
        acc += MASS[name] * c
        m_tot += MASS[name]
    return acc / m_tot


def measure(links, tags):
    """Bounding boxes + AGENTS.md Version 2 checks. Returns (log lines, all_pass)."""
    lines, ok_all = [], True
    obs = list(links.values())
    lines.append("=== Per-link world bounding boxes (TRACK mode, metres) ===")
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
        lines.append(f"[{'PASS' if ok else 'FAIL'}] {label:46s} {value:8.3f} {unit}"
                     f"  (spec {target} ± {tol})")

    def check_max(label, value, limit, unit="m"):
        nonlocal ok_all
        ok = value <= limit
        ok_all &= ok
        lines.append(f"[{'PASS' if ok else 'FAIL'}] {label:46s} {value:8.3f} {unit}"
                     f"  (spec <= {limit})")

    def check_min(label, value, limit, unit="m"):
        nonlocal ok_all
        ok = value >= limit
        ok_all &= ok
        lines.append(f"[{'PASS' if ok else 'FAIL'}] {label:46s} {value:8.3f} {unit}"
                     f"  (spec >= {limit})")

    lines.append("\n=== AGENTS.md Version 2 checks ===")
    slo, shi = eu.world_bbox([links["skirt"]])
    check("Skirt footprint length", shi.x - slo.x, SKIRT_L, 0.01)
    check("Skirt footprint width", shi.y - slo.y, SKIRT_W, 0.01)
    check_max("Overall length (incl. rudders)", hi.x - lo.x, SKIRT_L + 0.03)
    check_max("Overall width (tracks inboard)", hi.y - lo.y, SKIRT_W + 0.005)
    check_max("Hull top (bow bay), TRACK mode", tags["hull"]["bow_bay"][1].z, 1.00)
    lid = links["lidar"].matrix_world.translation
    check("LiDAR on the centreline above base_link (|x|+|y|)", abs(lid.x) + abs(lid.y), 0.0, 0.001)
    tmin = min(eu.world_bbox([links[f"track_{s}"]])[0].z for s in ("left", "right"))
    check("Tracks touch ground in TRACK mode (min z)", tmin, 0.0, 0.004)
    check("Skirt bottom in TRACK mode", slo.z, SKIRT_BOTTOM_Z, 0.005)
    blo, bhi = tags["track_left"]["belt"]
    check("Track belt width", bhi.y - blo.y, TRACK_W, 0.002)
    check("Track contact length (sprocket-idler centres)", 2 * TRACK_HALF, 1.40, 0.0)
    gauge = 2 * TRACK_Y
    check("Track gauge (centre to centre)", gauge, 0.84, 0.0)
    check("Length-to-gauge ratio (skid steer: 1.0-1.8)", 2 * TRACK_HALF / gauge, 1.4, 0.4, "")
    bag_inner = SKIRT_W / 2 - 2 * BAG_R
    check_max("Track outer edge inside the skirt bag", TRACK_Y + TRACK_W / 2, bag_inner)
    pplo, pphi = tags["hull"]["puff_ports"]
    check_max("Puff ports inside the skirt footprint (|y|)", max(-pplo.y, pphi.y), SKIRT_W / 2)
    plo, phi = tags["payload_box"]["box_core"]
    check("Payload box length (core)", phi.x - plo.x, PAY_L, 0.01)
    check("Payload box width (core)", phi.y - plo.y, PAY_W, 0.01)
    check("Payload box height (core)", phi.z - plo.z, PAY_H, 0.01)
    mass = sum(MASS.values())
    check("Total mass (incl. 30 kg payload)", mass, 300.0, 0.05, "kg")
    com = vehicle_com(links)
    check_max("Centre-of-mass height, TRACK mode", com.z, 0.50)
    lines.append(f"       (CoM x {com.x:+.3f} m before the generator balances the hull; "
                 f"CoM height / beam = {com.z / SKIRT_W:.2f})")
    check_max("Triangle budget", total_tris, 150000, "tri")

    # HOVER pose (frame 100): lifted HOVER_RISE, tracks retracted
    scene = bpy.context.scene
    scene.frame_set(100)
    hs_lo, _ = eu.world_bbox([links["skirt"]])
    t_lo = min(eu.world_bbox([links[f"track_{s}"]])[0].z for s in ("left", "right"))
    _, all_hi = eu.world_bbox(list(links.values()))
    scene.frame_set(1)
    lines.append("\n=== HOVER mode (frame 100) ===")
    check("Cushion gap under the skirt", hs_lo.z, HOVER_GAP, 0.003)
    check_min("Retracted tracks above the skirt bottom", t_lo - hs_lo.z, 0.15)
    check("Overall height incl. LiDAR", all_hi.z, OVERALL_H, 0.02)

    area = SKIRT_L * SKIRT_W - (4 - math.pi) * (BAG_CORNER_R + BAG_R) ** 2
    W = mass * 9.81
    contact = 2 * TRACK_W * 2 * TRACK_HALF
    lines.append("\n=== Derived (stated estimates, not validated) ===")
    lines.append(f"Cushion area (rounded footprint)   {area:.2f} m^2")
    lines.append(f"Cushion pressure  {mass:.0f} kg * 9.81 / A = {W / area:.0f} Pa")
    lines.append(f"Track ground pressure, no cushion support  {W / contact:.0f} Pa "
                 f"({contact:.3f} m^2 contact)")
    lines.append(f"Track ground pressure at 60 % cushion load share  {0.4 * W / contact:.0f} Pa")
    lines.append(f"Retract stroke {TRACK_STROKE} m; HOVER rise {HOVER_RISE:.3f} m")
    fp = puff_port_force()
    flow = PUFF_CD * PUFF_W * PUFF_H * math.sqrt(2 * W / area / 1.2)
    lines.append(f"Puff port: {PUFF_W} x {PUFF_H} m outlet, Cd {PUFF_CD}: side force "
                 f"2*Cd*p*A = {fp:.0f} N, air flow {flow:.2f} m^3/s each")
    lines.append(f"Puff-port yaw couple (bow + opposite stern vent) {2 * fp * PUFF_X:.0f} N m; "
                 f"rudders {RUDDER_MAX_DEG} deg max")
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
        "generated_by": "assets/vehicle_blender/version_2/build_vehicle.py",
        "blender_version": bpy.app.version_string,
        "model_name": MODEL_NAME,
        "units": "metres, kilograms, radians",
        "axes": "REP-103: +X forward, +Y left, +Z up",
        "model_frame": "ground plane under hull centre, vehicle standing on its "
                       "deployed tracks (TRACK mode: all joints at position 0)",
        "note": "origin_xyz = link origin (joint pivot) in the model frame. "
                "origin_in_parent = pose to use for the SDF joint/link "
                "(no rotation anywhere: all link frames are axis-aligned). "
                "collision may be one primitive or a list.",
        "total_mass_kg": round(sum(MASS.values()), 3),
        "hull_com_z": HULL_COM_Z,
        "cushion": {"skirt_bottom_track_mode": SKIRT_BOTTOM_Z, "hover_gap": HOVER_GAP,
                    "corner_xy": [round(SKIRT_L / 2 - BAG_CORNER_R / 2, 3),
                                  round(SKIRT_W / 2 - BAG_CORNER_R / 2, 3)]},
        "tracks": {"half_length": TRACK_HALF, "radius": TRACK_R, "width": TRACK_W,
                   "gauge": 2 * TRACK_Y, "stroke": TRACK_STROKE},
        "rudders": {"max_angle_rad": round(math.radians(RUDDER_MAX_DEG), 4)},
        "puff_ports": {"x": PUFF_X, "y": round(tags["hull"]["puff_ports"][1].y, 4),
                       "z": PUFF_Z, "width": PUFF_W, "height": PUFF_H,
                       "discharge_coefficient": PUFF_CD,
                       "cushion_pressure_pa": round(cushion_pressure(), 1),
                       "force_n": round(puff_port_force(), 1)},
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
        "front_34": eu.add_camera("cam_front_34", (4.4, -3.3, 2.1), (0.05, 0.0, 0.55), lens=50),
        "side": eu.add_camera("cam_side", (0.0, -8.0, 0.78), (0.0, 0.0, 0.78), ortho_scale=3.0),
        "top": eu.add_camera("cam_top", (0.0, 0.0, 6.0), (0.0, 0.0, 0.0), ortho_scale=2.9),
        "rear_34": eu.add_camera("cam_rear_34", (-4.2, 3.2, 2.0), (-0.1, 0.0, 0.6), lens=50),
        "tracks_cutaway": eu.add_camera("cam_cutaway", (3.0, -2.6, 0.75), (0.0, 0.0, 0.25),
                                        lens=35),
    }
    cams["top"].rotation_euler = (0, 0, 0)       # +X right, +Y up in image
    bpy.context.scene.camera = cams["front_34"]
    return cams


def render_previews(cams, renders_dir, links):
    eu.setup_render("CYCLES", res=(1280, 800), samples=32)
    for name in ("front_34", "side", "top", "rear_34"):
        bpy.context.scene.frame_set(1)
        eu.render_to(cams[name], renders_dir / f"{name}.png")
    # TRACK mode with the skirt hidden: shows the inboard tracks under the hull
    links["skirt"].hide_render = True
    eu.render_to(cams["tracks_cutaway"], renders_dir / "tracks_cutaway.png")
    bpy.context.scene.frame_set(100)             # HOVER mode, tracks retracted
    eu.render_to(cams["tracks_cutaway"], renders_dir / "hover_cutaway.png")
    links["skirt"].hide_render = False
    eu.render_to(cams["front_34"], renders_dir / "hover_mode.png")
    bpy.context.scene.frame_set(1)
    bpy.context.scene.camera = cams["front_34"]


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]
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
    lines.append(f"\nMeshes + link_frames.json written to {model_dir.relative_to(repo).as_posix()}")
    if not eu.has_collada():
        lines.append("NOTE: this Blender has no COLLADA exporter (removed in 5.0) - "
                     "only .glb meshes were written; use those in the SDF.")
    lines.append(f"OVERALL: {'ALL CHECKS PASS' if ok else 'SOME CHECKS FAILED'}")
    renders_dir.mkdir(parents=True, exist_ok=True)
    (renders_dir / "dimensions.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))

    build_stage()
    cams = add_preview_cameras()
    if do_render:
        render_previews(cams, renders_dir, links)
    bpy.ops.wm.save_as_mainfile(filepath=str(_HERE / f"{MODEL_NAME}.blend"))
    print("Saved", _HERE / f"{MODEL_NAME}.blend")
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
