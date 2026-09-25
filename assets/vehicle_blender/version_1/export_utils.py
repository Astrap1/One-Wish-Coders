"""
export_utils.py — shared helpers for the hover_recon Blender scripts.

Used by build_vehicle.py (and later build_world.py). Everything here is
plain bpy so it works both under `blender --background --python ...` and
the `bpy` Python module.

Conventions (REP-103): metres, +X forward, +Y left, +Z up.
"""
import json
import math
from pathlib import Path

import bpy
from mathutils import Vector


# --------------------------------------------------------------------------
# Scene housekeeping
# --------------------------------------------------------------------------
def clear_scene():
    """Remove every object, mesh, material, collection, camera, light and
    world so that re-running a build script is idempotent.

    We deliberately do NOT use read_factory_settings(): that would also reset
    the UI/preferences when the script is run inside a live Blender session.
    """
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)
    for coll in list(bpy.data.collections):
        bpy.data.collections.remove(coll)
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.cameras,
                  bpy.data.lights, bpy.data.curves, bpy.data.images,
                  bpy.data.node_groups, bpy.data.actions):
        for item in list(block):
            block.remove(item)
    for w in list(bpy.data.worlds):
        bpy.data.worlds.remove(w)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 1.0
    scene.frame_set(1)


def apply_transforms(ob):
    """Bake location/rotation/scale into the mesh except the location, which
    we keep as the link origin (= joint pivot). After this, rotation = 0 and
    scale = 1 — the exporter then writes geometry in the link frame."""
    if ob.type != 'MESH':
        return
    rs = ob.matrix_basis.to_3x3().to_4x4()          # rotation * scale only
    ob.data.transform(rs)
    ob.rotation_mode = 'XYZ'
    ob.rotation_euler = (0.0, 0.0, 0.0)
    ob.scale = (1.0, 1.0, 1.0)


# --------------------------------------------------------------------------
# Measurement
# --------------------------------------------------------------------------
def world_bbox(obs):
    """Axis-aligned bounding box (min, max) of the given objects in world
    coordinates."""
    lo = Vector((math.inf,) * 3)
    hi = Vector((-math.inf,) * 3)
    for ob in obs:
        if ob.type != 'MESH':
            continue
        mw = ob.matrix_world
        for v in ob.data.vertices:
            p = mw @ v.co
            lo = Vector(map(min, lo, p))
            hi = Vector(map(max, hi, p))
    return lo, hi


def tri_count(ob):
    return sum(len(p.vertices) - 2 for p in ob.data.polygons)


def fmt_v(v, nd=3):
    return "(" + ", ".join(f"{c:+.{nd}f}" for c in v) + ")"


# --------------------------------------------------------------------------
# Export
# --------------------------------------------------------------------------
def _isolate_copy(ob):
    """Make an unparented copy of `ob` at the world origin (identity
    transform) sharing the same mesh. Exporting that copy writes the mesh in
    the link's own frame, which is exactly what an SDF <visual> expects."""
    tmp = bpy.data.objects.new(ob.name, ob.data)       # same name in file
    bpy.context.scene.collection.objects.link(tmp)
    return tmp


def has_collada():
    """COLLADA import/export was removed in Blender 5.0."""
    try:
        bpy.ops.wm.collada_export.get_rna_type()
        return True
    except (AttributeError, KeyError, RuntimeError):
        return False


def iter_fcurves(action):
    """F-curves of an action on Blender 4.x (action.fcurves) and on 5.x
    (layered actions: layers -> strips -> channelbags)."""
    if hasattr(action, "fcurves"):
        return list(action.fcurves)
    out = []
    for layer in action.layers:
        for strip in layer.strips:
            for bag in strip.channelbags:
                out.extend(bag.fcurves)
    return out


def ensure_nodes(idblock):
    """`use_nodes` is deprecated in Blender 5.x (always on); set it only
    where it still matters."""
    try:
        if not idblock.use_nodes:
            idblock.use_nodes = True
    except AttributeError:
        pass
    return idblock.node_tree


def export_link(ob, mesh_dir, formats=("dae", "glb")):
    """Export one link object to mesh_dir/<name>.<ext>. Returns written paths."""
    mesh_dir = Path(mesh_dir)
    mesh_dir.mkdir(parents=True, exist_ok=True)
    orig_name = ob.name
    ob.name = orig_name + "__src"          # free the name for the temp copy
    tmp = _isolate_copy(ob)
    tmp.name = orig_name

    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    tmp.select_set(True)
    bpy.context.view_layer.objects.active = tmp

    written = []
    try:
        if "dae" in formats and has_collada():
            path = mesh_dir / f"{orig_name}.dae"
            bpy.ops.wm.collada_export(
                filepath=str(path), selected=True, apply_modifiers=True,
                triangulate=True, include_children=False,
                export_global_forward_selection='Y',   # keep Blender axes:
                export_global_up_selection='Z',        # Z-up, like Gazebo
                apply_global_orientation=False)
            written.append(path)
        if "glb" in formats:
            path = mesh_dir / f"{orig_name}.glb"
            bpy.ops.export_scene.gltf(
                filepath=str(path), export_format='GLB', use_selection=True,
                export_apply=True, export_yup=True,    # glTF spec is Y-up;
                export_materials='EXPORT',
                export_animations=False)             # loaders convert back
            written.append(path)
    finally:
        bpy.data.objects.remove(tmp, do_unlink=True)
        ob.name = orig_name
    return written


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


# --------------------------------------------------------------------------
# Preview renders
# --------------------------------------------------------------------------
def setup_render(engine="CYCLES", res=(1280, 800), samples=48):
    scene = bpy.context.scene
    if engine == "CYCLES":
        try:
            import addon_utils
            addon_utils.enable("cycles", default_set=True)
        except Exception:
            pass
    scene.render.engine = engine
    if engine == "CYCLES":
        scene.cycles.device = 'CPU'
        scene.cycles.samples = samples
        scene.cycles.use_denoising = True
        scene.cycles.max_bounces = 6
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = False
    scene.render.image_settings.file_format = 'PNG'
    try:
        scene.view_settings.view_transform = 'AgX'
        scene.view_settings.look = 'AgX - Base Contrast'
    except TypeError:
        pass


def add_camera(name, location, target, ortho_scale=None, lens=50):
    cam_data = bpy.data.cameras.new(name)
    cam = bpy.data.objects.new(name, cam_data)
    bpy.context.scene.collection.objects.link(cam)
    cam.location = location
    direction = Vector(target) - Vector(location)
    cam.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()
    if ortho_scale:
        cam_data.type = 'ORTHO'
        cam_data.ortho_scale = ortho_scale
    else:
        cam_data.lens = lens
    cam_data.clip_start = 0.01
    return cam


def render_to(cam, path):
    scene = bpy.context.scene
    scene.camera = cam
    scene.render.filepath = str(path)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.render.render(write_still=True)
