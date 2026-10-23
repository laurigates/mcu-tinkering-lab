"""Render an assembly scene from enclosure.assembly with Blender, headless.

    Blender -b -P render_assembly.py -- <piece>-assembly.json

Writes <piece>-tray-assembly.png (board seated in the open tray) and
<piece>-box-assembly.png (board in the box base, lid lifted above it) beside
the JSON. Runs inside Blender's own Python: bpy only, no build123d.
"""

import json
import math
import sys
from pathlib import Path

import bpy
import mathutils


def clear():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def material(name, rgba, rough=0.5, alpha=1.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = rgba
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Alpha"].default_value = alpha
    return m


def import_stl(path, mat, z_lift=0.0):
    bpy.ops.wm.stl_import(filepath=path)
    obj = bpy.context.selected_objects[0]
    obj.location.z += z_lift
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    return obj


def import_board(scene):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=scene["board_glb"])
    new = [o for o in bpy.data.objects if o not in before]
    root = bpy.data.objects.new("board", None)
    bpy.context.scene.collection.objects.link(root)
    for o in new:
        if o.parent is None:
            o.parent = root
    s = scene["board_scale"]
    root.scale = (s, s, s)
    root.location = scene["board_offset"]
    bpy.context.view_layer.update()
    # Keep only this piece of the panel.
    x, y, z, w, h, d = scene["region_box"]
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x + w / 2, y + h / 2, z + d / 2))
    cutter = bpy.context.active_object
    cutter.scale = (w, h, d)
    cutter.hide_render = True
    cutter.display_type = "WIRE"
    for o in new:
        if o.type != "MESH":
            continue
        centre = (
            sum(
                (o.matrix_world @ mathutils.Vector(c) for c in o.bound_box),
                mathutils.Vector(),
            )
            / 8
        )
        inside = x - 1 <= centre.x <= x + w + 1 and y - 1 <= centre.y <= y + h + 1
        big = max(o.dimensions) * s > 0.5 * min(w, h)
        if not inside and not big:
            o.hide_render = True
            continue
        mod = o.modifiers.new("piece", "BOOLEAN")
        mod.operation = "INTERSECT"
        mod.solver = "FLOAT"  # Blender 5: FLOAT, EXACT or MANIFOLD
        mod.object = cutter
    return root


def camera_and_light(size, z_extra=0.0):
    lx, ly, lz = size
    lz += z_extra
    centre = mathutils.Vector((lx / 2, ly / 2, lz / 2))
    span = max(lx, ly, lz)
    cam_data = bpy.data.cameras.new("cam")
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = span * 1.55
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.scene.collection.objects.link(cam)
    direction = mathutils.Vector((-1.0, -1.35, 1.05)).normalized()
    cam.location = centre + direction * span * 4
    cam.rotation_euler = (centre - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.0
    sun.rotation_euler = (math.radians(40), math.radians(-20), math.radians(-35))
    bpy.context.scene.collection.objects.link(sun)
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (
        0.92,
        0.93,
        0.95,
        1,
    )
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.9
    bpy.context.scene.world = world


def render(out):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = 48
    sc.render.resolution_x, sc.render.resolution_y = 1400, 1000
    sc.render.film_transparent = False
    sc.render.filepath = str(out)
    bpy.ops.render.render(write_still=True)


def setup():
    """A fresh scene and its materials (a factory reset removes old ones)."""
    clear()
    return (
        material("resin", (0.86, 0.86, 0.84, 1), rough=0.45),
        material("module stand-in", (0.20, 0.42, 0.85, 1), rough=0.6),
    )


def main(path):
    scene = json.loads(Path(path).read_text())
    stem = Path(path).with_name(scene["piece"])

    resin, stand = setup()
    import_stl(scene["tray"], resin)
    import_board(scene)
    if scene["stand_ins"]:
        import_stl(scene["stand_ins"], stand)
    camera_and_light(scene["size"])
    render(f"{stem}-tray-assembly.png")

    resin, stand = setup()
    import_stl(scene["box_base"], resin)
    import_board(scene)
    if scene["stand_ins"]:
        import_stl(scene["stand_ins"], stand)
    lift = 0.6 * scene["size"][2]
    import_stl(scene["box_lid"], resin, z_lift=lift)
    camera_and_light(scene["size"], z_extra=lift)
    render(f"{stem}-box-assembly.png")


if __name__ == "__main__":
    main(sys.argv[sys.argv.index("--") + 1])
