"""Product renders of a carrier, its breakouts and its cases, headless.

    Blender -b -P render_product.py -- <project>-product.json

For each view in the JSON (enclosure.product writes it), renders
<out>/<id>.png: RGBA, transparent background, a shadow-catcher floor, studio
lighting and a perspective camera fitted to the subject. The transparent
render is the source for the plate an image-edit model gets and for the
captioned copy; enclosure.product makes both.

The breakouts are look-alikes from modules.toml (sockets, header pins, the
module PCB and its main parts) placed by enclosure.modules: KiCad has no 3D
model for them. Reuses render_assembly's board and STL import.
"""

import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
import mathutils
from bpy_extras.object_utils import world_to_camera_view

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_assembly import import_board, import_stl, material

SOCKET_H = 8.5  # female header body
SPACER_H = 2.5  # male header plastic
PITCH = 2.54


def materials():
    """Made after each factory reset, which deletes earlier ones."""

    def metal(name, rgb, rough):
        m = material(name, (*rgb, 1), rough=rough)
        m.node_tree.nodes["Principled BSDF"].inputs["Metallic"].default_value = 1.0
        return m

    resin = material("resin", (0.88, 0.88, 0.86, 1), rough=0.55)
    bsdf = resin.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Subsurface Weight"].default_value = 0.08
    bsdf.inputs["Subsurface Radius"].default_value = (1.0, 0.9, 0.8)
    return {
        "resin": resin,
        "plastic": material("header plastic", (0.02, 0.02, 0.02, 1), rough=0.45),
        "ic": material("ic", (0.03, 0.03, 0.035, 1), rough=0.35),
        "gold": metal("gold", (1.0, 0.77, 0.34), 0.25),
        "tin": metal("tin", (0.8, 0.8, 0.82), 0.3),
        "alu": metal("aluminium", (0.85, 0.86, 0.88), 0.35),
        "glass": material("lens", (0.02, 0.03, 0.08, 1), rough=0.05),
        "ceramic": material("trimpot", (0.9, 0.9, 0.86, 1), rough=0.5),
        "terminal": material("terminal", (0.05, 0.25, 0.75, 1), rough=0.4),
    }


class Mesh:
    """Boxes and cylinders gathered per material into one object each."""

    def __init__(self, parent):
        self.parent = parent
        self.parts = {}

    def _bm(self, mat):
        return self.parts.setdefault(mat, bmesh.new())

    def box(self, mat, x0, y0, x1, y1, z0, z1):
        """Footprint-local mm, KiCad's y down: flipped to the scene's y up."""
        bm = self._bm(mat)
        m = mathutils.Matrix.Translation(
            ((x0 + x1) / 2, -(y0 + y1) / 2, (z0 + z1) / 2)
        ) @ mathutils.Matrix.Diagonal((abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), 1))
        bmesh.ops.create_cube(bm, size=1.0, matrix=m)

    def cyl(self, mat, x, y, r, z0, z1, segments=16):
        bm = self._bm(mat)
        m = mathutils.Matrix.Translation((x, -y, (z0 + z1) / 2))
        bmesh.ops.create_cone(
            bm, cap_ends=True, segments=segments, radius1=r, radius2=r,
            depth=z1 - z0, matrix=m,
        )  # fmt: skip

    def finish(self, name):
        for mat, bm in self.parts.items():
            me = bpy.data.meshes.new(f"{name}-{mat.name}")
            bm.to_mesh(me)
            bm.free()
            obj = bpy.data.objects.new(me.name, me)
            obj.data.materials.append(mat)
            obj.parent = self.parent
            bpy.context.scene.collection.objects.link(obj)


def feature(mesh, mats, pcb_mat, f, outline, z):
    x0, y0, x1, y1 = outline
    if "xy" in f:
        cx, cy = f["xy"]
    else:
        cx, cy = x0 + f["at"][0] * (x1 - x0), y0 + f["at"][1] * (y1 - y0)
    w, h, d = f["size"]
    z += f.get("z", 0.0)
    bx = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
    what = f["what"]
    if what == "chip" or what == "connector":
        mesh.box(mats["ic"], *bx, z, z + d)
    elif what == "board":
        mesh.box(pcb_mat, *bx, z, z + d)
    elif what in ("usb-c", "shield"):
        mesh.box(mats["tin"], *bx, z, z + d)
    elif what == "heatsink":
        base = 2.0
        mesh.box(mats["alu"], *bx, z, z + base)
        fins = 5
        for i in range(fins):
            fx = bx[0] + (i + 0.5) * w / fins
            mesh.box(mats["alu"], fx - 0.45, bx[1], fx + 0.45, bx[3], z + base, z + d)
    elif what == "trimpot":
        mesh.box(mats["ceramic"], *bx, z, z + d * 0.6)
        mesh.cyl(mats["tin"], cx, cy, min(w, h) * 0.35, z + d * 0.6, z + d)
    elif what == "camera":
        mesh.box(mats["ic"], *bx, z, z + 2.0)
        r = min(w, h) * 0.38
        mesh.cyl(mats["ic"], cx, cy, r, z + 2.0, z + d - 0.2, 32)
        mesh.cyl(mats["glass"], cx, cy, r * 0.6, z + d - 0.2, z + d, 32)
    elif what == "terminal":
        body = d * 0.8
        mesh.box(mats["terminal"], *bx, z, z + body)
        for i in (0.25, 0.75):
            mesh.cyl(mats["tin"], bx[0] + i * w, cy, min(w / 4, h / 3), z + body, z + d)
    elif what == "header":
        cols, rows = f["cols"], f["rows"]
        mesh.box(mats["plastic"], *bx, z, z + SPACER_H)
        for i in range(cols):
            for j in range(rows):
                px = cx + (i - (cols - 1) / 2) * PITCH
                py = cy + (j - (rows - 1) / 2) * PITCH
                mesh.box(mats["gold"], px - 0.32, py - 0.32, px + 0.32, py + 0.32,
                         z, z + d)  # fmt: skip
    else:
        raise ValueError(f"modules.toml: unknown feature {what!r}")


def build_module(spec, mats, fitted):
    """Sockets always (soldered to the carrier); the breakout itself if fitted."""
    root = bpy.data.objects.new(spec["ref"], None)
    bpy.context.scene.collection.objects.link(root)
    root.location = (*spec["origin"], spec["z"])
    root.rotation_euler = (0, 0, math.radians(spec["angle"]))
    mesh = Mesh(root)
    outline = spec["outline"]
    if "body" in spec:  # a connector or fuse: a block to its height
        mesh.box(material(spec["kind"], (*spec["body"], 1), rough=0.45),
                 *outline, 0, spec["above"])  # fmt: skip
        mesh.finish(spec["ref"])
        return
    half = PITCH / 2
    if spec["sockets"]:
        for x, y in spec["pads"]:
            mesh.box(mats["plastic"], x - half, y - half, x + half, y + half,
                     0, SOCKET_H)  # fmt: skip
    if fitted:
        pcb_mat = material(f"{spec['kind']} pcb", (*spec["pcb"], 1), rough=0.3)
        t = spec.get("pcb_thickness", 1.6)
        z_pcb = SOCKET_H + SPACER_H
        for x, y in spec["pads"]:
            mesh.box(mats["plastic"], x - half + 0.1, y - half + 0.1,
                     x + half - 0.1, y + half - 0.1, SOCKET_H, z_pcb)  # fmt: skip
            mesh.cyl(mats["tin"], x, y, 0.55, z_pcb + t, z_pcb + t + 0.9, 12)
        mesh.box(pcb_mat, *outline, z_pcb, z_pcb + t)
        for f in spec.get("features", []):
            feature(mesh, mats, pcb_mat, f, outline, z_pcb + t)
    mesh.finish(spec["ref"])


def visible_bounds():
    dg = bpy.context.evaluated_depsgraph_get()
    pts = []
    for o in bpy.context.scene.objects:
        if o.type != "MESH" or o.hide_render:
            continue
        e = o.evaluated_get(dg)
        if not e.data.vertices:
            continue
        pts += [e.matrix_world @ mathutils.Vector(c) for c in e.bound_box]
    lo = mathutils.Vector([min(p[i] for p in pts) for i in range(3)])
    hi = mathutils.Vector([max(p[i] for p in pts) for i in range(3)])
    return lo, hi, pts


def studio(view):
    """Floor that only catches shadows, a soft key and fill, a bright world."""
    lo, hi, pts = visible_bounds()
    span = max(hi - lo)
    bpy.ops.mesh.primitive_plane_add(size=span * 40, location=(0, 0, lo.z))
    bpy.context.active_object.is_shadow_catcher = True

    centre = (lo + hi) / 2
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = view.get("lens", 70)
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.scene.collection.objects.link(cam)
    bpy.context.scene.camera = cam
    d = mathutils.Vector(view.get("direction", (-1.0, -1.45, 0.95))).normalized()
    cam.location = centre + d * span * 6
    cam.rotation_euler = (centre - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.view_layer.update()
    loc, _ = cam.camera_fit_coords(bpy.context.evaluated_depsgraph_get(),
                                   [c for p in pts for c in p])  # fmt: skip
    # Keep the fit's distance but stay on the aim line (the fit slides the
    # camera sideways), then pull back: a margin round the subject, so an
    # edit model's crop or a caption bar never touches it.
    dist = (loc - centre).dot(d)
    cam.location = centre + d * dist * view.get("margin", 1.2)
    cam_data.clip_start = span * 0.05
    cam_data.clip_end = span * 50
    # In perspective the box centre does not project to the frame centre;
    # shift the lens so the subject's projected bounds are centred.
    bpy.context.view_layer.update()
    sc = bpy.context.scene
    ndc = [world_to_camera_view(sc, cam, p) for p in pts]
    u = (min(p.x for p in ndc) + max(p.x for p in ndc)) / 2 - 0.5
    v = (min(p.y for p in ndc) + max(p.y for p in ndc)) / 2 - 0.5
    w, h = sc.render.resolution_x, sc.render.resolution_y  # set before studio()
    cam_data.shift_x = u * min(1.0, w / h)
    cam_data.shift_y = v * min(1.0, h / w)

    for name, energy, angle, rot in (
        ("key", 3.2, 12, (50, -15, -40)),
        ("fill", 0.9, 30, (60, 20, 120)),
        ("rim", 1.6, 6, (70, 0, 200)),
    ):
        sun = bpy.data.objects.new(name, bpy.data.lights.new(name, "SUN"))
        sun.data.energy = energy
        sun.data.angle = math.radians(angle)
        sun.rotation_euler = tuple(math.radians(a) for a in rot)
        bpy.context.scene.collection.objects.link(sun)
    world = bpy.data.worlds.new("studio")
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value = (1, 1, 1, 1)
    bg.inputs["Strength"].default_value = 0.55
    bpy.context.scene.world = world


def render(out, samples):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    try:
        prefs.compute_device_type = "METAL"
        prefs.get_devices()
        for dev in prefs.devices:
            dev.use = True
        sc.cycles.device = "GPU"
    except TypeError:  # no Metal on this machine
        sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    sc.render.film_transparent = True
    sc.render.image_settings.color_mode = "RGBA"
    sc.render.filepath = str(out)
    bpy.ops.render.render(write_still=True)


def view_scene(scene, view):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    mats = materials()
    if view.get("case") == "tray":
        import_stl(scene["tray"], mats["resin"])
    elif view.get("case") == "box":
        import_stl(scene["box_base"], mats["resin"])
    lid = view.get("lid")
    if lid == "closed":
        import_stl(scene["box_lid"], mats["resin"])
    elif lid == "lifted":
        import_stl(scene["box_lid"], mats["resin"], z_lift=0.6 * scene["size"][2])
    if view.get("board"):
        import_board(scene)
        for spec in scene["modules"]:
            build_module(spec, mats, view.get("modules", True))


def main(path):
    job = json.loads(Path(path).read_text())
    out = Path(job["out"])
    out.mkdir(parents=True, exist_ok=True)
    only = set(sys.argv[sys.argv.index("--") + 2 :])
    for view in job["views"]:
        if only and view["id"] not in only:
            continue
        scene = json.loads(Path(view["scene"]).read_text())
        view_scene(scene, view)
        sc = bpy.context.scene
        sc.render.resolution_x, sc.render.resolution_y = job["size"]
        studio(view)
        render(out / f"{view['id']}.png", job["samples"])


if __name__ == "__main__":
    main(sys.argv[sys.argv.index("--") + 1])
