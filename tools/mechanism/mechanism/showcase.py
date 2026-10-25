"""Shaded renders of a mechanism design, one PNG per view, with VTK (pyvista).

Depth-buffered, unlike matplotlib's per-face sort (``quickview``), so parts do
not show through each other. Writes ``<design dir>/output/showcase/``:

- ``assembled-<view>.png``: five angles at the rest pose, and each of the
  design's ``SHOWCASE_POSES`` (a pan/tilt's travel corner by default)
- ``exploded-<view>.png``: every body pushed out from the assembly's centre and
  each group lifted in stacking order, capped so the stack stays in frame
- ``parts.png``: the printed parts alone, side by side and labelled

Printed parts take their ``colour`` (else a palette colour); stand-ins
(servos, horns, boards, screws) are pale blue and translucent.

Headless VTK needs Mesa's off-screen library (``libosmesa6`` on Debian and
Ubuntu); ``just mechanism::setup`` installs it. Blender (``bpy`` 5.2 from
PyPI) also renders headless here, but Cycles is ~7 s a frame on CPU and EEVEE
~45 s through software GL, against under 3 s a view with VTK, and the extra
fidelity buys nothing for checking a mechanism. Keep Blender for product
renders (tools/enclosure).

Run: ``python -m mechanism.showcase <design dir>``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pyvista as pv
from mechanism.check import DEFAULT_POSES, load, rest

PALETTE = ["#d9822b", "#3a7ca5", "#7a9e3d", "#b54a68", "#8a6bbf", "#c9b037", "#4aa89a"]
STANDIN = "#7fa7d6"  # servos, horns, boards, screws: pale blue, translucent
SIZE = (1200, 1000)
EXPLODE_MM = 28.0  # radial push per body from the assembly centre
LIFT_MM = 22.0  # extra lift per group, in stacking order...
LIFT_SPAN = 1.0  # ...capped so all lifts add up to this many assembly heights

# (name, camera azimuth, elevation): azimuth 0 looks from +X (the front), 90
# from +Y (the left).
VIEWS = [
    ("front-left", 35, 20),
    ("back-right", 215, 25),
    ("front", 0, 5),
    ("side", 90, 5),
    ("top", 20, 80),
]


def mesh(part, tol: float = 0.05) -> pv.PolyData:
    verts, tris = part.tessellate(tol)
    v = np.array([(p.X, p.Y, p.Z) for p in verts])
    f = np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).ravel()
    return pv.PolyData(v, f).compute_normals(split_vertices=True, feature_angle=30)


def colours(bodies) -> dict[str, str]:
    out, i = {}, 0
    for b in bodies:
        if b.printed:
            out[b.name] = b.colour or PALETTE[i % len(PALETTE)]
            i += 1
    return out


def _plotter() -> pv.Plotter:
    p = pv.Plotter(off_screen=True, window_size=SIZE, lighting="three lights")
    p.set_background("white")
    p.enable_anti_aliasing("ssaa")
    return p


def _add(p, m, colour, printed):
    p.add_mesh(
        m,
        color=colour,
        opacity=1.0 if printed else 0.35,
        smooth_shading=True,
        specular=0.25 if printed else 0.05,
    )
    if printed:
        p.add_mesh(m.extract_feature_edges(40), color="#2b2b2b", line_width=1)


def _shoot(p, path: Path, azim: float, elev: float, title: str, zoom: float = 1.15):
    p.camera_position = "yz"
    p.camera.azimuth = azim
    p.camera.elevation = elev
    p.reset_camera()
    p.camera.zoom(zoom)
    p.add_text(title, position="upper_left", font_size=11, color="#333333")
    p.screenshot(str(path))
    p.close()
    print(path)


def explode(bodies, meshes) -> list[pv.PolyData]:
    centre = np.mean([m.center for m in meshes], axis=0)
    groups: dict[str, list[float]] = {}
    for b, m in zip(bodies, meshes, strict=True):
        groups.setdefault(b.group, []).append(m.center[2])
    order = sorted(groups, key=lambda g: np.mean(groups[g]))
    rank = {g: i for i, g in enumerate(order)}
    height = max(m.bounds[5] for m in meshes) - min(m.bounds[4] for m in meshes)
    lift = min(LIFT_MM, LIFT_SPAN * height / max(len(rank) - 1, 1))
    out = []
    for b, m in zip(bodies, meshes, strict=True):
        d = np.array(m.center) - centre
        n = np.linalg.norm(d)
        push = d / n * EXPLODE_MM if n > 1e-6 else np.zeros(3)
        out.append(m.translate(push + (0, 0, rank[b.group] * lift), inplace=False))
    return out


def render_assembly(bodies, path: Path, view: tuple, title: str, exploded=False):
    _, azim, elev = view
    cols = colours(bodies)
    meshes = [mesh(b.part) for b in bodies]
    if exploded:
        meshes = explode(bodies, meshes)
    p = _plotter()
    for b, m in zip(bodies, meshes, strict=True):
        _add(p, m, cols.get(b.name, STANDIN), b.printed)
    # the lifted stack is tall; leave its top in frame
    _shoot(p, path, azim, elev, title, zoom=0.9 if exploded else 1.15)


def render_parts(parts: dict, cols: dict, path: Path, title: str):
    p = _plotter()
    x = 0.0
    for i, (name, part) in enumerate(parts.items()):
        m = mesh(part)
        lo = np.array(m.bounds[::2])
        m = m.translate((x - lo[0], -m.center[1], -lo[2]), inplace=False)
        _add(p, m, cols.get(name, PALETTE[i % len(PALETTE)]), True)
        p.add_point_labels(
            [[x + (m.bounds[1] - m.bounds[0]) / 2, 0, m.bounds[5] + 4]],
            [name],
            font_size=14,
            shape=None,
            show_points=False,
            always_visible=True,
            text_color="#333333",
        )
        x = m.bounds[1] + 12.0
    _shoot(p, path, 30, 35, title)


def main(argv: list[str]) -> int:
    design_dir = Path(argv[0]).resolve()
    mod = load(design_dir)
    out = design_dir / "output" / "showcase"
    out.mkdir(parents=True, exist_ok=True)
    name = design_dir.name
    at_rest = mod.assembly(*rest(mod))

    for view in VIEWS:
        render_assembly(
            at_rest,
            out / f"assembled-{view[0]}.png",
            view,
            f"{name}: assembled, {view[0]} (rest)",
        )
    for pose_name, pose in getattr(mod, "SHOWCASE_POSES", DEFAULT_POSES):
        render_assembly(
            mod.assembly(*pose),
            out / f"assembled-{pose_name}.png",
            VIEWS[0],
            f"{name}: assembled, {pose_name}",
        )
    for view in (("front-left", 35, 22), ("back-right", 215, 22)):
        render_assembly(
            at_rest,
            out / f"exploded-{view[0]}.png",
            view,
            f"{name}: exploded, {view[0]}",
            exploded=True,
        )
    parts = mod.printed_parts()
    render_parts(
        parts,
        colours(at_rest),
        out / "parts.png",
        f"{name}: printed parts ({len(parts)})",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
