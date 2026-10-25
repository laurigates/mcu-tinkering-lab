"""Shaded 3D renders of an entry, one image per view, with VTK (pyvista).

common/render.py's matplotlib views sort faces, not pixels, so parts show
through each other; these use a depth buffer. Writes
``output/showcase/<view>.png``:

- assembled from four angles at rest, and at the pan/tilt corner (60°, 30°)
- exploded: each body pushed out from the assembly's centre, groups lifted
  in the order they stack (base at the bottom), so every part is visible
- parts: the printed parts alone, side by side as they are ordered

Printed parts take their ``colour`` (or a palette colour); servos, horns, the
head board and screws are grey stand-ins. Needs ``libosmesa6`` for headless
VTK.

Run: ``python -m common.showcase entries/<slug>`` from hardware/pan-tilt/.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pyvista as pv

from common.check import load

PALETTE = ["#d9822b", "#3a7ca5", "#7a9e3d", "#b54a68", "#8a6bbf", "#c9b037", "#4aa89a"]
STANDIN = "#7fa7d6"  # servos, horns, head, screws: pale blue, translucent
SIZE = (1200, 1000)
EXPLODE_MM = 28.0  # radial push per body from the assembly centre
LIFT_MM = 22.0  # extra lift per group, in stacking order

# (name, pan, tilt, camera azimuth, elevation) — azimuth 0 looks from +X (the
# front, where the camera points), 90 from +Y (the robot's left).
ASSEMBLED = [
    ("front-left", 0, 0, 35, 20),
    ("back-right", 0, 0, 215, 25),
    ("front", 0, 0, 0, 5),
    ("side", 0, 0, 90, 5),
    ("top", 0, 0, 20, 80),
    ("corner-pan60-tilt30", 60, 30, 35, 20),
]


def _mesh(part, tol=0.05) -> pv.PolyData:
    verts, tris = part.tessellate(tol)
    v = np.array([(p.X, p.Y, p.Z) for p in verts])
    f = np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).ravel()
    return pv.PolyData(v, f).compute_normals(split_vertices=True, feature_angle=30)


def _colours(bodies) -> dict[str, str]:
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


def _add(p, mesh, colour, printed):
    p.add_mesh(
        mesh,
        color=colour,
        opacity=1.0 if printed else 0.35,
        smooth_shading=True,
        specular=0.25 if printed else 0.05,
        show_edges=False,
    )
    if printed:
        p.add_mesh(mesh.extract_feature_edges(40), color="#2b2b2b", line_width=1)


def _shoot(p, path: Path, azim: float, elev: float, title: str) -> None:
    p.camera_position = "yz"
    p.camera.azimuth = azim
    p.camera.elevation = elev
    p.reset_camera()
    p.camera.zoom(1.15)
    p.add_text(title, position="upper_left", font_size=11, color="#333333")
    p.screenshot(str(path))
    p.close()
    print(path)


def _explode(bodies, meshes):
    centre = np.mean([m.center for m in meshes], axis=0)
    groups = {}
    for b, m in zip(bodies, meshes, strict=True):
        groups.setdefault(b.group, []).append(m.center[2])
    rank = {
        g: i for i, g in enumerate(sorted(groups, key=lambda g: np.mean(groups[g])))
    }
    out = []
    for b, m in zip(bodies, meshes, strict=True):
        d = np.array(m.center) - centre
        n = np.linalg.norm(d)
        push = d / n * EXPLODE_MM if n > 1e-6 else np.zeros(3)
        out.append(m.translate(push + (0, 0, rank[b.group] * LIFT_MM), inplace=False))
    return out


def main(argv: list[str]) -> int:
    entry = Path(argv[0]).resolve()
    mod = load(entry)
    out = entry / "output" / "showcase"
    out.mkdir(parents=True, exist_ok=True)

    for name, pan, tilt, azim, elev in ASSEMBLED:
        bodies = mod.assembly(pan, tilt)
        colours = _colours(bodies)
        p = _plotter()
        for b in bodies:
            _add(p, _mesh(b.part), colours.get(b.name, STANDIN), b.printed)
        _shoot(
            p,
            out / f"assembled-{name}.png",
            azim,
            elev,
            f"{entry.name}: assembled, {name} (pan {pan}°, tilt {tilt}°)",
        )

    bodies = mod.assembly(0, 0)
    colours = _colours(bodies)
    for name, azim, elev in [("front-left", 35, 22), ("back-right", 215, 22)]:
        p = _plotter()
        for b, m in zip(
            bodies, _explode(bodies, [_mesh(b.part) for b in bodies]), strict=True
        ):
            _add(p, m, colours.get(b.name, STANDIN), b.printed)
        _shoot(
            p,
            out / f"exploded-{name}.png",
            azim,
            elev,
            f"{entry.name}: exploded, {name}",
        )

    parts = mod.printed_parts()
    p = _plotter()
    x = 0.0
    for i, (name, part) in enumerate(parts.items()):
        m = _mesh(part)
        lo = np.array(m.bounds[::2])
        m = m.translate((x - lo[0], -m.center[1], -lo[2]), inplace=False)
        colour = colours.get(name, PALETTE[i % len(PALETTE)])
        _add(p, m, colour, True)
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
    _shoot(p, out / "parts.png", 30, 35, f"{entry.name}: printed parts ({len(parts)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
