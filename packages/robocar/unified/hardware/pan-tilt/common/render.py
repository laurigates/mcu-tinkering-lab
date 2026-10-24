"""Shaded PNGs of an entry's assembly, with matplotlib (no Blender needed).

Printed parts in their declared colour, stand-ins grey and translucent. Writes
``output/render-<view>.png`` per pose for comparison between entries:
iso at rest, front at rest, and iso at the pan/tilt corner (60, 30).

Run: ``python -m common.render entries/<slug>`` from hardware/pan-tilt/.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from common.check import load

PALETTE = ["#d9822b", "#3a7ca5", "#7a9e3d", "#b54a68", "#8a6bbf", "#c9b037", "#4aa89a"]
VIEWS = {"iso": (24, -52), "front": (8, 0), "side": (8, -90), "back": (24, 140)}
POSES = [
    ("rest", 0, 0, "iso"),
    ("rest-front", 0, 0, "front"),
    ("corner", 60, 30, "iso"),
    ("rest-back", 0, 0, "back"),
]


def _tris(part, tol=0.2):
    verts, faces = part.tessellate(tol)
    v = np.array([(p.X, p.Y, p.Z) for p in verts])
    return v[np.array(faces)]


def draw(bodies, path: Path, view: str, title: str) -> None:
    fig = plt.figure(figsize=(7, 7), dpi=130)
    ax = fig.add_subplot(projection="3d")
    light = np.array([0.4, -0.5, 0.8])
    light /= np.linalg.norm(light)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    colours = {}
    for b in bodies:
        tris = _tris(b.part)
        lo, hi = (
            np.minimum(lo, tris.reshape(-1, 3).min(0)),
            np.maximum(hi, tris.reshape(-1, 3).max(0)),
        )
        n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
        if b.printed:
            base = matplotlib.colors.to_rgb(
                getattr(b, "colour", None)
                or colours.setdefault(b.name, PALETTE[len(colours) % len(PALETTE)])
            )
            alpha = 1.0
        else:
            base, alpha = (0.55, 0.55, 0.58), 0.35
        fc = np.clip(np.outer(shade, base), 0, 1)
        ax.add_collection3d(
            Poly3DCollection(tris, facecolors=fc, edgecolors="none", alpha=alpha)
        )
    c, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(*VIEWS[view])
    ax.set_axis_off()
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, facecolor="white")
    plt.close(fig)


def main(argv: list[str]) -> int:
    entry = Path(argv[0]).resolve()
    mod = load(entry)
    out = entry / "output"
    out.mkdir(exist_ok=True)
    for name, pan, tilt, view in POSES:
        draw(
            mod.assembly(pan, tilt),
            out / f"render-{name}.png",
            view,
            f"{entry.name}: pan {pan}°, tilt {tilt}° ({view})",
        )
        print(out / f"render-{name}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
