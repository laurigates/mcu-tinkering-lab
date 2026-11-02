"""The clash sweep as an animation: every gate pose, and the travel between them.

``check`` tests the poses in ``SWEEP`` and nothing between them. This walks the
same poses in an order where each step moves one joint (a pan/tilt nods its
full tilt range at each pan stop), adds ``--between`` interpolated poses per
step, and runs the gate's own clash test (``check.overlaps``) at every frame.
A clash only at an interpolated pose is a clash the gate cannot see; it is
reported separately as ``between_clashes``.

Each frame is a VTK render from one fixed camera: printed parts in their
colours, stand-ins translucent, and on a clash both bodies turned red with
their common solid drawn opaque and labelled in mm3. Writes
``<design dir>/output/animation/``:

- ``sweep.gif``: every frame, holding longer on gate poses and on clashes
- ``sweep-sheet.png``: the gate poses alone as a grid (pan across, tilt down
  for a two-joint grid), clashing poses framed red
- ``sweep.json``: the path, ``gate_clashes`` and ``between_clashes``

Run: ``python -m mechanism.animate <design dir> [--between N] [--size WxH]``;
exits 1 on any clash, gate pose or between.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import pyvista as pv
from PIL import Image, ImageDraw, ImageFont

from mechanism.check import label as pose_label
from mechanism.check import load, overlaps, poses, rest
from mechanism.showcase import STANDIN, VIEWS, colours, mesh

CLASH = "#d62728"
CLEAR = "#2ca02c"
MS_STEP, MS_GATE, MS_CLASH = 90, 350, 900  # GIF frame durations
THUMB_W = 220
CAPTION_PX = 15


@dataclass
class Frame:
    pose: tuple
    gate: bool  # one of the poses check.py tests
    clashes: list[dict]


def _snake(axes: list[list[float]]) -> list[tuple]:
    """Every point of the grid ``axes`` in an order where each step moves one axis."""
    if len(axes) == 1:
        return [(v,) for v in axes[0]]
    inner = _snake(axes[1:])
    out = []
    for i, v in enumerate(axes[0]):
        out += [(v, *rest_) for rest_ in (inner if i % 2 == 0 else inner[::-1])]
    return out


def gate_path(mod) -> list[tuple]:
    """The design's ``SWEEP``, reordered one joint step at a time when it is a grid."""
    sweep = [tuple(p) for p in poses(mod)]
    axes = [sorted(set(vals)) for vals in zip(*sweep, strict=True)]
    grid = set(itertools.product(*axes))
    return _snake(axes) if grid == set(sweep) and len(grid) == len(sweep) else sweep


def path(mod, between: int = 2) -> list[tuple]:
    """``gate_path`` with ``between`` evenly spaced poses inserted in each step."""
    gates = gate_path(mod)
    out = [gates[0]]
    for a, b in zip(gates, gates[1:], strict=False):
        for k in range(1, between + 1):
            f = k / (between + 1)
            out.append(
                tuple(round(x + (y - x) * f, 3) for x, y in zip(a, b, strict=True))
            )
        out.append(b)
    return out


def iter_frames(mod, at: list[tuple]) -> Iterator[tuple[Frame, list, list]]:
    """``(frame, bodies, overlaps)`` per pose, one pose in memory at a time."""
    gates = set(poses(mod))
    for pose in at:
        bodies = mod.assembly(*pose)
        ov = overlaps(bodies)
        clashes = [{"a": a.name, "b": b.name, "mm3": round(v, 3)} for a, b, _, v in ov]
        yield Frame(pose, pose in gates, clashes), bodies, ov


def frames(mod, at: list[tuple]) -> list[Frame]:
    return [f for f, _, _ in iter_frames(mod, at)]


def summary(mod, fs: list[Frame]) -> dict:
    gate = {pose_label(mod, f.pose): f.clashes for f in fs if f.gate and f.clashes}
    between = {
        pose_label(mod, f.pose): f.clashes for f in fs if not f.gate and f.clashes
    }
    return {
        "path": [pose_label(mod, f.pose) for f in fs],
        "gate_clashes": gate,
        "between_clashes": between,
    }


def _camera(bodies, size, view=VIEWS[0], zoom: float = 0.95):
    """One camera for every frame, framed on the rest pose with room to swing."""
    p = pv.Plotter(off_screen=True, window_size=size)
    for b in bodies:
        p.add_mesh(mesh(b.part, tol=0.2))
    p.camera_position = "yz"
    p.camera.azimuth, p.camera.elevation = view[1], view[2]
    p.reset_camera()
    p.camera.zoom(zoom)
    cam = p.camera_position
    p.close()
    return cam


def render(bodies, ov, cols, cam, size) -> Image.Image:
    """One pose from the fixed camera; clashing bodies red, their common solid dark red."""
    hit = {n for a, b, _, _ in ov for n in (a.name, b.name)}
    p = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
    p.set_background("white")
    for b in bodies:
        m = mesh(b.part)
        red = b.name in hit
        p.add_mesh(
            m,
            color=CLASH if red else cols.get(b.name, STANDIN),
            opacity=0.4
            if red
            else (1.0 if b.printed else 0.3),  # see the overlap inside
            smooth_shading=True,
            specular=0.25 if b.printed else 0.05,
        )
        if b.printed:
            p.add_mesh(m.extract_feature_edges(40), color="#2b2b2b", line_width=1)
    for _, _, common, vol in ov:
        cm = mesh(common)
        p.add_mesh(cm, color="#7a0000", smooth_shading=True)
        p.add_point_labels(
            [cm.center],
            [f"{vol:.2f} mm3"],
            font_size=12,
            text_color=CLASH,
            point_color=CLASH,
            point_size=8,
            always_visible=True,
        )
    p.camera_position = cam
    img = Image.fromarray(p.screenshot(return_img=True))
    p.close()
    return img


def _font(px: int):
    try:
        return ImageFont.load_default(size=px)
    except (TypeError, OSError):  # Pillow built without FreeType
        return ImageFont.load_default()


def caption(img: Image.Image, title: str, status: str, bad: bool) -> Image.Image:
    """``img`` under a header band: the pose on one line, clear or the clash on the next."""
    band = 2 * CAPTION_PX + 14
    out = Image.new("RGB", (img.width, img.height + band), "white")
    out.paste(img.convert("RGB"), (0, band))
    d = ImageDraw.Draw(out)
    d.text((8, 4), title, fill="#333333", font=_font(CAPTION_PX))
    d.text(
        (8, CAPTION_PX + 8),
        status,
        fill=CLASH if bad else CLEAR,
        font=_font(CAPTION_PX),
    )
    return out


def sheet(mod, shots: list[tuple[Frame, Image.Image]], path_: Path) -> None:
    """The gate poses as one grid image; a two-joint grid lays out joint 0 across."""
    axes = [sorted({f.pose[i] for f, _ in shots}) for i in range(len(shots[0][0].pose))]
    if len(axes) == 2 and len(axes[0]) * len(axes[1]) == len(shots):
        cols, rows = len(axes[0]), len(axes[1])

        def cell(f):
            return axes[0].index(f.pose[0]), rows - 1 - axes[1].index(f.pose[1])
    else:
        cols = math.ceil(math.sqrt(len(shots)))
        rows = math.ceil(len(shots) / cols)
        order = {id(f): i for i, (f, _) in enumerate(shots)}

        def cell(f):
            return order[id(f)] % cols, order[id(f)] // cols

    w0, h0 = shots[0][1].size
    tw, ih = THUMB_W, round(THUMB_W * h0 / w0)
    th = ih + CAPTION_PX + 8  # the pose label sits under the picture, not on it
    out = Image.new("RGB", (cols * tw, rows * th), "white")
    draw = ImageDraw.Draw(out)
    font = _font(CAPTION_PX - 2)
    for f, img in shots:
        c, r = cell(f)
        x, y = c * tw, r * th
        out.paste(img.convert("RGB").resize((tw, ih), Image.LANCZOS), (x, y))
        text = pose_label(mod, f.pose) + ("  CLASH" if f.clashes else "")
        draw.text(
            (x + 6, y + ih + 3), text, fill=CLASH if f.clashes else "#333333", font=font
        )
        draw.rectangle(
            [x, y, x + tw - 1, y + th - 1],
            outline=CLASH if f.clashes else "#cccccc",
            width=4 if f.clashes else 1,
        )
    out.quantize(colors=128, method=Image.Quantize.MEDIANCUT).save(path_, optimize=True)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="python -m mechanism.animate")
    ap.add_argument("design_dir", type=Path)
    ap.add_argument(
        "--between", type=int, default=2, help="interpolated poses per gate step"
    )
    ap.add_argument("--size", default="640x512", help="frame size WxH")
    args = ap.parse_args(argv)
    design_dir = args.design_dir.resolve()
    size = tuple(int(v) for v in args.size.split("x"))
    mod = load(design_dir)
    out = design_dir / "output" / "animation"
    out.mkdir(parents=True, exist_ok=True)

    at_rest = mod.assembly(*rest(mod))
    cols, cam = colours(at_rest), _camera(at_rest, size)
    del at_rest
    at = path(mod, args.between)
    fs, imgs, shots, durations = [], [], [], []
    for i, (f, bodies, ov) in enumerate(iter_frames(mod, at), 1):
        kind = "gate pose" if f.gate else "between gate poses"
        status = (
            "CLASH "
            + ", ".join(f"{c['a']} / {c['b']} {c['mm3']:.2f} mm3" for c in f.clashes)
            if f.clashes
            else "clear"
        )
        title = f"{design_dir.name}: {pose_label(mod, f.pose)}  ({kind}, {i}/{len(at)})"
        shot = render(bodies, ov, cols, cam, size)
        if f.gate:
            shots.append(shot)
        imgs.append(caption(shot, title, status, bool(f.clashes)))
        durations.append(MS_CLASH if f.clashes else MS_GATE if f.gate else MS_STEP)
        fs.append(f)
        print(f"{i}/{len(at)} {pose_label(mod, f.pose)}: {status}", flush=True)

    pal = [
        im.convert("RGB").quantize(colors=128, method=Image.Quantize.MEDIANCUT)
        for im in imgs
    ]
    pal[0].save(
        out / "sweep.gif",
        save_all=True,
        append_images=pal[1:],
        duration=durations,
        loop=0,
        optimize=True,
    )
    sheet(
        mod,
        list(zip([f for f in fs if f.gate], shots, strict=True)),
        out / "sweep-sheet.png",
    )
    s = {"entry": design_dir.name, "between": args.between, **summary(mod, fs)}
    (out / "sweep.json").write_text(json.dumps(s, indent=2) + "\n")
    print(out / "sweep.gif")
    return 1 if s["gate_clashes"] or s["between_clashes"] else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
