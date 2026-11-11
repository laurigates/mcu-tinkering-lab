"""Animate the assembly motions `gen` found: the board going into each case,
then the lid going on. A blocked step shows the board lifted to where it
jams, with the touching points in red.

Reads the <piece>-{tray-board,box-board,box-lid}.json and STL files `gen`
writes, renders with VTK (pyvista, off-screen) and writes, per piece and
style, <piece>-<style>-assembly.mp4 (needs ffmpeg on PATH) and
<piece>-<style>-assembly-keys.png, a strip of key frames.

Headless VTK needs Mesa's off-screen library (`libosmesa6` on Debian and
Ubuntu, as for tools/mechanism).

Run: ``python -m enclosure.sequence <project> [--fps 30] [--seconds 6]``.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import pyvista as pv

from .assemble import transform

CASE, BOARD, LID, CLASH = "#d9d4c7", "#2f7d4a", "#9db4c9", "#d62728"
SIZE = (960, 720)


def _poly(path: Path) -> pv.PolyData:
    return pv.read(str(path))


def _frames(path: list[list[float]], count: int) -> list[tuple[float, ...]]:
    """`count` poses along the path, spaced by how far each hop moves."""
    if len(path) < 2:
        return [tuple(path[0])] * count if path else []
    p = np.array(path)
    hop = np.abs(np.diff(p, axis=0))
    # 1 degree of tilt counts like 1 mm of travel for pacing.
    length = np.r_[0, np.cumsum(hop.max(axis=1))]
    t = np.linspace(0, length[-1], count)
    return [tuple(np.interp(x, length, p[:, i]) for i in range(5)) for x in t]


def _step(out: Path, stem: str) -> dict | None:
    js = out / f"{stem}.json"
    if not js.exists():
        return None
    step = json.loads(js.read_text())
    step["moving"] = _poly(out / f"{stem}-moving.stl")
    step["fixed"] = _poly(out / f"{stem}-fixed.stl")
    step["stem"] = stem
    return step


def _plan_frames(step: dict, count: int) -> list[tuple[float, ...]]:
    if step["found"]:
        return _frames(step["path"], count)
    # Blocked: rise from the seated pose to where it jams, and hold there,
    # where its clash points were taken.
    d = step.get("jam", step["deepest"])
    rise = _frames([[0, 0, 0, 0, 0], d], count // 2)
    return rise + [tuple(d)] * (count - len(rise))


def render(project: Path, fps: int = 30, seconds: float = 6.0) -> list[Path]:
    out = project / "hardware" / "enclosure" / "output"
    written = []
    for js in sorted(out.glob("*-board.json")):
        stem = js.stem  # <piece>-<style>-board
        piece, style = stem.rsplit("-", 2)[:2]
        steps = [_step(out, stem)]
        if style == "box":
            lid = _step(out, f"{piece}-box-lid")
            if lid is not None:
                steps.append(lid)
        written += _render_piece(out, piece, style, steps, fps, seconds)
    return written


def _render_piece(out, piece, style, steps, fps, seconds) -> list[Path]:
    n = max(2, int(fps * seconds / len(steps)))
    board_step = steps[0]
    base = board_step["fixed"]
    board = board_step["moving"]
    tmp = Path(tempfile.mkdtemp(prefix="enclosure-seq-"))
    pl = pv.Plotter(off_screen=True, window_size=SIZE)
    pl.set_background("white")
    pl.enable_anti_aliasing("ssaa")
    pl.add_mesh(base, color=CASE, opacity=0.45, smooth_shading=True)
    b_actor = pl.add_mesh(board, color=BOARD, smooth_shading=True)
    lid_actor = None
    if len(steps) > 1:
        lid_actor = pl.add_mesh(steps[1]["moving"], color=LID, opacity=0.6)
        lid_actor.visibility = False
    text = pl.add_text("", position="upper_left", font_size=12, color="black")
    bb = np.array(base.bounds).reshape(3, 2)
    centre = bb.mean(axis=1)
    span = float(np.ptp(bb, axis=1).max())
    pl.camera.focal_point = centre + np.array([0, 0, span * 0.2])
    pl.camera.position = centre + np.array([-1.3, -1.6, 1.4]) * span * 0.75
    pl.camera.up = (0, 0, 1)
    frames: list[Path] = []
    keys: list[np.ndarray] = []

    def shoot(label: str, key: bool = False) -> None:
        text.set_text("upper_left", label)
        pl.render()  # screenshot() alone reuses the last image
        f = tmp / f"{len(frames):05d}.png"
        pl.screenshot(str(f))
        frames.append(f)
        if key:
            keys.append(pl.screenshot(None, return_img=True))

    def pose(actor, step, q):
        actor.user_matrix = transform(q, np.array(step["pivot"]))

    for i, step in enumerate(steps):
        actor = b_actor if i == 0 else lid_actor
        if i == 1:
            actor.visibility = True
        what = "board" if i == 0 else "lid"
        poses = _plan_frames(step, n)
        if step["found"]:
            label = f"{piece} {style}: {what} in by {step['strategy']}"
        else:
            label = f"{piece} {style}: {what} BLOCKED ({step['note']})"
        for k, q in enumerate(poses):
            pose(actor, step, q)
            shoot(label, key=k in (0, len(poses) // 2))
        if not step["found"] and step["clash_points"]:
            pts = pv.PolyData(np.array(step["clash_points"]))
            pl.add_mesh(pts, color=CLASH, point_size=8, render_points_as_spheres=True)
            for _ in range(fps):
                shoot(label + ": red = touching")
            keys.append(pl.screenshot(None, return_img=True))
            break  # nothing after a blocked step can happen
        shoot(label, key=True)
    pl.close()
    if not keys:
        keys.append(np.asarray(pl.last_image))
    stem = out / f"{piece}-{style}-assembly"
    strip = np.concatenate(keys[:6], axis=1)
    _save_png(strip, Path(f"{stem}-keys.png"))
    written = [Path(f"{stem}-keys.png")]
    if shutil.which("ffmpeg"):
        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-framerate",
                str(fps),
                "-i",
                str(tmp / "%05d.png"),
                "-pix_fmt",
                "yuv420p",
                "-vf",
                "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                f"{stem}.mp4",
            ],
            check=True,
        )
        written.append(Path(f"{stem}.mp4"))
    else:
        print("ffmpeg not on PATH: frames left in", tmp, file=sys.stderr)
        return written
    shutil.rmtree(tmp)
    return written


def _save_png(img: np.ndarray, path: Path) -> None:
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk

    h, w = img.shape[:2]
    data = vtk.vtkImageData()
    data.SetDimensions(w, h, 1)
    arr = numpy_to_vtk(np.flipud(img).reshape(-1, img.shape[2]), deep=True)
    data.GetPointData().SetScalars(arr)
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(path))
    writer.SetInputData(data)
    writer.Write()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m enclosure.sequence")
    ap.add_argument("project", type=Path)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=6.0)
    a = ap.parse_args(argv)
    for f in render(a.project.resolve(), a.fps, a.seconds):
        print(f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
