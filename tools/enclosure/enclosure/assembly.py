"""Assembly scenes: the populated board seated in its case, for review renders.

    python -m enclosure.assembly <project> [<project> ...]

For each piece, writes <piece>-assembly.json beside the case STLs: the board
GLB KiCad exports (real colours, silkscreen, every 3D model it has), the
transform that seats it on the standoffs, the case parts, and stand-in blocks
for the parts with no 3D model (the heights enclosure.toml estimates).
blender/render_assembly.py turns each scene into PNGs.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tomllib
from pathlib import Path

from build123d import Align, Box, Pos, export_stl
from kicad import paths
from kicad.paths import REPO_ROOT

from . import board, case, modules

A = (Align.MIN,) * 3


def export_glb(project: Path, out: Path) -> Path:
    """The board as KiCad draws it, top-left corner at the origin (like the STEP)."""
    cfg = paths.load_board_config(project)
    ox, oy = cfg.data["outline"]["origin"]
    pcb = paths.kicad_project_dir(project) / f"{cfg.name}.kicad_pcb"
    subprocess.run(
        [
            str(board.KICAD_CLI), "pcb", "export", "glb", "--force", "--subst-models",
            "--include-silkscreen", "--include-soldermask", "--include-pads",
            "--include-tracks", "--user-origin", f"{ox}x{oy}mm",
            "--define-var", f"KICAD10_3DMODEL_DIR={board.KICAD_3DMODELS}",
            "-o", str(out), str(pcb),
        ],
        check=True, capture_output=True, encoding="utf-8", errors="replace",
    )  # fmt: skip
    return out


def scenes(project: Path) -> list[Path]:
    out = project / "hardware" / "enclosure" / "output"
    step = out / "board.step"
    if not step.exists():
        board.export_step(project, step)
    glb = export_glb(project, out / "board.glb")
    cfg = tomllib.loads(
        (project / "hardware" / "enclosure" / board.ENCLOSURE_TOML).read_text()
    )
    params = case.Params.from_toml(cfg.get("case", {}))
    written = []
    for piece in board.load_pieces(project, step):
        f = case.Frame(piece, params)
        r = piece.region
        stand_ins = []
        for q in piece.parts:
            if (
                q.side == "top"
                and q.height_source.startswith("enclosure.toml")
                and q.above
            ):
                x, y, w, h = f.rect(q.courtyard)
                stand_ins.append(Pos(x, y, f.z_top) * Box(w, h, q.above, align=A))
        proxies = None
        if stand_ins:
            proxies = out / f"{piece.name}-stand-ins.stl"
            solid = stand_ins[0]
            for s in stand_ins[1:]:
                solid += s
            export_stl(solid, str(proxies))
        wall_top = f.z_top + f.headroom + params.lid_clearance
        scene = {
            "piece": piece.name,
            "board_glb": str(glb),
            # Blender imports the GLB in metres, board top-left at (0, 0),
            # y up, bottom face at z 0: scale to mm, then seat it.
            "board_scale": 1000.0,
            "board_offset": [f.off - r.x0, f.off + r.y1, f.z_board],
            # The panel is one board; keep only this piece's region.
            "region_box": [f.off, f.off, f.z_board - 20, r.width, r.height, 80],
            "stand_ins": str(proxies) if proxies else None,
            "stand_in_refs": [
                q.ref
                for q in piece.parts
                if q.side == "top"
                and q.height_source.startswith("enclosure.toml")
                and q.above
            ],
            "tray": str(out / f"{piece.name}-tray.stl"),
            "box_base": str(out / f"{piece.name}-box-base.stl"),
            "box_lid": str(out / f"{piece.name}-box-lid.stl"),
            # Look-alikes of those parts, for the product renders.
            "modules": modules.specs(project, piece, f, cfg.get("heights", {})),
            "lid_bottom_z": wall_top,
            "size": [f.length, f.width, wall_top + params.lid],
        }
        path = out / f"{piece.name}-assembly.json"
        path.write_text(json.dumps(scene, indent=2) + "\n")
        written.append(path)
    return written


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        for p in scenes((REPO_ROOT / arg).resolve()):
            print(p.relative_to(REPO_ROOT))
