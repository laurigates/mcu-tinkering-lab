"""python -m enclosure <project> [<project> ...]

For each piece in <project>/hardware/enclosure/enclosure.toml, write the tray
and the box (base and lid) as STEP, STL and 3MF under
<project>/hardware/enclosure/output/, check that neither touches the board or
a part, and print the sizes and volumes a 3D-printing quote is priced on.
Exits non-zero on a clash or a missing part height.
"""

from __future__ import annotations

import argparse
import json
import sys
import tomllib
from pathlib import Path

from build123d import Align, Box, Mesher, Pos, export_step, export_stl, import_step
from kicad.paths import REPO_ROOT

from . import board, case

# Density, g/cm³, for a mass estimate only; quotes are priced by the fab.
DENSITY = {"PLA": 1.24, "resin": 1.15, "PA12": 1.01}


def write(part, stem: Path) -> None:
    export_step(part, f"{stem}.step")
    export_stl(part, f"{stem}.stl")
    m = Mesher()
    m.add_shape(part)
    m.write(f"{stem}.3mf")


def generate(project: Path, check: bool = True) -> list[dict]:
    out = project / "hardware" / "enclosure" / "output"
    out.mkdir(parents=True, exist_ok=True)
    step = board.export_step(project, out / "board.step")
    pieces = board.load_pieces(project, step)
    cfg = tomllib.loads(
        (project / "hardware" / "enclosure" / board.ENCLOSURE_TOML).read_text()
    )
    params = case.Params.from_toml(cfg.get("case", {}))
    board_step = import_step(step) if check else None
    report = []
    for piece in pieces:
        f = case.Frame(piece, params)
        region_box = Pos(f.off, f.off, -50) * Box(
            piece.region.width, piece.region.height, 200, align=(Align.MIN,) * 3
        )
        keep = case.keepout(f, board_step) if check else None
        tray = case.tray(f)
        base, lid = case.box(f)
        clash = {
            "tray": case.interference(tray, keep, region_box),
            "box-base": case.interference(base, keep, region_box),
            "box-lid": case.interference(lid, keep, region_box),
        }
        write(tray, out / f"{piece.name}-tray")
        write(base, out / f"{piece.name}-box-base")
        write(lid, out / f"{piece.name}-box-lid")
        export_step(base + lid, str(out / f"{piece.name}-box.step"))
        for style, solids in (("tray", [tray]), ("box", [base, lid])):
            bb = (
                solids[0] if len(solids) == 1 else solids[0] + solids[1]
            ).bounding_box()
            vol = sum(s.volume for s in solids) / 1000
            report.append(
                {
                    "project": str(project.relative_to(REPO_ROOT)),
                    "piece": piece.name,
                    "style": style,
                    "quantity": piece.quantity,
                    "size_mm": [
                        round(bb.size.X, 1),
                        round(bb.size.Y, 1),
                        round(bb.size.Z, 1),
                    ],
                    "volume_cm3": round(vol, 2),
                    "mass_g": {k: round(vol * d, 1) for k, d in DENSITY.items()},
                    "standoff_mm": round(f.standoff, 2),
                    "headroom_mm": round(f.headroom, 2),
                    "tallest": f"{piece.tallest.ref} {piece.tallest.above:.1f} mm ({piece.tallest.height_source})",
                    "clash_mm3": {
                        k: round(v, 3) for k, v in clash.items() if k.startswith(style)
                    },
                    "files": [
                        str((out / f"{piece.name}-{n}.stl").relative_to(REPO_ROOT))
                        for n in (
                            ["tray"] if style == "tray" else ["box-base", "box-lid"]
                        )
                    ],
                }
            )
    (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="python -m enclosure", description=__doc__.splitlines()[0]
    )
    ap.add_argument("projects", nargs="+", type=Path)
    ap.add_argument(
        "--no-check", action="store_true", help="skip the interference check"
    )
    args = ap.parse_args(argv)
    bad = 0
    for project in args.projects:
        project = (REPO_ROOT / project).resolve()  # absolute paths pass through
        try:
            rows = generate(project, check=not args.no_check)
        except board.EnclosureError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            bad += 1
            continue
        for r in rows:
            clash = sum(r["clash_mm3"].values())
            flag = "OK" if clash < 0.01 else f"CLASH {r['clash_mm3']}"
            print(
                f"{r['project']} {r['piece']} {r['style']} x{r['quantity']}: "
                f"{r['size_mm'][0]} x {r['size_mm'][1]} x {r['size_mm'][2]} mm, "
                f"{r['volume_cm3']} cm³, standoff {r['standoff_mm']}, "
                f"headroom {r['headroom_mm']}, tallest {r['tallest']}; {flag}"
            )
            bad += clash >= 0.01
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
