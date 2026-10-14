"""Fabrication outputs for a carrier: gerbers and drill files zipped for JLCPCB,
plus the BOM and CPL for its assembly service.

    PYTHONPATH=tools python3 -m kicad.fab [project ...] [--allow-none]

Needs KiCad 10.0.6 (`kicad-cli`, from $KICAD_CLI or the PATH). It exports what
was checked; `just <project>::carrier-fab` runs `carrier-check` first, as a
just dependency rather than from here. Per project, into the KiCad project's
`output/` (git-ignored):

- `fab/`: gerbers for the copper, paste, silkscreen, mask and Edge.Cuts layers
  (`--check-zones`), and Excellon drill files with gerber X2 drill maps;
- `<name>-fab.zip`: `fab/`'s files, the upload for JLCPCB's PCB order;
- `assembly/bom.csv` (`sch export bom`, grouped by value, footprint and LCSC)
  and `assembly/pos.csv` (front-side SMD placements);
- `assembly/bom-jlc.csv` and `assembly/cpl-jlc.csv` (`kicad.jlc`): only the
  parts with an LCSC number, for JLCPCB assembly; skip them to hand-solder.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

from . import jlc, paths, vendor
from .errors import KicadError
from .paths import REPO_ROOT

GERBER_LAYERS = (
    "F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
)


def kicad_cli() -> str:
    found = vendor.kicad_cli()
    if not found:
        raise KicadError(
            "kicad-cli not found (set KICAD_CLI): kicad.fab needs KiCad 10.0.6"
        )
    return found


def run(argv: list[str]) -> None:
    done = subprocess.run(argv, capture_output=True, text=True)
    out = (done.stdout + done.stderr).strip()
    if out:
        print(out)
    if done.returncode != 0:
        raise KicadError(f"{' '.join(argv[:4])}: exit {done.returncode}")


def fresh_dir(path: Path) -> Path:
    shutil.rmtree(path, ignore_errors=True)
    path.mkdir(parents=True)
    return path


def zip_dir(src: Path, dest: Path) -> None:
    """`src`'s files, flat and sorted, into a new zip at `dest`."""
    dest.unlink(missing_ok=True)
    with zipfile.ZipFile(dest, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(src.iterdir()):
            if f.is_file():
                z.write(f, f.name)


def fab(project: Path, cli: str) -> Path:
    """Write every fab output for one project; the output directory."""
    cfg = paths.load_board_config(project)
    kicad_dir = paths.carrier_dir(project) / cfg.name
    pcb = kicad_dir / f"{cfg.name}.kicad_pcb"
    sch = kicad_dir / f"{cfg.name}.kicad_sch"
    for f in (pcb, sch):
        if not f.is_file():
            raise KicadError(f"{f}: missing")
    out = kicad_dir / "output"
    gerbers = fresh_dir(out / "fab")
    run(
        [
            cli,
            "pcb",
            "export",
            "gerbers",
            "--check-zones",
            "--layers",
            GERBER_LAYERS,
            "-o",
            str(gerbers),
            str(pcb),
        ]
    )
    run(
        [
            cli,
            "pcb",
            "export",
            "drill",
            "--excellon-separate-th",
            "--generate-map",
            "--map-format",
            "gerberx2",
            "-o",
            f"{gerbers}/",
            str(pcb),
        ]
    )
    archive = out / f"{cfg.name}-fab.zip"
    zip_dir(gerbers, archive)
    print(f"wrote {archive}")
    asm = fresh_dir(out / "assembly")
    run(
        [
            cli,
            "sch",
            "export",
            "bom",
            "--fields",
            "Value,Reference,Footprint,LCSC",
            "--labels",
            "Comment,Designator,Footprint,LCSC",
            "--group-by",
            "Value,Footprint,LCSC",
            "--ref-range-delimiter",
            "",
            "--exclude-dnp",
            "-o",
            str(asm / "bom.csv"),
            str(sch),
        ]
    )
    run(
        [
            cli,
            "pcb",
            "export",
            "pos",
            "--format",
            "csv",
            "--units",
            "mm",
            "--side",
            "front",
            "--smd-only",
            "-o",
            str(asm / "pos.csv"),
            str(pcb),
        ]
    )
    jlc.write(asm / "bom.csv", asm / "pos.csv", asm)
    return out


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.fab", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    args = parser.parse_args(argv)
    try:
        projects = paths.select_projects(args.projects, args.allow_none, repo_root)
        if projects:
            cli = kicad_cli()
        for project in projects:
            fab(project, cli)
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
