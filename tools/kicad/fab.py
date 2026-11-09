"""Fabrication outputs for a carrier: gerbers and drill files zipped for JLCPCB,
plus the BOM and CPL for its assembly service.

    PYTHONPATH=tools python3 -m kicad.fab [project ...] [--allow-none] [--allow-dirty]

Needs KiCad 10.0.6 (`kicad-cli`, from $KICAD_CLI or the PATH). It exports what
was checked; `just <project>::carrier-fab` runs `carrier-check` first, as a
just dependency rather than from here.

Every output names the commit it came from (`kicad.provenance`): the run
refuses a tree with uncommitted changes to anything that feeds the board
(`$CARRIER_FAB_ALLOW_DIRTY=1` or `--allow-dirty` exports anyway and marks
everything `-dirty`), and the gerbers, drill files and CPL are plotted from a
temporary copy of the board that carries `<sha7> <pcb8>` on F.SilkS at
board.toml's `[fab] provenance_at`. The committed `.kicad_pcb` and its check
stamp are only read. `<sha7>` is the commit; `<pcb8>` the first 8 hex digits of
the stamp's `board_sha256`. Per project, into the KiCad project's `output/`
(git-ignored):

- `fab/`: gerbers for the copper, paste, silkscreen, mask and Edge.Cuts layers
  (`--check-zones`), and Excellon drill files with gerber X2 drill maps;
- `<name>-fab-<sha7>.zip`: `fab/`'s files, the upload for JLCPCB's PCB order;
- `assembly/bom.csv` (`sch export bom`, grouped by value, footprint and LCSC)
  and `assembly/pos.csv` (front-side SMD placements);
- `assembly/<name>-bom-jlc-<sha7>.csv` and `assembly/<name>-cpl-jlc-<sha7>.csv`
  (`kicad.jlc`): only the parts with an LCSC number, for JLCPCB assembly; skip
  them to hand-solder;
- `<name>-fab-<sha7>.csv`: the hand-off row (commit, silk text, board hash, the
  zip's sha256, the BOM and CPL), which the `jlcpcb-order` skill's hand-off CSV
  copies.

A previous run's zip and hand-off row are deleted first, so `output/` holds one
commit's files. It also prints the order options the board depends on
(`order_notes`), such as high-precision silkscreen for text under 1.0 mm.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

from . import jlc, paths, provenance, vendor
from .errors import KicadError
from .paths import REPO_ROOT

GERBER_LAYERS = (
    "F.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts"
)
# Project files the plot copy needs beside it: board setup and the custom
# rules, which `--check-zones` refills by.
PROJECT_SUFFIXES = (".kicad_pro", ".kicad_dru")


def kicad_cli() -> str:
    found = vendor.kicad_cli()
    if not found:
        raise KicadError(
            "kicad-cli not found (set KICAD_CLI): kicad.fab needs KiCad 10.0.6"
        )
    return found


def run(argv: list[str]) -> str:
    done = subprocess.run(argv, capture_output=True, text=True)
    out = (done.stdout + done.stderr).strip()
    if out:
        print(out)
    if done.returncode != 0:
        raise KicadError(f"{' '.join(argv[:4])}: exit {done.returncode}")
    return out


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


def order_notes(cfg: paths.BoardConfig) -> list[str]:
    """Order-form options the board needs that the upload itself cannot carry."""
    notes = []
    if cfg.data.get("silk", {}).get("high_precision") is True:
        notes.append(
            "select high-precision silkscreen: [silk] high_precision = true, text "
            "down to 0.8 mm, which standard silkscreen prints unreadably"
        )
    return notes


def stale_outputs(out: Path, name: str) -> list[Path]:
    """A previous run's zip and hand-off row: `<name>-fab.zip`, `<name>-fab-*.zip|csv`."""
    pattern = re.compile(rf"{re.escape(name)}-fab(-[0-9a-f]+(-dirty)?)?\.(zip|csv)")
    if not out.is_dir():
        return []
    return sorted(p for p in out.iterdir() if pattern.fullmatch(p.name))


def provenance_violations(report: str, text: str) -> list[str]:
    """DRC report items that name the provenance text (silk over a pad, another
    text, or the board edge)."""
    blocks = [b.split("\n\n", 1)[0] for b in re.split(r"\n(?=\[)", report)]
    return [b.strip() for b in blocks if b.startswith("[") and f"'{text}'" in b]


def plot_copy(
    kicad_dir: Path, name: str, board_text: str, dest: Path, cli: str, text: str
) -> Path:
    """The board with the provenance text, beside copies of its project files in
    `dest`; fails when KiCad's DRC finds the text on a pad, text or edge."""
    for suffix in PROJECT_SUFFIXES:
        src = kicad_dir / f"{name}{suffix}"
        if src.is_file():
            shutil.copy2(src, dest / src.name)
    pcb = dest / f"{name}.kicad_pcb"
    pcb.write_text(board_text, encoding="utf-8")
    report = dest / "provenance-drc.rpt"
    subprocess.run(
        [cli, "pcb", "drc", "--severity-all", "-o", str(report), str(pcb)],
        capture_output=True,
        text=True,
    )
    if not report.is_file():
        raise KicadError(f"{pcb.name}: kicad-cli pcb drc wrote no report")
    found = provenance_violations(report.read_text(encoding="utf-8"), text)
    if found:
        raise KicadError(
            "[fab] provenance_at puts the commit text where DRC objects; move it:\n"
            + "\n".join(found)
        )
    return pcb


def fab(
    project: Path, cli: str, allow_dirty: bool = False, repo_root: Path = REPO_ROOT
) -> Path:
    """Write every fab output for one project; the output directory."""
    cfg = paths.load_board_config(project)
    kicad_dir = paths.carrier_dir(project) / cfg.name
    pcb = kicad_dir / f"{cfg.name}.kicad_pcb"
    sch = kicad_dir / f"{cfg.name}.kicad_sch"
    stamp = kicad_dir / f"{cfg.name}.check.json"
    for f in (pcb, sch, stamp):
        if not f.is_file():
            raise KicadError(f"{f}: missing")
    state = provenance.git_state(project, repo_root)
    dirty = provenance.require_clean(state, allow_dirty)
    at, size = provenance.spec(cfg)
    pcb_bytes = pcb.read_bytes()
    board_sha = provenance.stamped_board_sha256(stamp, pcb_bytes)
    run_tag = provenance.tag(state.sha, dirty)
    text = provenance.silk_text(run_tag, board_sha)
    names = provenance.names(cfg.name, run_tag)
    if dirty:
        print(f"WARNING: dirty tree, every output is marked {provenance.DIRTY}:")
        for p in state.dirty:
            print(f"  {p}")
    board_text = provenance.add_text(
        pcb_bytes.decode("utf-8"), text, provenance.absolute(cfg, at), size
    )
    out = kicad_dir / "output"
    out.mkdir(exist_ok=True)
    for old in stale_outputs(out, cfg.name):
        old.unlink()
    gerbers = fresh_dir(out / "fab")
    asm = fresh_dir(out / "assembly")
    with tempfile.TemporaryDirectory(prefix="carrier-fab-") as tmp:
        plot = plot_copy(kicad_dir, cfg.name, board_text, Path(tmp), cli, text)
        print(f"silkscreen: {text!r} at {at} (board.toml [fab] provenance_at)")
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
                str(plot),
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
                str(plot),
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
                str(plot),
            ]
        )
    archive = out / names.zip
    zip_dir(gerbers, archive)
    print(f"wrote {archive}")
    for note in order_notes(cfg):
        print(f"ORDER: {note}")
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
    try:
        jlc.write(
            asm / "bom.csv",
            asm / "pos.csv",
            asm,
            bom_name=names.bom,
            cpl_name=names.cpl,
        )
    except KeyError as e:
        raise KicadError(
            f"{asm}: column {e} missing from bom.csv or pos.csv "
            "(kicad-cli's export columns changed?)"
        ) from None
    row = provenance.handoff_row(
        cfg.name,
        state,
        dirty,
        text,
        board_sha,
        archive,
        asm / names.bom,
        asm / names.cpl,
        repo_root,
    )
    provenance.write_handoff(out / names.handoff, row)
    print(f"wrote {out / names.handoff}: zip sha256 {row['zip_sha256']}")
    return out


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.fab", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="export from uncommitted changes, marking the silk text and every "
        f"file name '{provenance.DIRTY}' (also ${provenance.ALLOW_DIRTY_ENV}=1)",
    )
    args = parser.parse_args(argv)
    allow_dirty = args.allow_dirty or provenance.allow_dirty_from_env()
    try:
        projects = paths.select_projects(args.projects, args.allow_none, repo_root)
        if projects:
            cli = kicad_cli()
        for project in projects:
            fab(project, cli, allow_dirty, repo_root)
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
