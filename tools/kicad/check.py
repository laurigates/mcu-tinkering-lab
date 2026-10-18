"""The local carrier gates, in order, ending in the check stamp.

    PYTHONPATH=tools python3 -m kicad.check [project ...] [--allow-none]

Needs KiCad 10.0.6 (`kicad-cli`, from $KICAD_CLI or the PATH). Per project, it
stops at the first failure and names the report:

1. `kicad.schematic --check`: the schematic and library tables are current;
2. `kicad.vendor --check` (MANIFEST.tsv hashes), then `--check-installed` as a
   warning only: drift from the installed KiCad is for review, not an error;
3. ERC, `--severity-all --exit-code-violations`;
4. DRC with `--schematic-parity`, guarded: KiCad 10.0.6 exits 0 and reports 0
   parity issues when it cannot load the schematic, saying so only on stdout
   (`Failed to fetch schematic netlist`). So the schematic must exist beside the
   board, that line must be absent, and the run must report
   `Found 0 schematic parity issues`;
5. `kicad.parity`: KiCad's netlist of the schematic equals the join's;
6. `kicad.boardcheck`: board pads, values and paths equal the join's;
7. `kicad.stamp --write`: the stamp CI verifies.

A carrier at `stage = "schematic"` (board.toml) has no board yet: it stops
after step 5, with no DRC, board check or stamp.

Reports go to `<carrier>/<name>/output/` (git-ignored).
"""

from __future__ import annotations

import argparse
import contextlib
import io
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path

from . import boardcheck, parity, paths, schematic, stamp, vendor
from .errors import KicadError
from .paths import REPO_ROOT

FETCH_FAILED = "Failed to fetch schematic netlist"
PARITY_OK = "Found 0 schematic parity issues"


def kicad_cli() -> str:
    found = vendor.kicad_cli()
    if not found:
        raise KicadError(
            "kicad-cli not found (set KICAD_CLI): kicad.check needs KiCad 10.0.6"
        )
    return found


def run_cli(argv: list[str]) -> tuple[int, str]:
    done = subprocess.run(argv, capture_output=True, text=True)
    return done.returncode, done.stdout + done.stderr


def erc(cli: str, sch: Path, report: Path) -> list[str]:
    code, out = run_cli(
        [
            cli,
            "sch",
            "erc",
            "--severity-all",
            "--exit-code-violations",
            "-o",
            str(report),
            str(sch),
        ]
    )
    if code != 0:
        return [f"ERC exit {code}: see {report}\n{out.strip()}"]
    return []


def drc(cli: str, pcb: Path, sch: Path, report: Path) -> list[str]:
    """DRC with schematic parity, failing on the silent no-schematic pass."""
    if not sch.is_file():
        return [
            f"DRC: no schematic {sch.name} beside {pcb.name}; parity would silently pass"
        ]
    code, out = run_cli(
        [
            cli,
            "pcb",
            "drc",
            "--severity-all",
            "--schematic-parity",
            "--exit-code-violations",
            "-o",
            str(report),
            str(pcb),
        ]
    )
    errors = []
    if FETCH_FAILED in out:
        errors.append(
            f"DRC: {FETCH_FAILED!r}: KiCad did not load the schematic for parity"
        )
    if code != 0:
        errors.append(f"DRC exit {code}: see {report}")
    if PARITY_OK not in out:
        errors.append(f"DRC: output lacks {PARITY_OK!r}")
    if errors:
        errors.append(out.strip())
    return errors


def _step(name: str, fn: Callable[[], int]) -> bool:
    """Run an in-process CLI, echoing its output; True when it passed."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = fn()
    text = out.getvalue().rstrip()
    print(f"== {name}: {'ok' if code == 0 else 'FAIL'}")
    if text:
        print("   " + text.replace("\n", "\n   "))
    return code == 0


def check_project(project: Path, repo_root: Path = REPO_ROOT) -> bool:
    """Every local gate for one project; True when all passed (and, with a board, the stamp is written)."""
    arg = [str(project)]
    cfg = paths.load_board_config(project)
    kicad_dir = paths.carrier_dir(project) / cfg.name
    sch = kicad_dir / f"{cfg.name}.kicad_sch"
    pcb = kicad_dir / f"{cfg.name}.kicad_pcb"
    reports = kicad_dir / "output"
    if not _step(
        "schematic --check", lambda: schematic.main([*arg, "--check"], repo_root)
    ):
        return False
    if not _step("vendor --check", lambda: vendor.main(["--check"], repo_root)):
        return False
    drift = vendor.check_installed(repo_root)
    print(
        f"== vendor --check-installed: {'warning' if drift else 'ok'} (informational)"
    )
    for d in drift:
        print(f"   WARNING: {d}")
    cli = kicad_cli()
    reports.mkdir(parents=True, exist_ok=True)
    gates = [("ERC", lambda: erc(cli, sch, reports / "erc.rpt"), reports / "erc.rpt")]
    if cfg.has_board:
        gates.append(
            (
                "DRC --schematic-parity",
                lambda: drc(cli, pcb, sch, reports / "drc.rpt"),
                reports / "drc.rpt",
            )
        )
    for name, errors, report in gates:
        found = errors()
        print(f"== {name}: {'FAIL' if found else 'ok'} ({report})")
        for e in found:
            print("   " + e.replace("\n", "\n   "))
        if found:
            return False
    steps = [("parity", lambda: parity.main(arg, repo_root))]
    if cfg.has_board:
        steps += [
            ("boardcheck", lambda: boardcheck.main(arg, repo_root)),
            ("stamp --write", lambda: stamp.main([*arg, "--write"], repo_root)),
        ]
    for name, fn in steps:
        if not _step(name, fn):
            return False
    if not cfg.has_board:
        print(paths.skip_line(project, repo_root) + ": no DRC, board check or stamp")
    return True


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.check", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    args = parser.parse_args(argv)
    try:
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            if not check_project(project, repo_root):
                print(f"FAIL: {project}: carrier check stopped at the step above")
                return 1
            if paths.load_board_config(project).has_board:
                print(f"OK: {project}: every carrier gate passed; stamp written")
            else:
                print(f"OK: {project}: every schematic gate passed")
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
