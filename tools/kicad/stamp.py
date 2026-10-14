"""The check stamp: proof that KiCad read the committed carrier files and agreed.

    PYTHONPATH=tools python3 -m kicad.stamp [project ...] [--allow-none]   # CI: KiCad-free
    PYTHONPATH=tools python3 -m kicad.stamp --write <project>             # local, kicad-cli

`<carrier>/<name>/<name>.check.json` records the sha256 of the committed
`.kicad_sch` and `.kicad_pcb`, the kicad-cli version, and KiCad's own netlist
export of that schematic, normalised as `kicad.parity` does (order-free,
`unconnected-*` names included). Sorted keys, `indent=1`, a trailing newline.

`--write` is called by `kicad.check` as its last step, after ERC, DRC with
schematic parity, netlist parity and the board check all passed; nothing else
should write it. The default mode fails when the stamp is missing, when either
hash differs from the committed file, or when the stamped netlist differs from
the join's (`netlist.build`). That closes two gaps a KiCad-free CI has: a writer
bug that `schematic --check` cannot see (it compares the writer with itself),
and a hand-edited board that `kicad.boardcheck` cannot see (it reads pad nets,
not copper). It does not stop a deliberate forgery; it stops drift.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

from . import library, netlist, parity, paths
from .errors import KicadError
from .paths import REPO_ROOT

SUFFIX = ".check.json"
KEYS = {"board_sha256", "kicad_cli_version", "netlist", "schematic_sha256"}


def hint(module: str) -> str:
    return (
        f"run just {module}::carrier-check locally (KiCad 10.0.6); the stamp is written "
        "only by that recipe — do not edit it or the .kicad_pcb by hand"
    )


def files(project: Path) -> tuple[Path, Path, Path]:
    """(stamp, schematic, board) of a project."""
    cfg = paths.load_board_config(project)
    kicad_dir = paths.carrier_dir(project) / cfg.name
    return (
        kicad_dir / f"{cfg.name}{SUFFIX}",
        kicad_dir / f"{cfg.name}.kicad_sch",
        kicad_dir / f"{cfg.name}.kicad_pcb",
    )


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def render(sch: bytes, pcb: bytes, exported: dict[str, Any], version: str) -> str:
    """The stamp file's text."""
    data = {
        "board_sha256": sha256(pcb),
        "kicad_cli_version": version,
        "netlist": exported,
        "schematic_sha256": sha256(sch),
    }
    return json.dumps(data, sort_keys=True, indent=1, ensure_ascii=False) + "\n"


def compare(
    stamp: dict[str, Any], sch: bytes, pcb: bytes, want: dict[str, Any]
) -> list[str]:
    """Why a parsed stamp does not vouch for these files and this join; [] when it does."""
    missing = sorted(KEYS - set(stamp))
    if missing:
        return [f"stamp lacks {missing}"]
    errors = []
    if stamp["schematic_sha256"] != sha256(sch):
        errors.append("schematic_sha256 differs from the committed .kicad_sch")
    if stamp["board_sha256"] != sha256(pcb):
        errors.append("board_sha256 differs from the committed .kicad_pcb")
    got = stamp["netlist"]
    if not isinstance(got, dict) or set(got) != {"components", "nets", "unconnected"}:
        errors.append("netlist: expected components, nets and unconnected")
    else:
        errors += [f"netlist: {e}" for e in parity.diff(want, got, "join", "stamp")]
    return errors


def check(project: Path, repo_root: Path = REPO_ROOT) -> list[str]:
    """KiCad-free: the committed stamp against the committed files and the join."""
    vendored = repo_root / library.VENDORED.relative_to(REPO_ROOT)
    stamp_path, sch, pcb = files(project)
    if not stamp_path.is_file():
        return [f"{stamp_path.name}: missing"]
    try:
        stamp = json.loads(stamp_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        return [f"{stamp_path.name}: not JSON ({e})"]
    for p in (sch, pcb):
        if not p.is_file():
            return [f"{p.name}: missing"]
    want = netlist.as_dict(netlist.build(project, repo_root, vendored), vendored)
    return compare(stamp, sch.read_bytes(), pcb.read_bytes(), want)


def write(project: Path) -> Path:
    """Local: export the committed schematic with kicad-cli and write the stamp."""
    from .vendor import installed_version

    stamp_path, sch, pcb = files(project)
    if not pcb.is_file():
        raise KicadError(f"{pcb}: no board")
    exported = parity.normalise(parity.export(sch))
    version = installed_version() or "unknown"
    stamp_path.write_text(
        render(sch.read_bytes(), pcb.read_bytes(), exported, version), encoding="utf-8"
    )
    return stamp_path


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.stamp", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    parser.add_argument(
        "--write",
        action="store_true",
        help="local: export the schematic with kicad-cli and write the stamp "
        "(kicad.check does this after every gate passed)",
    )
    args = parser.parse_args(argv)
    failed = False
    try:
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            if args.write:
                print(f"wrote {write(project).name}")
                continue
            errors = check(project, repo_root)
            for e in errors:
                print(f"FAIL: {e}")
            name = files(project)[0].name
            if errors:
                failed = True
                print(
                    f"STAMP FAIL: {name}: {hint(paths.just_module(project, repo_root))}"
                )
            else:
                print(f"OK: {name} vouches for the committed schematic, board and join")
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
