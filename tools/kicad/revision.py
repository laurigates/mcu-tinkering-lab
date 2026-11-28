"""The board revision, and the order ledger that keeps one revision one board.

The commit hash cannot live in a committed file: a commit's hash depends on
its content, so a file naming its own commit changes the hash it names. The
hash is therefore written where it is knowable, at export (`kicad.provenance`:
`carrier-fab` stamps a plotting copy and the file names). What a commit *can*
carry is the revision a person gives the board, and that needs a rule to mean
anything: two boards that differ must never be ordered under one revision.

- `board.toml [fab] revision = N` (an integer from 1) is the one place the
  revision is set. `carrier-fab` writes it on the silkscreen as `rN` ahead of
  the commit (`rN <sha7> <pcb8>`) and into every output name
  (`<name>-rN-fab-<sha7>.zip`), so the fab cart line names both.
- A committed `[silk]` text starting `revN` or `rN` (balancebot's
  `rev3 2026-11`) must name the same N.
- `hardware/carrier/orders.toml` lists each order: revision, commit, the
  `.kicad_pcb`'s sha256, vendor and order number. Once revision N is in it,
  a committed board with another sha256 must carry a revision above every
  ordered one. Before the first order the board may change freely.

`check` runs KiCad-free in the `kicad-carrier` pre-commit hook and in
`just kicad::check`; `carrier-fab` refuses to export a board the ledger
says needs a bump. `record` appends the order from the hand-off row
`carrier-fab` wrote, after checkout:

    PYTHONPATH=tools python3 -m kicad.revision [project ...]
    PYTHONPATH=tools python3 -m kicad.revision record <project> --vendor JLCPCB --order W...
"""

from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import re
import sys
import tomllib
from pathlib import Path
from typing import Any

from . import paths
from .errors import KicadError
from .paths import REPO_ROOT

LEDGER = "orders.toml"
ORDER_KEYS = ("revision", "date", "vendor", "order", "git_sha", "pcb_sha256")
OPTIONAL_KEYS = ("zip", "zip_sha256", "silk_text", "note")
_SILK_REV = re.compile(r"(?:rev|r)(\d+)\b", re.IGNORECASE)
_HEADER = (
    "# Fabrication orders of this carrier (kicad.revision). One [[order]] per\n"
    "# order, appended by `python3 -m kicad.revision record` from carrier-fab's\n"
    "# hand-off row. Once a revision is here, a board with another pcb_sha256\n"
    "# needs a higher [fab] revision in board.toml. Do not edit past entries.\n"
)


def label(rev: int) -> str:
    """The revision as the silkscreen and the file names carry it."""
    return f"r{rev}"


def number(cfg: paths.BoardConfig) -> int:
    """board.toml's [fab] revision: an integer from 1."""
    fab = cfg.data.get("fab")
    rev = fab.get("revision") if isinstance(fab, dict) else None
    if not isinstance(rev, int) or isinstance(rev, bool) or rev < 1:
        raise KicadError(
            f"{cfg.path}: [fab] revision = <integer from 1> is required, got {rev!r}: "
            "the revision carrier-fab writes on the silkscreen and in the file names"
        )
    return rev


def silk_errors(cfg: paths.BoardConfig, rev: int) -> list[str]:
    """Committed [silk] texts that name a revision other than `rev`."""
    texts = cfg.data.get("silk", {}).get("texts", [])
    errors = []
    for t in texts if isinstance(texts, list) else []:
        text = t.get("text") if isinstance(t, dict) else None
        m = _SILK_REV.match(text) if isinstance(text, str) else None
        if m and int(m.group(1)) != rev:
            errors.append(
                f"{cfg.path}: [silk] text {text!r} names another revision than "
                f"[fab] revision = {rev}; change one of them"
            )
    return errors


def load_orders(path: Path) -> list[dict[str, Any]]:
    """The ledger's [[order]] entries, each with ORDER_KEYS; [] without a ledger."""
    if not path.is_file():
        return []
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except tomllib.TOMLDecodeError as e:
        raise KicadError(f"{path}: {e}") from None
    orders = data.get("order", [])
    if set(data) - {"order"} or not isinstance(orders, list):
        raise KicadError(f"{path}: only [[order]] entries are allowed")
    for i, o in enumerate(orders):
        missing = [k for k in ORDER_KEYS if k not in o]
        unknown = sorted(set(o) - set(ORDER_KEYS) - set(OPTIONAL_KEYS))
        if missing or unknown:
            raise KicadError(
                f"{path}: order {i + 1}: missing {missing}, unknown {unknown}"
            )
        rev = o["revision"]
        if not isinstance(rev, int) or isinstance(rev, bool) or rev < 1:
            raise KicadError(f"{path}: order {i + 1}: revision must be an integer")
    return orders


def order_errors(orders: list[dict[str, Any]], rev: int, pcb_sha256: str) -> list[str]:
    """Why the committed board may not carry `rev`, given what was ordered."""
    errors = []
    newest = max((o["revision"] for o in orders), default=0)
    for o in orders:
        if o["revision"] == rev and o["pcb_sha256"] != pcb_sha256:
            errors.append(
                f"{label(rev)} was ordered ({o['vendor']} {o['order']}, {o['date']}) as "
                f"board {o['pcb_sha256'][:8]} from {o['git_sha'][:7]}; the board is now "
                f"{pcb_sha256[:8]}: set [fab] revision = {newest + 1} in board.toml"
            )
            break
    if rev < newest:
        errors.append(
            f"[fab] revision = {rev} is below the ordered {label(newest)}; "
            f"set it to {newest if _ordered(orders, newest, pcb_sha256) else newest + 1}"
        )
    return errors


def _ordered(orders: list[dict[str, Any]], rev: int, pcb_sha256: str) -> bool:
    return any(o["revision"] == rev and o["pcb_sha256"] == pcb_sha256 for o in orders)


def _toml_value(v: Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    s = str(v).replace("\\", "\\\\").replace('"', '\\"')
    if "\n" in s:
        raise KicadError(f"ledger value {v!r}: newline")
    return f'"{s}"'


def append_order(path: Path, entry: dict[str, Any]) -> None:
    """Add one [[order]] to the ledger, creating it with its header."""
    keys = [k for k in (*ORDER_KEYS, *OPTIONAL_KEYS) if k in entry]
    block = "\n[[order]]\n" + "".join(f"{k} = {_toml_value(entry[k])}\n" for k in keys)
    text = path.read_text(encoding="utf-8") if path.is_file() else _HEADER
    path.write_text(text + block, encoding="utf-8")
    load_orders(path)  # what was written must read back


def files(project: Path) -> tuple[paths.BoardConfig, Path, Path]:
    """The board config, the committed .kicad_pcb and the ledger path."""
    cfg = paths.load_board_config(project)
    carrier = paths.carrier_dir(project)
    return cfg, carrier / cfg.name / f"{cfg.name}.kicad_pcb", carrier / LEDGER


def check(project: Path) -> list[str]:
    """Every revision error for one carrier project."""
    cfg, pcb, ledger = files(project)
    try:
        rev = number(cfg)
    except KicadError as e:
        return [str(e)]
    errors = silk_errors(cfg, rev)
    if pcb.is_file():
        sha = hashlib.sha256(pcb.read_bytes()).hexdigest()
        errors += [
            f"{ledger}: {e}" for e in order_errors(load_orders(ledger), rev, sha)
        ]
    return errors


def require_orderable(project: Path, pcb_sha256: str) -> int:
    """The revision, or KicadError when the ledger says the board needs a bump."""
    cfg, _pcb, ledger = files(project)
    rev = number(cfg)
    errors = order_errors(load_orders(ledger), rev, pcb_sha256)
    if errors:
        raise KicadError("\n".join(f"{ledger}: {e}" for e in errors))
    return rev


def handoff_rows(project: Path) -> list[dict[str, str]]:
    cfg = paths.load_board_config(project)
    out = paths.carrier_dir(project) / cfg.name / "output"
    rows = []
    for path in sorted(out.glob(f"{cfg.name}-r*-fab-*.csv")) if out.is_dir() else []:
        with open(path, newline="", encoding="utf-8") as f:
            rows += list(csv.DictReader(f))
    return rows


def record(project: Path, vendor: str, order: str, date: str) -> dict[str, Any]:
    """Append the order of the one export in output/ to the ledger."""
    cfg, pcb, ledger = files(project)
    rows = handoff_rows(project)
    if len(rows) != 1:
        raise KicadError(
            f"{project}: {len(rows)} hand-off rows in output/; run carrier-fab so "
            "output/ holds the one export that was ordered"
        )
    (row,) = rows
    if row.get("dirty") != "no":
        raise KicadError(f"{row.get('zip')}: a dirty export is never an order")
    sha = hashlib.sha256(pcb.read_bytes()).hexdigest()
    if row.get("pcb_sha256") != sha:
        raise KicadError(
            f"{row.get('zip')}: exported from board {row.get('pcb_sha256', '')[:8]}, "
            f"the committed board is {sha[:8]}; re-run carrier-fab, check the cart "
            "against the new hand-off row, then record"
        )
    rev = number(cfg)
    if str(rev) != row.get("revision"):
        raise KicadError(
            f"{row.get('zip')}: revision {row.get('revision')}, board.toml {rev}"
        )
    entry = {
        "revision": rev,
        "date": date,
        "vendor": vendor,
        "order": order,
        "git_sha": row["git_sha"],
        "pcb_sha256": sha,
        **{k: row[k] for k in ("zip", "zip_sha256", "silk_text") if row.get(k)},
    }
    append_order(ledger, entry)
    print(f"recorded {label(rev)} {row['git_sha'][:7]} {vendor} {order} in {ledger}")
    return entry


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        if argv[:1] == ["record"]:
            p = argparse.ArgumentParser(prog="kicad.revision record")
            p.add_argument("project", type=Path)
            p.add_argument("--vendor", required=True)
            p.add_argument("--order", required=True, help="the vendor's order number")
            p.add_argument("--date", default=datetime.date.today().isoformat())
            a = p.parse_args(argv[1:])
            record(a.project, a.vendor, a.order, a.date)
            return 0
        parser = argparse.ArgumentParser(
            prog="kicad.revision", description=__doc__.split("\n")[0]
        )
        paths.add_project_args(parser)
        args = parser.parse_args(argv)
        failed = False
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            if not paths.load_board_config(project).has_board:
                print(paths.skip_line(project, repo_root))
                continue
            errors = check(project)
            for e in errors:
                print(f"FAIL: {e}")
            failed |= bool(errors)
            if not errors:
                cfg = paths.load_board_config(project)
                print(f"ok: {cfg.name} {label(number(cfg))}")
        return 1 if failed else 0
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
