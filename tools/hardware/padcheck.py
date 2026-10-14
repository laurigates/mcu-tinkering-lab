"""Re-derive the pad centres a breakout page records from the vendor's board file.

    PYTHONPATH=tools python3 -m hardware.padcheck              # check every page
    PYTHONPATH=tools python3 -m hardware.padcheck --print SLUG # emit the table

`just hardware::pad-centres-check` runs the first form. It is local only: it
fetches each page's `.brd` from GitHub at the commit the page's Source section
pins, so CI stays offline and KiCad-free (ADR-027) and reads only the recorded
table, through `hardware.layout.parse_pad_centres`.

For every page with a pad-centre table, the check runs `tools/breakout-pinout.py
--json` on the fetched file for the headers the layout table cites, negates y
into KiCad's convention, and fails on any pad that moved, is missing or is
extra. `--print` writes the table from the same numbers, so a new page is
authored by the code that checks it rather than by copying coordinates out of a
terminal.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import tempfile
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import quote

from .board import _tables
from .errors import HardwareError
from .layout import (
    BOARDS_DIR,
    REPO_ROOT,
    _board_path,
    _is_centres,
    _layout_headers,
    parse_pad_centres,
)

PINOUT = REPO_ROOT / "tools/breakout-pinout.py"
TOLERANCE = 0.005  # mm: the page and breakout-pinout.py both print 0.01 mm

Centres = dict[tuple[str, str], tuple[float, float]]


@dataclass(frozen=True)
class BoardFile:
    repo: str  # owner/name on GitHub
    path: str  # the .brd within the repo
    commit: str  # the 40-hex commit the page was read at


def _one(path: Path, what: str, found: list[str]) -> str:
    unique = sorted(set(found))
    if len(unique) != 1:
        raise HardwareError(
            f"{path}: the Source section names {len(unique)} {what}; expected exactly one"
            + (f" ({', '.join(unique)})" if unique else "")
        )
    return unique[0]


def source(path: Path) -> BoardFile:
    """The vendor board file a page's `## Source` section cites."""
    text = path.read_text(encoding="utf-8")
    match = re.search(r"^## Source\s*$(.*?)(?=^## |\Z)", text, re.MULTILINE | re.DOTALL)
    if not match:
        raise HardwareError(f"{path}: no '## Source' section naming the board file")
    body = match.group(1)
    return BoardFile(
        repo=_one(
            path,
            "GitHub repos",
            re.findall(r"https://github\.com/([\w.-]+/[\w.-]+)", body),
        ),
        path=_one(path, ".brd files", re.findall(r"`([^`\n]+\.brd)`", body)),
        commit=_one(path, "commits", re.findall(r"`([0-9a-f]{40})`", body)),
    )


def raw_url(src: BoardFile) -> str:
    return (
        f"https://raw.githubusercontent.com/{src.repo}/{src.commit}/{quote(src.path)}"
    )


def _mm(value: float) -> float:
    return round(value, 2) + 0.0  # + 0.0 turns -0.0 into 0.0


def expected_centres(doc: dict[str, Any], elements: set[str]) -> Centres:
    """`breakout-pinout.py --json` output → {(element, pad): (x, y)}, y down."""
    out: Centres = {}
    present = set()
    for element in doc["elements"]:
        name = element["element"]
        if name not in elements:
            continue
        present.add(name)
        for pin in element["pins"]:
            out[name, pin["pad"]] = (_mm(pin["x"]), _mm(-pin["y"]))
    missing = sorted(elements - present)
    if missing:
        raise HardwareError(f"the board file has no element {', '.join(missing)}")
    return out


def recorded(path: Path) -> Centres:
    return {
        (c.element, c.pad): (c.x, c.y)
        for pads in parse_pad_centres(path).values()
        for c in pads
    }


def diff(have: Centres, want: Centres) -> list[str]:
    """One line per pad whose recorded centre differs from the board file's."""
    lines = []
    for key in sorted(set(have) | set(want)):
        label = ".".join(key)
        if key not in want:
            lines.append(f"{label}: recorded, but not in the board file")
        elif key not in have:
            lines.append(f"{label}: in the board file at {want[key]}, not recorded")
        elif any(abs(a - b) > TOLERANCE for a, b in zip(have[key], want[key])):
            lines.append(f"{label}: recorded {have[key]}, board file {want[key]}")
    return lines


def table(centres: Centres) -> str:
    rows = ["| Header | X (mm) | Y (mm) |", "|--------|--------|--------|"]
    rows += [f"| {e}.{p} | {x:.2f} | {y:.2f} |" for (e, p), (x, y) in centres.items()]
    return "\n".join(rows) + "\n"


def centre_pages() -> list[Path]:
    """Every board page that records pad centres."""
    return sorted(
        path
        for path in BOARDS_DIR.glob("*.md")
        if any(_is_centres(t[0]) for t in _tables(path.read_text(encoding="utf-8")))
    )


def _cited_elements(path: Path) -> set[str]:
    tables = _tables(path.read_text(encoding="utf-8"))
    return {element for element, _ in _layout_headers(path, tables)}


def derive(path: Path) -> Centres:
    """Fetch the page's board file and read its headers' centres from it."""
    src = source(path)
    elements = _cited_elements(path)
    # Downloaded data goes in its own directory; the parser runs isolated (-I)
    # from outside it, so nothing in the download can be imported.
    with tempfile.TemporaryDirectory() as tmp:
        brd = Path(tmp) / "board.brd"
        with urllib.request.urlopen(raw_url(src), timeout=60) as resp:
            brd.write_bytes(resp.read())
        args = [
            sys.executable,
            "-I",
            str(PINOUT),
            str(brd),
            "--json",
            "--min-pads",
            "1",
        ]
        for element in sorted(elements):
            args += ["--element", element]
        run = subprocess.run(args, capture_output=True, text=True, check=False)
    if run.returncode != 0:
        raise HardwareError(f"{path}: breakout-pinout.py failed: {run.stderr.strip()}")
    return expected_centres(json.loads(run.stdout), elements)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "--print", metavar="SLUG", help="print the table for one board page"
    )
    args = ap.parse_args(argv)

    try:
        if args.print:
            print(table(derive(_board_path(args.print))), end="")
            return 0
        pages = centre_pages()
        if not pages:
            print("no board page records pad centres", file=sys.stderr)
            return 1
        failed = False
        for page in pages:
            lines = diff(recorded(page), derive(page))
            rel = page.relative_to(REPO_ROOT)
            if lines:
                failed = True
                print(f"{rel}: {len(lines)} pad(s) differ from {raw_url(source(page))}")
                for line in lines:
                    print(f"  {line}")
            else:
                print(f"{rel}: {len(recorded(page))} pad centres match the board file")
    except HardwareError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
