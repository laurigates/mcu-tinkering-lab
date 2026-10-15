"""Generate socket footprints for breakout boards from their recorded pad centres.

    PYTHONPATH=tools python3 -m kicad.sockets           # regenerate
    PYTHONPATH=tools python3 -m kicad.sockets --check   # CI: fail if stale

A carrier takes a breakout (TB6612FNG, MAX98357A, ...) in female headers
placed where the breakout's own header pads are. Those positions are board
facts, recorded once in the breakout's page under `docs/reference/boards/`
(`## Pad centres`, read by `hardware.board_pad_centres` and re-derived from
the vendor's board file by `just hardware::pad-centres-check`). This module
turns them into footprints, so the carrier's sockets come from the same
numbers as the wiring docs and nobody measures a breakout twice.

Each `[[socket]]` table of `tools/kicad/lib/breakout-sockets/sockets.toml`
names the footprint, the board page, and the pads to socket in footprint pad
order: an element (`"JP1"`) is all of its pads in pad-number order, an
`element.pad` (`"JP6.10"`) is that one pad. Footprint pad n is the n-th pad
listed. Pad 1 is the footprint origin. The page's coordinates are already in
KiCad's y-down convention (`layout.py`: Eagle's y negated), so a pad's
position is its page centre minus pad 1's, with no further flip.

Pad 1 copies the vendored `PinSocket_1x08_P2.54mm_Vertical` pad 1 (rect),
every other pad its pad 2 (circle): the same copper and drill as a stock
2.54 mm socket strip. The F.CrtYd and F.SilkS rectangles enclose the pad
centres at PinSocket's own margins (COURTYARD_MARGIN, SILK_MARGIN).

The footprints are written to `Breakout_Sockets.pretty/` with no other
input, so `--check` regenerates in memory and byte-compares, writing nothing.
Regenerating also deletes a footprint the config no longer names and rewrites
the subtree's MANIFEST.tsv (`kicad.vendor`), which `vendor --check` pins.
"""

from __future__ import annotations

import argparse
import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

from hardware import HardwareError, parse_pad_centres

from . import library, sexpr, vendor
from .errors import KicadError
from .paths import REPO_ROOT

NICK = "Breakout_Sockets"
CONFIG = "sockets.toml"
BOARDS_REL = Path("docs/reference/boards")
TEMPLATE = "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical"
# PinSocket_1x08's F.CrtYd and F.SilkS sit this far outside its pad centres
# (test_sockets checks the courtyard one against the vendored footprint).
COURTYARD_MARGIN = 1.77
SILK_MARGIN = 1.33
SOCKET_KEYS = frozenset({"name", "board", "pads"})
_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
_PAD = re.compile(r"[A-Za-z0-9_]+(?:\.[A-Za-z0-9_]+)?")


@dataclass(frozen=True)
class Socket:
    """One `[[socket]]` table: footprint `name` for board page `board`."""

    name: str
    board: str
    pads: tuple[str, ...]


def load_config(path: Path) -> list[Socket]:
    """The `[[socket]]` tables of a sockets.toml, validated."""
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise KicadError(f"{path}: missing") from None
    except tomllib.TOMLDecodeError as e:
        raise KicadError(f"{path}: {e}") from None
    unknown = sorted(set(data) - {"socket"})
    tables = data.get("socket")
    if unknown or not isinstance(tables, list) or not tables:
        raise KicadError(f"{path}: expected only [[socket]] tables, at least one")
    out = []
    for n, t in enumerate(tables, 1):
        where = f"{path}: [[socket]] #{n}"
        unknown = sorted(set(t) - SOCKET_KEYS)
        missing = sorted(SOCKET_KEYS - set(t))
        if unknown or missing:
            raise KicadError(
                f"{where}: unknown key(s) {unknown}, missing key(s) {missing}; "
                f"expected {sorted(SOCKET_KEYS)}"
            )
        for key in ("name", "board"):
            if not isinstance(t[key], str) or not _NAME.fullmatch(t[key]):
                raise KicadError(f"{where}: {key} must be a name, got {t[key]!r}")
        pads = t["pads"]
        if (
            not isinstance(pads, list)
            or not pads
            or not all(isinstance(p, str) and _PAD.fullmatch(p) for p in pads)
        ):
            raise KicadError(
                f"{where}: pads must be a non-empty list of 'ELEMENT' or "
                f"'ELEMENT.PAD', got {pads!r}"
            )
        out.append(Socket(t["name"], t["board"], tuple(pads)))
    names = [s.name for s in out]
    dupes = sorted({x for x in names if names.count(x) > 1})
    if dupes:
        raise KicadError(f"{path}: name {dupes} listed more than once")
    return out


def _pad_key(pad: str) -> tuple[int, int | str]:
    return (0, int(pad)) if pad.isdigit() else (1, pad)


def resolve(socket: Socket, page: Path) -> list[tuple[str, float, float]]:
    """(element.pad, x, y) of each socketed pad, in footprint pad order."""
    try:
        centres = parse_pad_centres(page)
    except (HardwareError, OSError) as e:
        raise KicadError(f"{socket.name}: {e}") from None
    out: list[tuple[str, float, float]] = []
    for entry in socket.pads:
        element, _, pad = entry.partition(".")
        if element not in centres:
            raise KicadError(
                f"{socket.name}: {entry}: {page} records no centres for element "
                f"{element!r} (it has {sorted(centres)})"
            )
        found = sorted(centres[element], key=lambda c: _pad_key(c.pad))
        if pad:
            found = [c for c in found if c.pad == pad]
            if not found:
                raise KicadError(
                    f"{socket.name}: {entry}: {page} records no centre for that pad"
                )
        out += [(f"{c.element}.{c.pad}", c.x, c.y) for c in found]
    labels = [label for label, _, _ in out]
    dupes = sorted({x for x in labels if labels.count(x) > 1})
    if dupes:
        raise KicadError(f"{socket.name}: {', '.join(dupes)} listed twice")
    return out


def fmt(v: float) -> str:
    """A KiCad coordinate: at most 4 decimals, no trailing zeros, no "-0"."""
    s = f"{round(v, 4):.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def template_pads(text: str, where: object) -> tuple[str, str]:
    """PinSocket's pad 1 and pad 2 blocks, as written."""
    body = sexpr.inner(text)
    blocks = {}
    for a, b in sexpr.spans(body):
        kw, number = sexpr.head(body[a:b])
        if kw == "pad" and number in ("1", "2"):
            blocks.setdefault(number, body[a:b])
    if set(blocks) != {"1", "2"}:
        raise KicadError(f"{where}: expected pads 1 and 2 to copy")
    return blocks["1"], blocks["2"]


def _pad(block: str, number: int, x: float, y: float) -> str:
    block = re.sub(r'^\(pad "\d+"', f'(pad "{number}"', block, count=1)
    return re.sub(r"\(at [^)]*\)", f"(at {fmt(x)} {fmt(y)})", block, count=1)


def _rect(x0: float, y0: float, x1: float, y1: float, width: str, layer: str) -> str:
    return (
        f"(fp_rect\n\t\t(start {fmt(x0)} {fmt(y0)})\n\t\t(end {fmt(x1)} {fmt(y1)})\n"
        f"\t\t(stroke\n\t\t\t(width {width})\n\t\t\t(type solid)\n\t\t)\n"
        f'\t\t(fill no)\n\t\t(layer "{layer}")\n\t)'
    )


def _property(name: str, value: str, x: float, y: float, layer: str) -> str:
    return (
        f'(property "{name}" "{value}"\n\t\t(at {fmt(x)} {fmt(y)} 0)\n'
        f'\t\t(layer "{layer}")\n\t\t(effects\n\t\t\t(font\n'
        f"\t\t\t\t(size 1 1)\n\t\t\t\t(thickness 0.15)\n\t\t\t)\n\t\t)\n\t)"
    )


def render_footprint(
    socket: Socket, pads: list[tuple[str, float, float]], template: str
) -> str:
    """The `.kicad_mod` text for `socket`, its pads from `resolve`."""
    rect_pad, circle_pad = template_pads(template, TEMPLATE)
    _, x0, y0 = pads[0]
    rel = [(label, x - x0, y - y0) for label, x, y in pads]
    xs = [x for _, x, _ in rel]
    ys = [y for _, _, y in rel]
    left, top, right, bottom = min(xs), min(ys), max(xs), max(ys)
    order = ", ".join(f"{n}={label}" for n, (label, _, _) in enumerate(rel, 1))
    items = [
        "(version 20260206)",
        '(generator "kicad.sockets")',
        '(layer "F.Cu")',
        f'(descr "Socket for the {socket.board} breakout '
        f'(docs/reference/boards/{socket.board}.md), pads: {order}")',
        '(tags "socket breakout 2.54mm")',
        _property("Reference", "REF**", left, top - COURTYARD_MARGIN - 1, "F.SilkS"),
        _property("Value", socket.name, left, bottom + COURTYARD_MARGIN + 1, "F.Fab"),
        "(attr through_hole)",
        "(duplicate_pad_numbers_are_jumpers no)",
        _rect(
            left - SILK_MARGIN,
            top - SILK_MARGIN,
            right + SILK_MARGIN,
            bottom + SILK_MARGIN,
            "0.12",
            "F.SilkS",
        ),
        _rect(
            left - COURTYARD_MARGIN,
            top - COURTYARD_MARGIN,
            right + COURTYARD_MARGIN,
            bottom + COURTYARD_MARGIN,
            "0.05",
            "F.CrtYd",
        ),
    ]
    for n, (_, x, y) in enumerate(rel, 1):
        items.append(_pad(rect_pad if n == 1 else circle_pad, n, x, y))
    items.append("(embedded_fonts no)")
    return f'(footprint "{socket.name}"\n\t' + "\n\t".join(items) + "\n)\n"


def subtree(repo_root: Path = REPO_ROOT) -> Path:
    return repo_root / vendor.SOCKETS_REL


def pretty(repo_root: Path = REPO_ROOT) -> Path:
    return subtree(repo_root) / f"{NICK}.pretty"


def render(repo_root: Path = REPO_ROOT) -> dict[Path, str]:
    """Output path -> footprint text, for every socket in the config."""
    vendored = repo_root / library.VENDORED.relative_to(REPO_ROOT)
    template = library.footprint_file(TEMPLATE, vendored=vendored).read_text(
        encoding="utf-8"
    )
    out = {}
    for s in load_config(subtree(repo_root) / CONFIG):
        page = repo_root / BOARDS_REL / f"{s.board}.md"
        if not page.is_file():
            raise KicadError(f"{s.name}: no board page {page}")
        out[pretty(repo_root) / f"{s.name}.kicad_mod"] = render_footprint(
            s, resolve(s, page), template
        )
    return out


def _rel(path: Path, repo_root: Path) -> str:
    try:
        return path.relative_to(repo_root).as_posix()
    except ValueError:
        return str(path)


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.sockets", description=__doc__.split("\n")[0]
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if a footprint differs from its regeneration; write nothing",
    )
    args = parser.parse_args(argv)
    regen = "PYTHONPATH=tools python3 -m kicad.sockets"
    try:
        rendered = render(repo_root)
        failed = False
        for path, text in rendered.items():
            rel = _rel(path, repo_root)
            current = path.read_text(encoding="utf-8") if path.is_file() else None
            if current == text:
                print(f"ok    {rel}")
            elif args.check:
                print(f"FAIL: {rel}: stale; run `{regen}`")
                failed = True
            else:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text, encoding="utf-8")
                print(f"wrote {rel}")
        for path in sorted(pretty(repo_root).glob("*.kicad_mod")):
            if path in rendered:
                continue
            rel = _rel(path, repo_root)
            if args.check:
                print(f"FAIL: {rel}: not generated by {CONFIG}; run `{regen}`")
                failed = True
            else:
                path.unlink()
                print(f"removed {rel}")
        if not args.check:
            print(vendor.write_manifest(repo_root, vendor.SOCKETS_TREE))
        return 1 if failed else 0
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
