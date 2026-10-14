"""The carrier schematic, written from the netlist: KiCad 10 format, deterministic.

    PYTHONPATH=tools python3 -m kicad.schematic [project ...] [--check] [--allow-none]

Writes, in `<carrier>/<name>/` (`name` from board.toml `[project]`):

- `<name>.kicad_sch`: every component of `netlist.build()` on a grid, each
  connected pin a 2.54 mm wire stub ending in a global label named for its net,
  each no-connect pin a no-connect mark, and a PWR_FLAG on every net that has a
  `power_in` pin and no `power_out` pin (ERC needs a driver);
- `sym-lib-table` and `fp-lib-table` (`kicad.libtables`): every library
  nickname the schematic uses, pointed at the vendored copy.

`--check` writes nothing and fails when any of the three differs from what it
would write, or when the hand-kept `<name>.kicad_pro` no longer names
`<name>.kicad_sch` as its top-level sheet (the KiCad GUI can rewrite that list
to `[]`).

The schematic is not hand-editable: a GUI edit is exactly what `--check`
reports. Layout is deterministic, not pretty; the readable drawing of the
circuit is the docs schematic (ADR-023). Every UUID is uuid5 under a namespace
keyed by the project's carrier directory, so a change to one part's
pins changes only that part's lines.
"""

from __future__ import annotations

import argparse
import json
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path

from . import libtables, library, netlist, paths, symbols
from .errors import KicadError
from .paths import REPO_ROOT

VERSION = "20260306"
GENERATOR = "mcu-tinkering-lab tools/kicad"
GENERATOR_VERSION = "10.0"
COMMENT = "Generated from hardware.toml by tools/kicad; do not edit"
FLAG = "power:PWR_FLAG"
FLAG_PREFIX = "#FLG"

GRID = 2.54  # symbol origins snap to this; library pins sit on 1.27 multiples
STUB = 2.54  # wire from a pin to its label
MARGIN = 20.32  # each side of a cell, for stubs and labels
ROW_GAP = 17.78
START = 25.4
RIGHT_MARGIN = 20.0  # wrap before paper width minus this
TITLE_BLOCK = 40.0  # the last row must end above paper height minus this
# Landscape sizes in mm, smallest first: a sheet that overflows moves to the next.
PAPERS = {
    "A4": (297.0, 210.0),
    "A3": (420.0, 297.0),
    "A2": (594.0, 420.0),
    "A1": (841.0, 594.0),
}

# A library pin's angle points from its connection point toward the body. The
# stub runs the other way, in schematic coordinates (y down).
OUTWARD = {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}
LABEL_ANGLE = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}


def q(s: str) -> str:
    """A KiCad string literal: `"` and `\\` escaped."""
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def fmt(v: float) -> str:
    """4-decimal fixed point, trailing zeros stripped, never `-0`."""
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def snap(v: float) -> float:
    return round(round(v / GRID) * GRID, 4)


def namespace(key: str) -> uuid.UUID:
    """The uuid5 namespace for a carrier: `key` is its repo-relative carrier dir."""
    return uuid.uuid5(uuid.NAMESPACE_URL, f"mcu-tinkering-lab/kicad/{key}")


def project_key(project: Path, repo_root: Path = REPO_ROOT) -> str:
    """The namespace key of `project`: its carrier dir, repo-relative, POSIX."""
    carrier = paths.carrier_dir(project).resolve()
    try:
        return carrier.relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        raise KicadError(f"{carrier}: not inside the repo {repo_root}") from None


@dataclass(frozen=True)
class _Item:
    """One symbol instance to place: a component or a power flag."""

    ref: str
    lib_id: str
    value: str
    footprint: str
    fields: tuple[tuple[str, str], ...]
    block: str
    pins: tuple[symbols.Pin, ...]
    extent: tuple[float, float, float, float]
    uid: str  # uuid key of the symbol; "sym/<ref>" or "flag/<net>"
    pin_key: str  # prefix of its pin/wire/label/nc keys; "<ref>" or "flag/<net>"


class _Writer:
    def __init__(self, ns: uuid.UUID, project_name: str):
        self.ns = ns
        self.project = project_name
        self.root = self.uid("sheet")

    def uid(self, key: str) -> str:
        return str(uuid.uuid5(self.ns, key))

    def prop(
        self,
        name: str,
        value: str,
        at: tuple[float, float],
        hide: bool,
        justify: str = "",
        ind: str = "\t\t",
    ) -> str:
        lines = [
            f"{ind}(property {q(name)} {q(value)}",
            f"{ind}\t(at {fmt(at[0])} {fmt(at[1])} 0)",
        ]
        if hide:
            lines.append(f"{ind}\t(hide yes)")
        lines += [
            f"{ind}\t(show_name no)",
            f"{ind}\t(do_not_autoplace no)",
            *_effects(ind + "\t", justify),
            f"{ind})",
        ]
        return "\n".join(lines)

    def symbol(self, item: _Item, at: tuple[float, float]) -> str:
        x, y = at
        x0, y0, x1, y1 = item.extent
        flag = item.ref.startswith("#")
        flags = symbols.flags(item.block)
        yn = {True: "yes", False: "no"}
        props = [
            self.prop(
                "Reference",
                item.ref,
                (x + x0, y - y1 - 1.27) if not flag else at,
                hide=flag,
                justify="" if flag else "left",
            ),
            self.prop("Value", item.value, (x + x0, y - y0 + 2.54), False, "left"),
            self.prop("Footprint", item.footprint, at, True),
            self.prop("Datasheet", "", at, True),
            self.prop("Description", "", at, True),
            *(self.prop(k, v, at, True) for k, v in item.fields),
        ]
        pins = [
            f"\t\t(pin {q(p.number)}\n"
            f'\t\t\t(uuid "{self.uid(f"pin/{item.pin_key}/{p.number}")}")\n'
            "\t\t)"
            for p in item.pins
        ]
        return "\n".join(
            [
                "\t(symbol",
                f"\t\t(lib_id {q(item.lib_id)})",
                f"\t\t(at {fmt(x)} {fmt(y)} 0)",
                "\t\t(unit 1)",
                "\t\t(body_style 1)",
                f"\t\t(exclude_from_sim {yn[flags['exclude_from_sim']]})",
                f"\t\t(in_bom {yn[flags['in_bom']]})",
                f"\t\t(on_board {yn[flags['on_board']]})",
                f"\t\t(in_pos_files {yn[flags['in_pos_files']]})",
                "\t\t(dnp no)",
                f'\t\t(uuid "{self.uid(item.uid)}")',
                *props,
                *pins,
                "\t\t(instances",
                f"\t\t\t(project {q(self.project)}",
                f'\t\t\t\t(path "/{self.root}"',
                f"\t\t\t\t\t(reference {q(item.ref)})",
                "\t\t\t\t\t(unit 1)",
                "\t\t\t\t)",
                "\t\t\t)",
                "\t\t)",
                "\t)",
            ]
        )

    def wire(self, key: str, a: tuple[float, float], b: tuple[float, float]) -> str:
        return "\n".join(
            [
                "\t(wire",
                "\t\t(pts",
                f"\t\t\t(xy {fmt(a[0])} {fmt(a[1])}) (xy {fmt(b[0])} {fmt(b[1])})",
                "\t\t)",
                "\t\t(stroke",
                "\t\t\t(width 0)",
                "\t\t\t(type solid)",
                "\t\t)",
                f'\t\t(uuid "{self.uid(key)}")',
                "\t)",
            ]
        )

    def label(self, key: str, net: str, at: tuple[float, float], angle: int) -> str:
        justify = "left" if angle in (0, 90) else "right"
        return "\n".join(
            [
                f"\t(global_label {q(net)}",
                "\t\t(shape bidirectional)",
                f"\t\t(at {fmt(at[0])} {fmt(at[1])} {angle})",
                *_effects("\t\t", justify),
                f'\t\t(uuid "{self.uid(key)}")',
                self.prop("Intersheetrefs", "${INTERSHEET_REFS}", at, True),
                "\t)",
            ]
        )

    def no_connect(self, key: str, at: tuple[float, float]) -> str:
        return "\n".join(
            [
                "\t(no_connect",
                f"\t\t(at {fmt(at[0])} {fmt(at[1])})",
                f'\t\t(uuid "{self.uid(key)}")',
                "\t)",
            ]
        )


def _effects(ind: str, justify: str = "") -> list[str]:
    lines = [
        f"{ind}(effects",
        f"{ind}\t(font",
        f"{ind}\t\t(size 1.27 1.27)",
        f"{ind}\t)",
    ]
    if justify:
        lines.append(f"{ind}\t(justify {justify})")
    lines.append(f"{ind})")
    return lines


def pin_point(at: tuple[float, float], pin: symbols.Pin) -> tuple[float, float]:
    """Where a library pin lands on the sheet: library y points up."""
    return (round(at[0] + pin.x, 4), round(at[1] - pin.y, 4))


def stub_end(at: tuple[float, float], pin: symbols.Pin) -> tuple[float, float]:
    if pin.angle not in OUTWARD:
        raise KicadError(
            f"pin {pin.number} at angle {pin.angle}: expected 0/90/180/270"
        )
    dx, dy = OUTWARD[pin.angle]
    p = pin_point(at, pin)
    return (round(p[0] + dx * STUB, 4), round(p[1] + dy * STUB, 4))


def _item(
    ref: str,
    lib_id: str,
    value: str,
    footprint: str,
    fields: tuple[tuple[str, str], ...],
    block: str,
    uid: str,
    pin_key: str,
) -> _Item:
    return _Item(
        ref=ref,
        lib_id=lib_id,
        value=value,
        footprint=footprint,
        fields=fields,
        block=block,
        pins=tuple(symbols.pins(block)),
        extent=symbols.extent(block),
        uid=uid,
        pin_key=pin_key,
    )


def _check_stacked(item: _Item, net_of: dict[tuple[str, str], str]) -> None:
    """Pins at one point must share a net; otherwise the drawing would short them."""
    at: dict[tuple[float, float], tuple[str, str]] = {}
    for p in item.pins:
        point = (p.x, p.y)
        net = net_of.get((item.ref, p.number), "")
        if point in at and at[point][1] != net:
            other = at[point][0]
            raise KicadError(
                f"{item.ref}: pins {other} and {p.number} of {item.lib_id} are stacked at "
                f"({fmt(p.x)}, {fmt(p.y)}) but on nets {at[point][1] or 'none'!r} and "
                f"{net or 'none'!r}"
            )
        at.setdefault(point, (p.number, net))


def _pack(
    rows: list[list[_Item]], paper: tuple[float, float]
) -> list[tuple[_Item, tuple[float, float]]] | None:
    """Shelf-pack each group of items, a new shelf per group; None if it overflows."""
    width, height = paper
    out = []
    y = START
    for group in rows:
        if not group:
            continue
        x, row_h = START, 0.0
        for item in group:
            x0, y0, x1, y1 = item.extent
            cell = (x1 - x0) + 2 * MARGIN
            if x + cell > width - RIGHT_MARGIN and x > START:
                x, y, row_h = START, y + row_h, 0.0
            out.append((item, (snap(x - x0 + MARGIN), snap(y + y1 + 7.62))))
            row_h = max(row_h, (y1 - y0) + ROW_GAP)
            x += cell
        y += row_h
    return out if y <= height - TITLE_BLOCK else None


def render(
    nl: netlist.Netlist,
    project_name: str,
    key: str,
    paper: str = "A3",
    vendored: Path = library.VENDORED,
) -> str:
    """The `.kicad_sch` text of `nl`; `key` is the uuid namespace key (`project_key`)."""
    return _render(nl, project_name, key, paper, vendored)[0]


def _render(
    nl: netlist.Netlist,
    project_name: str,
    key: str,
    paper: str,
    vendored: Path,
) -> tuple[str, list[str]]:
    """The schematic text and the sorted lib_ids it embeds."""
    if paper not in PAPERS:
        raise KicadError(f"paper {paper!r}: expected one of {sorted(PAPERS)}")
    w = _Writer(namespace(key), project_name)
    net_of = {node: name for name, nodes in nl.nets.items() for node in nodes}
    no_connect = set(nl.no_connect)
    blocks: dict[str, str] = {}

    def block_of(lib_id: str) -> str:
        if lib_id not in blocks:
            blocks[lib_id] = symbols.load(lib_id, vendored=vendored)
        return blocks[lib_id]

    comps = [
        _item(
            c.ref,
            c.lib_id,
            c.value,
            c.footprint,
            c.fields,
            block_of(c.lib_id),
            f"sym/{c.ref}",
            c.ref,
        )
        for c in nl.comps
    ]
    etypes: dict[str, set[str]] = {}
    for item in comps:
        _check_stacked(item, net_of)
        for p in item.pins:
            node = (item.ref, p.number)
            if node in net_of:
                etypes.setdefault(net_of[node], set()).add(p.etype)
            elif node not in no_connect:
                raise KicadError(
                    f"{item.ref}.{p.number}: on no net and not a no-connect"
                )
    flagged = sorted(
        n for n, t in etypes.items() if "power_in" in t and "power_out" not in t
    )
    flags = []
    for i, net in enumerate(flagged, 1):
        ref = f"{FLAG_PREFIX}{i:02d}"
        flags.append(
            _item(
                ref,
                FLAG,
                "PWR_FLAG",
                "",
                (),
                block_of(FLAG),
                f"flag/{net}",
                f"flag/{net}",
            )
        )
        for p in flags[-1].pins:
            net_of[(ref, p.number)] = net

    order = list(PAPERS)
    placed = None
    for size in order[order.index(paper) :]:
        placed = _pack([comps, flags], PAPERS[size])
        if placed is not None:
            paper = size
            break
    if placed is None:
        raise KicadError(
            f"{project_name}: {len(comps)} symbols do not fit on {order[-1]}"
        )

    body = []
    for item, at in placed:
        body.append(w.symbol(item, at))
        drawn: set[tuple[float, float]] = set()
        for p in item.pins:
            point = pin_point(at, p)
            if point in drawn:
                continue  # a stacked pin: one stub per point
            drawn.add(point)
            k = f"{item.pin_key}/{p.number}"
            net = net_of.get((item.ref, p.number))
            if net is None:
                body.append(w.no_connect(f"nc/{k}", point))
                continue
            end = stub_end(at, p)
            body.append(w.wire(f"wire/{k}", point, end))
            body.append(w.label(f"label/{k}", net, end, LABEL_ANGLE[OUTWARD[p.angle]]))

    used = sorted({i.lib_id for i, _ in placed})
    lib_symbols = [symbols.embed(blocks[lib_id], lib_id) for lib_id in used]
    text = "\n".join(
        [
            "(kicad_sch",
            f"\t(version {VERSION})",
            f"\t(generator {q(GENERATOR)})",
            f"\t(generator_version {q(GENERATOR_VERSION)})",
            f'\t(uuid "{w.root}")',
            f"\t(paper {q(paper)})",
            "\t(title_block",
            f"\t\t(title {q(project_name)})",
            f"\t\t(comment 1 {q(COMMENT)})",
            "\t)",
            "\t(lib_symbols",
            *lib_symbols,
            "\t)",
            *body,
            "\t(sheet_instances",
            '\t\t(path "/"',
            '\t\t\t(page "1")',
            "\t\t)",
            "\t)",
            "\t(embedded_fonts no)",
            ")",
            "",
        ]
    )
    return text, used


def outputs(project: Path, repo_root: Path = REPO_ROOT) -> dict[Path, str]:
    """Each file `kicad.schematic` writes for `project` -> its content."""
    vendored = repo_root / library.VENDORED.relative_to(REPO_ROOT)
    cfg = paths.load_board_config(project)
    kicad_dir = paths.carrier_dir(project) / cfg.name
    nl = netlist.build(project, repo_root, vendored)
    sch, used = _render(
        nl, cfg.name, project_key(project, repo_root), cfg.paper, vendored
    )
    return {
        kicad_dir / f"{cfg.name}.kicad_sch": sch,
        kicad_dir / "sym-lib-table": libtables.render_sym_lib_table(
            kicad_dir, used, vendored
        ),
        kicad_dir / "fp-lib-table": libtables.render_fp_lib_table(
            kicad_dir, sorted({c.footprint for c in nl.comps}), vendored
        ),
    }


def check_project_file(project: Path) -> list[str]:
    """The hand-kept `.kicad_pro` must exist and name the schematic as its top sheet."""
    cfg = paths.load_board_config(project)
    pro = paths.carrier_dir(project) / cfg.name / f"{cfg.name}.kicad_pro"
    want = f"{cfg.name}.kicad_sch"
    try:
        data = json.loads(pro.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return [f"{pro}: missing; it is hand-kept (DRC/ERC severities, netclasses)"]
    except json.JSONDecodeError as e:
        return [f"{pro}: not JSON ({e})"]
    sheets = (data.get("schematic") or {}).get("top_level_sheets") or []
    got = sheets[0].get("filename") if sheets and isinstance(sheets[0], dict) else None
    if got != want:
        return [
            f"{pro}: schematic.top_level_sheets[0].filename is {got!r}, expected {want!r} "
            "(the KiCad GUI rewrites the list to []; restore it)"
        ]
    return []


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.schematic", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    parser.add_argument(
        "--check", action="store_true", help="write nothing; fail if stale"
    )
    args = parser.parse_args(argv)
    failed = False
    try:
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            files = outputs(project, repo_root)
            stale = [
                p
                for p, text in files.items()
                if not p.is_file() or p.read_text(encoding="utf-8") != text
            ]
            shown = [_rel(p, repo_root) for p in stale]
            if args.check:
                errors = check_project_file(project)
                for e in errors:
                    print(f"FAIL: {e}")
                if stale:
                    print(
                        f"FAIL: stale: {shown}; run `just {paths.just_module(project, repo_root)}::carrier-schematic` "
                        "(the schematic is generated from hardware.toml; do not edit it)"
                    )
                if errors or stale:
                    failed = True
                else:
                    print(
                        f"OK: {_rel(paths.carrier_dir(project), repo_root)}: up to date"
                    )
                continue
            for p in stale:
                p.write_text(files[p], encoding="utf-8")
            print(f"wrote {shown}" if stale else "up to date")
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 1 if failed else 0


def _rel(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path)


if __name__ == "__main__":
    sys.exit(main())
