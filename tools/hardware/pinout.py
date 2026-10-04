"""Physical pinout images, drawn from the board references (#629).

Each board a project's build guide shows is rendered to an SVG of its outline
with every header pad on the edge it sits on, in the order the vendor's board
file gives (`.claude/rules/board-layout-from-vendor-files.md`). Nothing about a
pad is typed here: the pads come from `hardware.layout` — the same
`ModuleLayout` the physical schematic symbols read (#495) — and the labels
beside them come from the join:

  * the MCU board (`[source] board` in hardware.toml) — each pad with a GPIO
    is labelled with that GPIO, the pin role `main/pin_config.h` gives it, and
    the part pin hardware.toml wires it to;
  * every `[parts.<id>]` with a `board` key — each pad a net lands on is
    labelled with the MCU pad and role driving it, or with the PWM-driver
    channel and role for a `[[channel_nets]]` entry (#666); a driver's own
    channel pads are labelled with the role and the pin each one drives.

A net naming a pin the board reference does not have is an error, so the
picture cannot quietly disagree with hardware.toml.

The images are not to scale. Pad order and edge are exact; the outline is
sized to fit the pads and their names. Which way up the board is held — USB-C
at the top for the XIAO, the board file's own +y for a breakout — is stated on
each board's reference page and in the build guide's caption.

Usage, from the repo root:

    PYTHONPATH=tools python3 -m hardware.pinout [--check] [<project_dir> ...]

Images go to `<project>/docs/auto/pinouts/<board-slug>.svg`; any other `.svg`
in that directory is removed, so a board taken out of hardware.toml does not
leave a stale picture behind. `--check` writes nothing and exits 1 if any image
is stale, missing or orphaned. With no project, every git-tracked
`packages/**/hardware.toml` is processed.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from xml.sax.saxutils import escape

from .docs import _tracked_projects
from .errors import HardwareError
from .layout import ModuleLayout, Pad, parse_layout
from .model import REPO_ROOT, HardwareModel, join

OUT_DIR = Path("docs/auto/pinouts")

# Geometry, in SVG user units. DejaVu Sans Mono is one of Typst's bundled fonts,
# so `--ignore-system-fonts` still resolves it; being monospaced, a string's
# width is its length times one advance, which is all the layout below needs.
FONT = "DejaVu Sans Mono"
ADVANCE = 0.602  # DejaVu Sans Mono advance width, in em
PITCH = 22.0  # centre-to-centre spacing of pads along an edge
PAD_R = 5.5
NAME_PT = 10.0  # pad name, printed inside the outline
LABEL_PT = 9.0  # role label, printed outside the outline
TITLE_PT = 12.0
NOTE_PT = 8.0
GAP = 6.0  # pad edge to its name, outline to a label
MARGIN = 12.0
# Printed size of one user unit. The width/height attributes carry it, so a
# document that places the image at its natural size prints every board's text
# at the same size (a 10-unit pad name is 2.5 mm, about 7 pt).
MM_PER_UNIT = 0.25

INK = "#1f2933"
BOARD_FILL = "#eef4ee"
PAD_FILL = "#d4a72c"  # a pad nothing is wired to
WIRED_FILL = "#2f6fb0"  # a pad a label names
MUTED = "#5b6770"


def _width(text: str, pt: float) -> float:
    return len(text) * pt * ADVANCE


def _longest(texts: list[str], pt: float) -> float:
    return max((_width(t, pt) for t in texts), default=0.0)


def _title(path: Path) -> str:
    """The reference page's own heading, so the image names the board it read."""
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("# "):
            return line[2:].strip()
    raise HardwareError(f"{path}: no '# ' heading to title the pinout with")


@dataclass(frozen=True)
class Pinout:
    """One board to draw: its layout, its heading, and a label per pad anchor."""

    slug: str
    title: str
    layout: ModuleLayout
    labels: Mapping[str, str]


def render_svg(pinout: Pinout, notes: tuple[str, ...] = ()) -> str:
    """The pinout as a self-contained SVG document (deterministic text).

    `notes` are printed as small lines under the board — where the layout came
    from, and how to read the picture.
    """
    layout = pinout.layout
    unknown = sorted(set(pinout.labels) - set(layout.by_anchor))
    if unknown:
        raise HardwareError(
            f"{layout.path}: label(s) for {unknown}, which are not pads of this board"
        )

    sides = {s: layout.side(s) for s in ("L", "R", "T", "B")}
    names = {s: [p.name for p in pads] for s, pads in sides.items()}
    labels = {
        s: [pinout.labels.get(p.anchor, "") for p in pads] for s, pads in sides.items()
    }

    def zone(side: str, pt: float, texts: dict[str, list[str]]) -> float:
        """Depth a side's text needs, measured perpendicular to its edge."""
        if not sides[side]:
            return 0.0
        depth = _longest(texts[side], pt)
        return depth + GAP if depth else 0.0

    # Inside the outline: a pad row, then the names, on every populated edge.
    inset = {s: (PITCH + zone(s, NAME_PT, names)) if sides[s] else 0.0 for s in sides}
    core_w = max(len(sides["T"]), len(sides["B"]), 1) * PITCH
    core_h = max(len(sides["L"]), len(sides["R"]), 1) * PITCH
    board_w = MARGIN * 2 + inset["L"] + core_w + inset["R"]
    board_h = MARGIN * 2 + inset["T"] + core_h + inset["B"]

    # Outside the outline: the labels, rotated on the top and bottom edges.
    out = {s: zone(s, LABEL_PT, labels) for s in sides}
    title_h = TITLE_PT * 1.6
    line_h = NOTE_PT * 1.4
    board_x = MARGIN + out["L"]
    board_y = MARGIN + title_h + out["T"]
    notes_y = board_y + board_h + out["B"] + GAP
    width = max(
        board_x + board_w + out["R"] + MARGIN,
        MARGIN * 2 + _width(pinout.title, TITLE_PT),
        MARGIN * 2 + _longest(list(notes), NOTE_PT),
    )
    height = notes_y + line_h * len(notes) + MARGIN

    core_x = board_x + MARGIN + inset["L"]
    core_y = board_y + MARGIN + inset["T"]

    def along(n: int, i: int, start: float, span: float) -> float:
        """Centre of pad i (0-based) of n, centred within [start, start+span]."""
        return start + (span - n * PITCH) / 2 + PITCH * (i + 0.5)

    def pad_xy(pad: Pad, n: int, i: int) -> tuple[float, float]:
        if pad.side == "L":
            return board_x + PITCH / 2, along(n, i, core_y, core_h)
        if pad.side == "R":
            return board_x + board_w - PITCH / 2, along(n, i, core_y, core_h)
        if pad.side == "T":
            return along(n, i, core_x, core_w), board_y + PITCH / 2
        return along(n, i, core_x, core_w), board_y + board_h - PITCH / 2

    def f(v: float) -> str:
        return f"{v:.1f}"

    def text(
        x: float,
        y: float,
        body: str,
        pt: float,
        anchor: str,
        fill: str = INK,
        rotate: bool = False,
    ) -> str:
        transform = f' transform="rotate(-90 {f(x)} {f(y)})"' if rotate else ""
        return (
            f'<text x="{f(x)}" y="{f(y)}" font-size="{pt:g}" text-anchor="{anchor}" '
            f'dominant-baseline="central" fill="{fill}"{transform}>{escape(body)}</text>'
        )

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width * MM_PER_UNIT:.2f}mm" '
        f'height="{height * MM_PER_UNIT:.2f}mm" '
        f'viewBox="0 0 {f(width)} {f(height)}" font-family="{FONT}">',
        f"<title>{escape(pinout.title)}</title>",
        f'<rect x="0" y="0" width="{f(width)}" height="{f(height)}" fill="#ffffff"/>',
        text(MARGIN, MARGIN + TITLE_PT * 0.6, pinout.title, TITLE_PT, "start"),
        f'<rect x="{f(board_x)}" y="{f(board_y)}" width="{f(board_w)}" height="{f(board_h)}" '
        f'rx="6" fill="{BOARD_FILL}" stroke="{INK}" stroke-width="1.5"/>',
    ]

    for side, pads in sides.items():
        n = len(pads)
        for i, pad in enumerate(pads):
            x, y = pad_xy(pad, n, i)
            label = pinout.labels.get(pad.anchor, "")
            fill = WIRED_FILL if label else PAD_FILL
            parts.append(
                f'<circle cx="{f(x)}" cy="{f(y)}" r="{PAD_R:g}" fill="{fill}" stroke="{INK}" '
                f'stroke-width="1" data-anchor="{escape(pad.anchor, {chr(34): "&quot;"})}" '
                f'data-side="{side}" data-pos="{pad.pos}"/>'
            )
            inner = PAD_R + GAP
            outer = GAP
            if side == "L":
                parts.append(text(x + inner, y, pad.name, NAME_PT, "start"))
                if label:
                    parts.append(text(board_x - outer, y, label, LABEL_PT, "end"))
            elif side == "R":
                parts.append(text(x - inner, y, pad.name, NAME_PT, "end"))
                if label:
                    parts.append(
                        text(board_x + board_w + outer, y, label, LABEL_PT, "start")
                    )
            elif side == "T":
                parts.append(text(x, y + inner, pad.name, NAME_PT, "end", rotate=True))
                if label:
                    parts.append(
                        text(x, board_y - outer, label, LABEL_PT, "start", rotate=True)
                    )
            else:
                parts.append(
                    text(x, y - inner, pad.name, NAME_PT, "start", rotate=True)
                )
                if label:
                    parts.append(
                        text(
                            x,
                            board_y + board_h + outer,
                            label,
                            LABEL_PT,
                            "end",
                            rotate=True,
                        )
                    )

    for i, note in enumerate(notes):
        y = notes_y + line_h * (i + 0.5)
        parts.append(text(MARGIN, y, note, NOTE_PT, "start", fill=MUTED))
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def _role_name(role: str) -> str:
    return role.removesuffix("_PIN").removesuffix("_CHANNEL")


def mcu_labels(model: HardwareModel, layout: ModuleLayout) -> dict[str, str]:
    """Each GPIO pad of the MCU board: its GPIO, its role and where it goes."""
    roles_by_gpio: dict[int, list[str]] = {}
    for role, gpio in model.roles.items():
        roles_by_gpio.setdefault(gpio, []).append(role)
    undrawn = {u.role for u in model.undrawn}

    labels: dict[str, str] = {}
    for pad in layout.pads:
        if pad.gpio is None:
            continue
        bits = []
        for role in roles_by_gpio.get(pad.gpio, []):
            nets = [n for n in model.nets if n.role == role]
            ends = ", ".join(f"{model.parts[n.part].name} {n.pin}" for n in nets)
            if ends:
                bits.append(f"{_role_name(role)} → {ends}")
            elif role in undrawn:
                bits.append(f"{_role_name(role)} (not wired)")
            else:
                bits.append(_role_name(role))
        labels[pad.anchor] = " · ".join([f"GPIO{pad.gpio}", *bits])
    return labels


def part_labels(
    model: HardwareModel, part: str, layout: ModuleLayout
) -> dict[str, str]:
    """Each pad of one part that a net lands on or starts from.

    A pad driven from the MCU names the MCU pad and role; one driven from a PWM
    driver names the driver's channel and role; a driver's own channel pad
    names the role and every pin it reaches.
    """
    by_name: dict[str, list[Pad]] = {}
    for pad in layout.pads:
        by_name.setdefault(pad.name, []).append(pad)

    def anchor(role: str, pin: str) -> str:
        pads = by_name.get(pin, [])
        if len(pads) != 1:
            what = "no pad" if not pads else f"{len(pads)} pads"
            raise HardwareError(
                f"[parts.{part}] net {role} -> {part}.{pin}: {layout.path.name} "
                f"has {what} named {pin!r}"
            )
        return pads[0].anchor

    labels: dict[str, list[str]] = {}
    for net in (n for n in model.nets if n.part == part):
        mcu = model.pin_for(net.role)
        source = mcu.name if mcu else f"GPIO{model.roles[net.role]}"
        labels.setdefault(anchor(net.role, net.pin), []).append(
            f"MCU {source} · {_role_name(net.role)}"
        )
    for net in (n for n in model.channel_nets if n.part == part):
        driver = model.parts[net.source].name
        labels.setdefault(anchor(net.role, net.pin), []).append(
            f"{driver} ch{net.channel} · {_role_name(net.role)}"
        )

    # A driver's channel pad: the role on it and every pin it reaches.
    reaches: dict[str, dict[str, list[str]]] = {}
    for net in (n for n in model.channel_nets if n.source == part):
        pad = anchor(net.role, str(net.channel))
        reaches.setdefault(pad, {}).setdefault(net.role, []).append(
            f"{model.parts[net.part].name} {net.pin}"
        )
    for pad, by_role in reaches.items():
        for role, ends in by_role.items():
            labels.setdefault(pad, []).append(f"{_role_name(role)} → {', '.join(ends)}")
    return {anchor: " · ".join(texts) for anchor, texts in labels.items()}


def pinouts(model: HardwareModel, repo_root: Path = REPO_ROOT) -> list[Pinout]:
    """The MCU board, then every part with a board reference, in hardware.toml order."""
    mcu = parse_layout(model.board.path)
    result = [
        Pinout(
            slug=model.board.path.stem,
            title=_title(model.board.path),
            layout=mcu,
            labels=mcu_labels(model, mcu),
        )
    ]
    for key, part in model.parts.items():
        if not part.board:
            continue
        path = repo_root / part.board
        layout = parse_layout(path)
        result.append(
            Pinout(
                slug=path.stem,
                title=_title(path),
                layout=layout,
                labels=part_labels(model, key, layout),
            )
        )
    slugs = [p.slug for p in result]
    dupes = sorted({s for s in slugs if slugs.count(s) > 1})
    if dupes:
        raise HardwareError(
            f"{model.project_dir}: board(s) {dupes} named by more than one part"
        )
    return result


def regenerate(
    project_dir: Path, repo_root: Path = REPO_ROOT
) -> dict[Path, str | None]:
    """Path -> regenerated SVG, or None for an SVG in the directory nothing produces."""
    model = join(project_dir, repo_root=repo_root)
    out_dir = model.project_dir / OUT_DIR
    wanted: dict[Path, str | None] = {}
    for pinout in pinouts(model, repo_root):
        notes = (
            f"Pads from {pinout.layout.path.relative_to(repo_root).as_posix()}",
            "Component side up · pad order exact · not to scale",
        )
        wanted[out_dir / f"{pinout.slug}.svg"] = render_svg(pinout, notes)
    if out_dir.is_dir():
        for stale in sorted(out_dir.glob("*.svg")):
            wanted.setdefault(stale, None)
    return wanted


def check_project(project_dir: Path, repo_root: Path = REPO_ROOT) -> list[str]:
    """One message per stale, missing or orphaned image; [] when clean."""
    problems = []
    for path, svg in regenerate(project_dir, repo_root).items():
        if svg is None:
            problems.append(f"ORPHANED: {path} (no board in hardware.toml produces it)")
        elif not path.is_file():
            problems.append(f"MISSING: {path}")
        elif path.read_text(encoding="utf-8") != svg:
            problems.append(f"STALE: {path}")
    return problems


def write_project(project_dir: Path, repo_root: Path = REPO_ROOT) -> list[str]:
    """Write every image and remove orphans; one line per file touched or checked."""
    report = []
    for path, svg in regenerate(project_dir, repo_root).items():
        if svg is None:
            path.unlink()
            report.append(f"Removed {path}")
        elif path.is_file() and path.read_text(encoding="utf-8") == svg:
            report.append(f"Up to date: {path}")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(svg, encoding="utf-8")
            report.append(f"Wrote {path}")
    return report


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="hardware.pinout", description=__doc__.split("\n")[0]
    )
    parser.add_argument(
        "--check", action="store_true", help="write nothing; exit 1 on drift"
    )
    parser.add_argument("projects", nargs="*", type=Path)
    args = parser.parse_args(argv)
    projects = args.projects or _tracked_projects(repo_root)

    problems = 0
    for project in projects:
        if args.check:
            for line in check_project(project, repo_root):
                problems += 1
                print(line)
            continue
        for line in write_project(project, repo_root):
            print(line)
    if problems:
        print(
            f"{problems} pinout image(s) out of date. Run `just hardware::gen` and commit "
            "the result; edit the board reference or hardware.toml, never the SVG."
        )
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except HardwareError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(2)
