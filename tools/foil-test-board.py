#!/usr/bin/env python3
"""Write the copper-foil test board's cut files (PRD-013 FR-022).

    python3 tools/foil-test-board.py           # write docs/prototyping/foil-test-board/*.svg
    python3 tools/foil-test-board.py --check   # fail if the committed files differ

One single-layer board, 70 x 60 mm, knife-cut from adhesive copper foil on a
Silhouette Portrait 2. It answers the questions that decide whether foil can
carry a module carrier, before any real board is laid out for it:

- trace widths 0.6-2.0 mm: the narrowest that survives weeding and transfer;
- clearances 0.4-1.27 mm between 1 mm traces: the smallest gap that stays open;
- a 90 degree, a 45 degree and a filleted corner: which ones tear;
- the pads of KiCad's PinSocket_1x08_P2.54mm_Vertical_SMD: whether a socket
  holds under a module being plugged and pulled;
- a 3 mm power trace with 5 mm pads: heat and current on 20 AWG leads;
- a crossover: a 1 mm trace bridged over another on a Kapton patch.

Three files, all sized in mm with 1 mm user units:

- copper.svg  cut paths only: every copper island, the board outline, and
              three spare bridge strips beside the board
- kapton.svg  cut paths only: three crossover patches
- guide.svg   copper filled and labelled, with a 50 mm scale bar, to print

Every cut path is closed. Copper sits face up: transfer tape lifts it from the
top and puts it down the same way up, so nothing is mirrored.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "docs/prototyping/foil-test-board"
BOARD_W, BOARD_H = 70.0, 60.0
MARGIN = 10.0
SPARES_X = 78.0  # bridge strips and patches, right of the board
PAGE_W, PAGE_H = MARGIN + SPARES_X + 12 + MARGIN, MARGIN + BOARD_H + MARGIN
CUT = "#e00000"

WIDTHS = (0.6, 0.8, 1.0, 1.27, 1.5, 2.0)
GAPS = (0.4, 0.6, 0.8, 1.0, 1.27)
CORNER_W, CORNER_LEG, FILLET_R, CHAMFER = 0.8, 8.0, 2.0, 2.0
# KiCad 10.0.6 stock Connector_PinSocket_2.54mm.pretty/
# PinSocket_1x08_P2.54mm_Vertical_SMD_Pin1Left.kicad_mod: pad N at
# ((-1)^N * 1.65, -8.89 + (N - 1) * 2.54), size 1.9 x 1. Rotated here so the
# row runs along x: pins 1, 3, 5, 7 above the row's axis.
SOCKET_PITCH, SOCKET_OFFSET, SOCKET_PAD = 2.54, 1.65, (1.0, 1.9)
CROSS_W, CROSS_PAD, CROSS_CLEAR, PATCH_W = 1.0, 2.0, 1.0, 4.0


@dataclass(frozen=True)
class Shape:
    name: str
    layer: str  # "copper", "kapton" or "outline"
    d: str  # SVG path data, closed, in board mm (origin at the board's corner)
    bbox: tuple[float, float, float, float]  # x0, y0, x1, y1


def fmt(v: float) -> str:
    return f"{v:.3f}".rstrip("0").rstrip(".")


def polygon(name, layer, pts) -> Shape:
    d = "M" + "L".join(f"{fmt(x)} {fmt(y)}" for x, y in pts) + "Z"
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return Shape(name, layer, d, (min(xs), min(ys), max(xs), max(ys)))


def rect(name, layer, x, y, w, h) -> Shape:
    return polygon(name, layer, [(x, y), (x + w, y), (x + w, y + h), (x, y + h)])


def dogbone(name, x1, x2, y, w, pad) -> Shape:
    """A horizontal trace of width `w` from x1 to x2 with a square pad at each end."""
    p, t = pad / 2, w / 2
    return polygon(
        name,
        "copper",
        [
            (x1 - p, y - p), (x1 + p, y - p), (x1 + p, y - t), (x2 - p, y - t),
            (x2 - p, y - p), (x2 + p, y - p), (x2 + p, y + p), (x2 - p, y + p),
            (x2 - p, y + t), (x1 + p, y + t), (x1 + p, y + p), (x1 - p, y + p),
        ],
    )  # fmt: skip


def stroke(name, pts, w) -> Shape:
    """The outline of an open polyline of width `w`: flat ends, mitred joins."""

    def normal(a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        return -dy / n, dx / n

    left, right = [], []
    for i, p in enumerate(pts):
        ns = [normal(pts[j], pts[j + 1]) for j in (i - 1, i) if 0 <= j < len(pts) - 1]
        nx, ny = sum(n[0] for n in ns), sum(n[1] for n in ns)
        k = math.hypot(nx, ny)
        nx, ny = nx / k, ny / k
        scale = (w / 2) / (nx * ns[0][0] + ny * ns[0][1])  # mitre length
        left.append((p[0] + nx * scale, p[1] + ny * scale))
        right.append((p[0] - nx * scale, p[1] - ny * scale))
    return polygon(name, "copper", left + right[::-1])


def fillet(name, x0, y0, leg, r, w) -> Shape:
    """An L of width `w`: right from (x0, y0), then down, turning on radius `r`."""
    t, cx, cy = w / 2, x0 + leg - r, y0 + r  # cx, cy: the arc's centre
    ro, ri = r + t, r - t
    d = (
        f"M{fmt(x0)} {fmt(y0 - t)}H{fmt(cx)}"
        f"A{fmt(ro)} {fmt(ro)} 0 0 1 {fmt(cx + ro)} {fmt(cy)}"
        f"V{fmt(y0 + leg)}H{fmt(cx + ri)}V{fmt(cy)}"
        f"A{fmt(ri)} {fmt(ri)} 0 0 0 {fmt(cx)} {fmt(y0 + t)}"
        f"H{fmt(x0)}Z"
    )
    return Shape(name, "copper", d, (x0, y0 - t, cx + ro, y0 + leg))


def shapes() -> list[Shape]:
    out = [rect("board outline", "outline", 0, 0, BOARD_W, BOARD_H)]
    for i, w in enumerate(WIDTHS):
        out.append(dogbone(f"width {fmt(w)}", 6, 30, 5 + 5 * i, w, 3.0))
    for i, g in enumerate(GAPS):
        top = 4 + 6 * i
        out.append(rect(f"gap {fmt(g)} upper", "copper", 40, top, 24, 1.0))
        out.append(rect(f"gap {fmt(g)} lower", "copper", 40, top + 1.0 + g, 24, 1.0))
    leg, w = CORNER_LEG, CORNER_W
    out.append(stroke("corner 90", [(4, 37), (4 + leg, 37), (4 + leg, 37 + leg)], w))
    out.append(
        stroke(
            "corner 45",
            [
                (14, 37),
                (14 + leg - CHAMFER, 37),
                (14 + leg, 37 + CHAMFER),
                (14 + leg, 37 + leg),
            ],
            w,
        )
    )
    out.append(fillet("corner fillet", 24, 37, leg, FILLET_R, w))
    pw, ph = SOCKET_PAD
    for n in range(1, 9):
        cx = 41 + (n - 1) * SOCKET_PITCH
        cy = 41 + (-SOCKET_OFFSET if n % 2 else SOCKET_OFFSET)
        out.append(rect(f"socket pin {n}", "copper", cx - pw / 2, cy - ph / 2, pw, ph))
    out.append(dogbone("power 3 mm", 6, 38, 53, 3.0, 5.0))
    # The crossover: a through trace along y = 52.5, and a trace across it at
    # x = 56 broken into two stubs, each ending in a pad CROSS_CLEAR from it.
    y, p = 52.5, CROSS_PAD / 2
    out.append(dogbone("crossover through", 46, 66, y, CROSS_W, 2.5))
    inner = CROSS_W / 2 + CROSS_CLEAR + p  # stub pad centre from the trace's axis
    for name, pad_y, end_y in (("upper", y - inner, 46.5), ("lower", y + inner, 58.5)):
        lo, hi = sorted((pad_y, end_y))
        out.append(
            polygon(
                f"crossover {name}",
                "copper",
                stub_with_pads(56, lo, hi, CROSS_W, CROSS_PAD),
            )
        )
    span = 2 * inner
    for i in range(3):
        out.append(
            rect(
                f"spare bridge {i + 1}",
                "copper",
                SPARES_X + 4 * i,
                4,
                CROSS_W,
                span + CROSS_PAD,
            )
        )
        # The patch covers the through trace with margin and stops 0.2 mm short
        # of each stub pad, which the bridge is soldered to.
        out.append(
            rect(
                f"spare patch {i + 1}",
                "kapton",
                SPARES_X - 1.5 + 4.5 * i,
                30,
                PATCH_W,
                2 * (CROSS_W / 2 + CROSS_CLEAR) - 0.4,
            )
        )
    return out


def stub_with_pads(x, y_top, y_bot, w, pad):
    """A vertical trace from y_top to y_bot with a square pad at each end."""
    p, t = pad / 2, w / 2
    assert y_bot - y_top > pad, "the pads would overlap and the outline fold"
    return [
        (x - p, y_top - p), (x + p, y_top - p), (x + p, y_top + p), (x + t, y_top + p),
        (x + t, y_bot - p), (x + p, y_bot - p), (x + p, y_bot + p), (x - p, y_bot + p),
        (x - p, y_bot - p), (x - t, y_bot - p), (x - t, y_top + p), (x - p, y_top + p),
    ]  # fmt: skip


def header(width, height, title) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{fmt(width)}mm" '
        f'height="{fmt(height)}mm" viewBox="0 0 {fmt(width)} {fmt(height)}" '
        'font-family="Helvetica, Arial, sans-serif">',
        f"<title>{title}</title>",
    ]


def cut_file(layers, title) -> str:
    out = header(PAGE_W, PAGE_H, title)
    out.append(f'<g transform="translate({fmt(MARGIN)} {fmt(MARGIN)})">')
    for s in shapes():
        if s.layer in layers:
            out.append(
                f'<path class="cut" data-name="{s.name}" d="{s.d}" fill="none" '
                f'stroke="{CUT}" stroke-width="0.1"/>'
            )
    out += ["</g>", "</svg>"]
    return "\n".join(out) + "\n"


LABELS = [
    (6, 2.2, "widths 0.6 → 2.0 mm, top down"),
    (40, 2.2, "gaps 0.4 → 1.27 mm, top down"),
    (4, 35, "90° · 45° · fillet R2"),
    (38, 35.2, "SMD socket 1x8 (pin 1 left, top row)"),
    (6, 49.2, "power: 3 mm trace, 5 mm pads"),
    (46, 47.6, "crossover"),
    (SPARES_X, 2.2, "bridges"),
    (SPARES_X - 0.5, 28.2, "Kapton (kapton.svg)"),
]


def guide_file() -> str:
    out = header(PAGE_W, PAGE_H + 12, "Copper-foil test board: guide, 1:1")
    out.append('<rect width="100%" height="100%" fill="white"/>')
    out.append(f'<g transform="translate({fmt(MARGIN)} {fmt(MARGIN)})">')
    fills = {"copper": "#b87333", "kapton": "#e8a33d", "outline": "none"}
    for s in shapes():
        stroke_ = CUT if s.layer == "outline" else "none"
        out.append(
            f'<path data-name="{s.name}" d="{s.d}" fill="{fills[s.layer]}" '
            f'stroke="{stroke_}" stroke-width="0.2"/>'
        )
    for x, y, text in LABELS:
        out.append(
            f'<text x="{fmt(x)}" y="{fmt(y)}" font-size="1.8" fill="#1f1f1f">{text}</text>'
        )
    y = BOARD_H + 6
    out.append(
        f'<line class="scale" x1="0" y1="{y}" x2="50" y2="{y}" stroke="#1f1f1f" stroke-width="0.3"/>'
    )
    for t in range(0, 51, 10):
        out.append(
            f'<line x1="{t}" y1="{y - 1.5}" x2="{t}" y2="{y + 1.5}" stroke="#1f1f1f" stroke-width="0.2"/>'
        )
    out.append(
        f'<text x="52" y="{y + 1}" font-size="2.5" fill="#1f1f1f">50 mm at 100 %</text>'
    )
    out += ["</g>", "</svg>"]
    return "\n".join(out) + "\n"


def outputs() -> dict[Path, str]:
    return {
        OUT / "copper.svg": cut_file(
            {"copper", "outline"}, "Copper-foil test board: copper cut"
        ),
        OUT / "kapton.svg": cut_file({"kapton"}, "Copper-foil test board: Kapton cut"),
        OUT / "guide.svg": guide_file(),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--check", action="store_true", help="fail if the committed files differ"
    )
    args = parser.parse_args(argv)
    files = outputs()
    if args.check:
        stale = [
            p.name for p, t in files.items() if not p.is_file() or p.read_text() != t
        ]
        if stale:
            print(f"FAIL: stale: {stale}; run python3 tools/foil-test-board.py")
            return 1
        print("foil test board up to date")
        return 0
    OUT.mkdir(parents=True, exist_ok=True)
    for path, text in files.items():
        path.write_text(text)
    print(f"wrote {', '.join(p.name for p in files)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
