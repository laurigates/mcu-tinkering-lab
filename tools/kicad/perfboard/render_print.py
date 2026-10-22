"""The 1:1 print: the component side and the mirrored solder side, in millimetres.

The SVG's width and height are in mm and its user unit is 1 mm, so a print at
100 % puts the holes 2.54 mm apart. Each panel carries:

- the hole-grid outline as a cut line, for kiss-cutting a placement sticker;
- a crosshair on each corner hole, to line the print up with a real board.

The solder side also has every wire, coloured as in `perfboard.svg`, but no
numbers: they do not fit at 2.54 mm. The page has a 50 mm scale bar, so the
printer's scaling can be checked with a ruler.

Elements carry ids and data attributes the tests read: the panels are
`<g id="component">` and `<g id="solder">`, a pad is `data-pad="REF.pin"`.
"""

from __future__ import annotations

from .grid import PITCH, Board, col_name
from .render_svg import GRID, INK, MUTED, PART_INK

MARGIN = 10  # page edge to content, mm
HEAD = 31  # title, note and scale bar above the panels, mm
LABEL = 5  # the band for column letters and row numbers, mm
GAP = 12  # between the two panels, besides their labels, mm
PAGE = 190  # widest content that fits A4 portrait with its margins, mm
SCALE_BAR = 50  # mm
CUT = "#e00000"
COPPER = "#b87333"


def mm(v):
    """A coordinate as SVG text: 3 decimals, trailing zeros dropped."""
    return f"{v:.3f}".rstrip("0").rstrip(".")


def svg(parts, numbered, routes, board: Board, raised, heavy, title):
    bw, bh = board.cols * PITCH, board.rows * PITCH
    side_by_side = LABEL + 2 * bw + GAP + 2 * LABEL <= PAGE
    comp = (MARGIN + LABEL, MARGIN + HEAD)
    if side_by_side:
        sold = (comp[0] + bw + LABEL + GAP, comp[1])
        width = sold[0] + bw + LABEL + MARGIN
        height = comp[1] + bh + MARGIN
    else:
        sold = (comp[0], comp[1] + bh + GAP + LABEL)
        width = comp[0] + bw + LABEL + MARGIN
        height = sold[1] + bh + MARGIN
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{mm(width)}mm" '
        f'height="{mm(height)}mm" viewBox="0 0 {mm(width)} {mm(height)}" '
        f'font-family="Helvetica, Arial, sans-serif" fill="{INK}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{MARGIN}" y="{MARGIN + 4}" font-size="4" font-weight="bold">'
        f"{title}, 1:1</text>",
        f'<text x="{MARGIN}" y="{MARGIN + 8}" font-size="2.5" fill="{MUTED}">'
        "Print at 100 % (no fit to page): the bar must measure 50 mm.</text>",
        f'<text x="{MARGIN}" y="{MARGIN + 11.5}" font-size="2.5" fill="{MUTED}">'
        "Red: the hole-grid outline, a cut line. Wire numbers: perfboard.svg.</text>",
    ]
    bx, by = MARGIN, MARGIN + 17
    out.append(
        f'<line class="scale" x1="{bx}" y1="{by}" x2="{bx + SCALE_BAR}" y2="{by}" '
        f'stroke="{INK}" stroke-width="0.3"/>'
    )
    for t in range(0, SCALE_BAR + 1, 10):
        out.append(
            f'<line x1="{bx + t}" y1="{by - 1.5}" x2="{bx + t}" y2="{by + 1.5}" '
            f'stroke="{INK}" stroke-width="0.2"/>'
        )
    out.append(
        f'<text x="{bx + SCALE_BAR + 2}" y="{by + 1}" font-size="2.5">{SCALE_BAR} mm</text>'
    )
    net_colour = {}
    for _, _, _, colour, (ha, _), (hb, _) in numbered:
        net_colour[ha] = net_colour[hb] = colour

    def panel(gid, origin, mirror, caption):
        x0, y0 = origin

        def X(c):
            return x0 + ((board.cols - 1 - c) if mirror else c) * PITCH + PITCH / 2

        def Y(r):
            return y0 + r * PITCH + PITCH / 2

        out.append(f'<g id="{gid}">')
        out.append(
            f'<text x="{mm(x0)}" y="{mm(y0 - LABEL - 1)}" font-size="2.5" '
            f'font-weight="bold">{caption}</text>'
        )
        for c in range(board.cols):
            out.append(
                f'<text x="{mm(X(c))}" y="{mm(y0 - 1)}" font-size="1.5" '
                f'text-anchor="middle" fill="{MUTED}">{col_name(c)}</text>'
            )
        nx, anchor = (x0 + bw + 1, "start") if mirror else (x0 - 1, "end")
        for r in range(board.rows):
            out.append(
                f'<text x="{mm(nx)}" y="{mm(Y(r) + 0.5)}" font-size="1.5" '
                f'text-anchor="{anchor}" fill="{MUTED}">{r + 1}</text>'
            )
            for c in range(board.cols):
                out.append(
                    f'<circle class="hole" cx="{mm(X(c))}" cy="{mm(Y(r))}" r="0.5" '
                    f'fill="none" stroke="{GRID}" stroke-width="0.1"/>'
                )
        if not mirror:
            for p in sorted(parts.values(), key=lambda p: p.ref in raised):
                c0, r0, c1, r1 = p.body
                xa, xb = sorted((X(c0), X(c1)))
                dash = ' stroke-dasharray="1 0.5"' if p.ref in raised else ""
                fill = "none" if p.ref in raised else "#eef3fa"
                out.append(
                    f'<rect x="{mm(xa)}" y="{mm(Y(r0))}" width="{mm(xb - xa)}" '
                    f'height="{mm(Y(r1) - Y(r0))}" fill="{fill}" stroke="{PART_INK}" '
                    f'stroke-width="0.15"{dash}/>'
                )
        else:
            for n, _, net, colour, _, _ in numbered:
                pts = " ".join(f"{mm(X(x / 2))},{mm(Y(y / 2))}" for x, y in routes[n])
                out.append(
                    f'<polyline class="wire" points="{pts}" fill="none" stroke="{colour}" '
                    f'stroke-width="{0.6 if net in heavy else 0.35}" '
                    'stroke-linecap="round" stroke-linejoin="round"/>'
                )
        for p in sorted(parts.values(), key=lambda p: p.ref):
            for num, hole in sorted(p.pads.items()):
                c, r = hole
                fill = net_colour.get(hole, "#cfcfcf") if mirror else COPPER
                attrs = f'data-pad="{p.ref}.{num}" fill="{fill}"'
                if num == "1":
                    out.append(
                        f'<rect {attrs} x="{mm(X(c) - 0.75)}" y="{mm(Y(r) - 0.75)}" '
                        'width="1.5" height="1.5"/>'
                    )
                else:
                    out.append(
                        f'<circle {attrs} cx="{mm(X(c))}" cy="{mm(Y(r))}" r="0.8"/>'
                    )
        if not mirror:
            # Labels last, haloed, just above the body so they leave pad 1's
            # square (the polarity mark) visible; kept inside the hole grid,
            # where a body reaching past it would put them on the column letters.
            for p in sorted(parts.values(), key=lambda p: p.ref):
                c0, r0, c1, _ = p.body
                lx = max(min(X(c0), X(c1)), x0) + 0.4
                ly = Y(r0) - 0.3
                if ly < y0 + 1.5:  # held inside: step right of the pad-1 corner
                    lx, ly = lx + PITCH, y0 + 1.5
                out.append(
                    f'<text x="{mm(lx)}" y="{mm(ly)}" font-size="1.6" fill="{PART_INK}" '
                    f'stroke="white" stroke-width="0.4" paint-order="stroke">{p.ref}</text>'
                )
        for c, r in (
            (0, 0),
            (board.cols - 1, 0),
            (0, board.rows - 1),
            (board.cols - 1, board.rows - 1),
        ):
            x, y = X(c), Y(r)
            out.append(
                f'<path class="corner" d="M{mm(x - 1.6)} {mm(y)}H{mm(x + 1.6)}'
                f'M{mm(x)} {mm(y - 1.6)}V{mm(y + 1.6)}" stroke="{INK}" stroke-width="0.1"/>'
            )
        out.append(
            f'<rect class="cut" x="{mm(x0)}" y="{mm(y0)}" width="{mm(bw)}" '
            f'height="{mm(bh)}" fill="none" stroke="{CUT}" stroke-width="0.2"/>'
        )
        out.append("</g>")

    panel("component", comp, False, "Component side")
    panel("solder", sold, True, "Solder side, mirrored · all wires")
    out.append("</svg>")
    return "\n".join(out) + "\n"
