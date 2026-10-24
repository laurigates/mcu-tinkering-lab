"""The drawing: the component side, then one mirrored solder-side panel per step.

Each step panel shows only that step's wires, coloured per net with a legend
and numbered as in the wire list; heavy nets are drawn thicker. Pad 1 is
square. Raised modules are dashed outlines drawn over the low parts.
"""

from __future__ import annotations

import math

from .grid import Board, col_name
from .route import is_bus

# Categorical slots 1-8 of the dataviz reference palette, light mode. Checked
# with its validator against a white surface: CVD and normal-vision separation
# pass; slots 3-5 are below 3:1 contrast, so every wire also carries a number
# and every panel a legend.
PALETTE = [
    "#2a78d6",
    "#eb6834",
    "#1baf7a",
    "#eda100",
    "#e87ba4",
    "#008300",
    "#6250d6",
    "#e34948",
]
INK, MUTED, GRID = "#1f1f1f", "#6b6b6b", "#d9d9d9"
PART_INK = "#4a5a78"  # part outlines and their labels
FONT = 10  # px, the drawing's default font size
# Text box estimate: Helvetica averages about 0.55 em per character; 0.62 em
# over-estimates it, so a box built from this covers the drawn text.
CHAR_EM = 0.62
ASCENT, DESCENT = 0.8, 0.25  # em above and below the baseline, generous
LETTERS_DY = 4  # the column letters' baseline, px above the hole grid
NUMBERS_DX = 4  # the row numbers' anchor, px beside the hole grid


def text_box(x, y, text, anchor="start", size=FONT):
    """(x0, y0, x1, y1): a box that covers `text` drawn with baseline at (x, y)."""
    width = CHAR_EM * size * len(text)
    left = {"start": x, "middle": x - width / 2, "end": x - width}[anchor]
    return left, y - ASCENT * size, left + width, y + DESCENT * size


def label_at(body_left, body_top, text, grid_top, grid_right):
    """Baseline (x, y) of a part's label on the component side.

    Inside the body's top-left corner while the body's top edge is on the hole
    grid. A body that reaches above the grid (a module's courtyard at row 1)
    would put it on the column letters, so that label goes above the letter
    row instead; the component panel has no legend, so the band is free.
    Either way a label that would run into the row numbers to the right of
    the grid is pulled left to end before them.
    """
    if body_top < grid_top:
        letters_top = text_box(0, grid_top - LETTERS_DY, "A")[1]
        y = letters_top - 2 - DESCENT * FONT
    else:
        y = body_top + 10
    right = text_box(body_left + 3, y, text)[2]
    return body_left + 3 - max(0, right - (grid_right + NUMBERS_DX - 1)), y


def svg(parts, numbered, routes, board: Board, steps, raised, heavy):
    s = 17  # px per hole
    w, h = board.cols * s, board.rows * s
    head = 74  # panel title, legend, column letters
    pad_x, gap_x, gap_y, margin = 24, 46, 30, 20
    pw, ph = w + pad_x, h + head
    ncols = 3
    panels = 1 + len(steps)
    nrows = -(-panels // ncols)
    width = margin * 2 + ncols * pw + (ncols - 1) * gap_x
    height = margin * 2 + nrows * ph + (nrows - 1) * gap_y
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'font-family="Helvetica, Arial, sans-serif" font-size="10" fill="{INK}">',
        '<rect width="100%" height="100%" fill="white"/>',
    ]

    def panel(index, title, subtitle, mirror, wires, legend):
        col, row = index % ncols, index // ncols
        x0 = margin + col * (pw + gap_x) + (pad_x if mirror else 0)
        y0 = margin + row * (ph + gap_y)
        top = y0 + head

        def X(c):
            return x0 + ((board.cols - 1 - c) if mirror else c) * s + s / 2

        def Y(r):
            return top + r * s + s / 2

        out.append(
            f'<text x="{x0}" y="{y0 + 12}" font-size="13" font-weight="bold">{title}</text>'
        )
        out.append(f'<text x="{x0}" y="{y0 + 27}" fill="{MUTED}">{subtitle}</text>')
        lx, ly = x0, y0 + 44
        for net, colour in legend:
            label_w = 14 + 6.2 * len(net) + 14
            if lx + label_w > x0 + w:
                lx, ly = x0, ly + 14
            out.append(
                f'<line x1="{lx}" y1="{ly - 3}" x2="{lx + 12}" y2="{ly - 3}" stroke="{colour}" '
                f'stroke-width="{3 if net in heavy else 2}" stroke-linecap="round"/>'
            )
            out.append(f'<text x="{lx + 16}" y="{ly}">{net}</text>')
            lx += label_w
        out.append(
            f'<rect x="{x0}" y="{top}" width="{w}" height="{h}" fill="white" stroke="{GRID}"/>'
        )
        for c in range(board.cols):
            out.append(
                f'<text x="{X(c)}" y="{top - LETTERS_DY}" text-anchor="middle" fill="{MUTED}">{col_name(c)}</text>'
            )
        for r in range(board.rows):
            nx = x0 - NUMBERS_DX if mirror else x0 + w + NUMBERS_DX
            anchor = "end" if mirror else "start"
            out.append(
                f'<text x="{nx}" y="{Y(r) + 3}" text-anchor="{anchor}" fill="{MUTED}">{r + 1}</text>'
            )
            for c in range(board.cols):
                out.append(f'<circle cx="{X(c)}" cy="{Y(r)}" r="1.6" fill="{GRID}"/>')
        if not mirror:
            for p in sorted(parts.values(), key=lambda p: p.ref in raised):
                c0, r0, c1, r1 = p.body
                xa, xb = X(c0), X(c1)
                is_raised = p.ref in raised
                fill = "none" if is_raised else "#eef3fa"
                dash = ' stroke-dasharray="4 2"' if is_raised else ""
                out.append(
                    f'<rect x="{min(xa, xb)}" y="{Y(r0)}" width="{abs(xb - xa)}" height="{Y(r1) - Y(r0)}" '
                    f'fill="{fill}" stroke="{PART_INK}" stroke-width="1"{dash}/>'
                )
                label = f"{p.ref} {p.value}"
                lx, ly = label_at(min(xa, xb), Y(r0), label, top, x0 + w)
                out.append(
                    f'<text x="{lx}" y="{ly}" fill="{PART_INK}" '
                    f'stroke="white" stroke-width="3" paint-order="stroke">{label}</text>'
                )

        def corners(path):
            """A route's ends and bends: the points its polyline needs."""
            keep = [path[0]]
            for a, p, b in zip(path, path[1:], path[2:]):
                if (a[0] == p[0]) != (p[0] == b[0]):
                    keep.append(p)
            return keep + [path[-1]]

        # Wires first, pads over them, so each wire visibly ends on its pad.
        # A bus is bare: a wide pale band with its net's colour down the middle.
        active = {}
        for wire in wires:
            n, _, net, colour, a, b = wire
            pts = " ".join(f"{X(x / 2):g},{Y(y / 2):g}" for x, y in corners(routes[n]))
            width_ = 3 if net in heavy else 2
            strokes = [("white", width_ + 4), (colour, width_)]
            if is_bus(wire):
                strokes.insert(1, ("#c9c9c9", width_ + 4))
                for x, y in routes[n][::2]:
                    active[(x // 2, y // 2)] = colour
            for stroke, sw in strokes:
                out.append(
                    f'<polyline points="{pts}" fill="none" stroke="{stroke}" '
                    f'stroke-width="{sw}" stroke-linecap="round" stroke-linejoin="round"/>'
                )
            active[a[0]] = active[b[0]] = colour
        pad_holes = {h for p in parts.values() for h in p.pads.values()}
        for (c, r), colour in sorted(active.items()):
            if (c, r) not in pad_holes:  # a splice or a bus end: wire ends meet
                out.append(f'<circle cx="{X(c)}" cy="{Y(r)}" r="3" fill="{colour}"/>')
        for p in parts.values():
            for num, hole in p.pads.items():
                c, r = hole
                if mirror and hole in active:
                    fill = active[hole]
                elif mirror:
                    fill = "#cfcfcf"
                else:
                    fill = "#b87333"
                if num == "1":
                    out.append(
                        f'<rect x="{X(c) - 4}" y="{Y(r) - 4}" width="8" height="8" fill="{fill}"/>'
                    )
                else:
                    out.append(f'<circle cx="{X(c)}" cy="{Y(r)}" r="4" fill="{fill}"/>')
        visits = {}
        for n, *_ in wires:
            for node in routes[n]:
                visits.setdefault(node, set()).add(n)
        placed = []
        for n, _, net, colour, _, _ in wires:
            # Put each number on a straight stretch of its own wire, as near the
            # middle as it can go while clearing other wires' crossings and the
            # numbers already placed. Gap crossings come first: there the label
            # clears every pad.
            path = routes[n]
            bends = set(corners(path))
            mid = (len(path) - 1) / 2
            spots = sorted(
                (
                    (node[0] % 2 == 0 or node[1] % 2 == 0, abs(i - mid), node)
                    for i, node in enumerate(path)
                    if node not in bends and visits[node] == {n}
                ),
            ) or [(True, 0, path[len(path) // 2])]
            spots = [(X(x / 2), Y(y / 2)) for *_, (x, y) in spots]

            def clearance(pt):
                return min((math.dist(pt, q) for q in placed), default=99)

            clear = [pt for pt in spots if clearance(pt) >= 15]
            mx, my = clear[0] if clear else max(spots, key=clearance)
            placed.append((mx, my))
            out.append(
                f'<circle cx="{mx}" cy="{my}" r="7" fill="white" stroke="{colour}" stroke-width="1.5"/>'
            )
            out.append(
                f'<text x="{mx}" y="{my + 3}" text-anchor="middle" font-size="8.5" font-weight="bold">{n}</text>'
            )

    panel(
        0,
        "Component side",
        "parts as placed; dashed = module raised on its socket",
        False,
        [],
        [],
    )
    for step, st in enumerate(steps):
        title, subtitle, step_nets = st.title, st.subtitle, st.nets
        wires = [w for w in numbered if w[1] == step]
        first, last = wires[0][0], wires[-1][0]
        legend = [(net, PALETTE[i]) for i, net in enumerate(step_nets)]
        panel(
            step + 1,
            f"Step {step + 1} · {title} (wires {first}–{last})",
            f"solder side, mirrored · {subtitle}",
            True,
            wires,
            legend,
        )
    out.append("</svg>")
    return "\n".join(out) + "\n"
