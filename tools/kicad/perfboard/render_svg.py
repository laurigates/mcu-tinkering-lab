"""The drawing: the component side, then one mirrored solder-side panel per step.

Each step panel shows only that step's wires, coloured per net with a legend
and numbered as in the wire list; heavy nets are drawn thicker. Pad 1 is
square. Raised modules are dashed outlines drawn over the low parts.
"""

from __future__ import annotations

import math

from .grid import Board, col_name

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
    net_of_pin = {}
    for _, _, net, colour, (ha, la), (hb, lb) in numbered:
        net_of_pin[ha], net_of_pin[hb] = (net, colour), (net, colour)

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
                f'<text x="{X(c)}" y="{top - 4}" text-anchor="middle" fill="{MUTED}">{col_name(c)}</text>'
            )
        for r in range(board.rows):
            nx = x0 - 4 if mirror else x0 + w + 4
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
                    f'fill="{fill}" stroke="#4a5a78" stroke-width="1"{dash}/>'
                )
                out.append(
                    f'<text x="{min(xa, xb) + 3}" y="{Y(r0) + 10}" fill="#4a5a78" '
                    f'stroke="white" stroke-width="3" paint-order="stroke">{p.ref} {p.value}</text>'
                )

        def corners(path):
            """A route's ends and bends: the points its polyline needs."""
            keep = [path[0]]
            for a, p, b in zip(path, path[1:], path[2:]):
                if (a[0] == p[0]) != (p[0] == b[0]):
                    keep.append(p)
            return keep + [path[-1]]

        # Wires first, pads over them, so each wire visibly ends on its pad.
        for n, _, net, colour, _, _ in wires:
            pts = " ".join(f"{X(x / 2):g},{Y(y / 2):g}" for x, y in corners(routes[n]))
            width_ = 3 if net in heavy else 2
            for stroke, sw in (("white", width_ + 4), (colour, width_)):
                out.append(
                    f'<polyline points="{pts}" fill="none" stroke="{stroke}" '
                    f'stroke-width="{sw}" stroke-linecap="round" stroke-linejoin="round"/>'
                )
        active = {ha for *_, (ha, _), _ in wires} | {hb for *_, (hb, _) in wires}
        for p in parts.values():
            for num, hole in p.pads.items():
                c, r = hole
                if mirror and hole in active:
                    fill = net_of_pin[hole][1]
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
            # numbers already placed. Channel intersections come first: there
            # the label clears every pad.
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
