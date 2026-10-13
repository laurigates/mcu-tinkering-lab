"""Lay the balancebot carrier out on a pad-per-hole perfboard.

The carrier schematic (../carrier/) is the source of truth for parts and nets.
This script places each part on a 2.54 mm hole grid, plans point-to-point wires
on the solder side, maze-routes each one through the channels between holes,
checks the result, and writes:

- perfboard.svg  component side (left) and the mirrored solder side (right)
- perfboard.md   the wire list, one line per connection, hole to hole

    just balancebot::perfboard          # regenerate
    just balancebot::perfboard-check    # fail if the committed outputs are stale

Pad offsets and courtyards are read from the KiCad footprint files, never typed
in. Parts that the PCB fits with off-grid or SMD footprints are swapped for
through-hole equivalents here (SWAPS); their pin numbers stay the schematic's.
"""

import argparse
import heapq
import math
import random
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
CARRIER = HERE.parent / "carrier" / "balancebot-carrier"
SCH = CARRIER / "balancebot-carrier.kicad_sch"
KICAD_FP = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints")
PROJECT_FP = {"Seeed_XIAO": CARRIER / "lib" / "Seeed_XIAO.pretty"}
PITCH = 2.54

# A 50 x 70 mm proto board: columns A..R, rows 1..24, component side up,
# robot forward = row 1.
COLS, ROWS = 18, 24

# Through-hole stand-ins for parts the PCB fits differently. Value overrides are
# electrically equivalent parts (B5819W is the SOD-123 1N5819).
SWAPS = {
    "J1": (
        "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
        "XT30 pigtail",
    ),
    "J2": ("Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", None),
    "J4": ("Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", None),
    "J5": ("Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", None),
    "C1": ("Capacitor_THT:CP_Radial_D8.0mm_P2.50mm", None),
    "C2": ("Capacitor_THT:CP_Radial_D8.0mm_P2.50mm", None),
    "C3": ("Capacitor_THT:C_Disc_D5.0mm_W2.5mm_P2.50mm", None),
    "D1": ("Diode_THT:D_DO-41_SOD81_P2.54mm_Vertical_AnodeUp", "1N5819"),
    "R1": ("Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical", None),
    "R2": ("Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical", None),
    "R3": ("Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical", None),
}
SKIP = {"H1", "H2", "H3", "H4"}  # mounting holes: the proto board has its own

# Modules standing in sockets are about 8.5 mm above the board, so low parts
# may sit under them; nothing tall may.
RAISED = {"U1", "A1", "A2", "J3"}
TALL = {"C1", "C2", "J2", "J4", "J5"}
# The GY-521 module lies flat on J3: 21 x 16 mm, header along the long edge.
# Its body is not in any footprint, so it is given in J3's own frame, in holes
# from pin 1 before rotation: the pins run along +y (0..7), the module extends
# 6.3 holes (16 mm) along +x.
GY521_BODY = (-0.6, -0.6, 5.7, 7.6)  # (x0, y0, x1, y1)

# ref: (column, row, rotation_deg). Column/row are 0-based hole indices of the
# footprint anchor (pad 1 for most parts, centre for the XIAO).
PLACEMENT = {}  # filled in below, kept beside the drawing logic for review


@dataclass
class Part:
    ref: str
    value: str
    fpid: str
    pads: dict  # pin number -> (col, row) absolute hole
    body: tuple  # (c0, r0, c1, r1) in hole units, component-side view


def footprint_geometry(fpid):
    lib, name = fpid.split(":", 1)
    path = PROJECT_FP.get(lib, KICAD_FP / f"{lib}.pretty") / f"{name}.kicad_mod"
    text = path.read_text()
    pads = {}
    for m in re.finditer(
        r'\(pad "(\d+)" thru_hole \w+\s*\(at ([-\d.]+) ([-\d.]+)', text
    ):
        pads.setdefault(m.group(1), (float(m.group(2)), float(m.group(3))))
    xs, ys = [], []
    for block in re.findall(
        r"\(fp_(?:line|rect|circle|arc|poly)(.*?)\(layer \"F\.CrtYd\"\)", text, re.S
    ):
        for x, y in re.findall(
            r"\((?:start|end|center|mid|xy) ([-\d.]+) ([-\d.]+)\)", block
        ):
            xs.append(float(x))
            ys.append(float(y))
    if not pads or not xs:
        sys.exit(f"{fpid}: no through-hole pads or no courtyard in {path}")
    return pads, (min(xs), min(ys), max(xs), max(ys))


def rotate(x, y, deg):
    """KiCad footprint rotation, y down: positive is counter-clockwise on screen."""
    r = {0: (x, y), 90: (y, -x), 180: (-x, -y), 270: (-y, x)}
    return r[deg % 360]


def to_hole(mm_value, what):
    h = mm_value / PITCH
    if abs(h - round(h)) > 0.1:  # 2.50 mm leads bend onto a 2.54 mm grid
        sys.exit(f"{what}: {mm_value} mm is not on the 2.54 mm grid")
    return round(h)


def export_netlist():
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "carrier.xml"
        subprocess.run(
            [
                "kicad-cli",
                "sch",
                "export",
                "netlist",
                "--format",
                "kicadxml",
                "-o",
                str(out),
                str(SCH),
            ],
            check=True,
            capture_output=True,
        )
        return ET.parse(out).getroot()


def build_parts(root):
    parts = {}
    for comp in root.iter("comp"):
        ref = comp.get("ref")
        if ref in SKIP:
            continue
        if ref not in PLACEMENT:
            sys.exit(f"{ref} has no PLACEMENT entry")
        fpid, value = SWAPS.get(ref, (comp.findtext("footprint"), None))
        value = value or comp.findtext("value")
        pads_mm, crt = footprint_geometry(fpid)
        c, r, rot = PLACEMENT[ref]
        pads = {}
        for num, (x, y) in pads_mm.items():
            rx, ry = rotate(x, y, rot)
            pads[num] = (
                c + to_hole(rx, f"{ref}.{num}"),
                r + to_hole(ry, f"{ref}.{num}"),
            )
        if ref == "J3":
            corners = [
                (GY521_BODY[0] * PITCH, GY521_BODY[1] * PITCH),
                (GY521_BODY[2] * PITCH, GY521_BODY[3] * PITCH),
            ]
        else:
            corners = [(crt[0], crt[1]), (crt[2], crt[3])]
        pts = [rotate(x, y, rot) for x, y in corners]
        xs = [c + p[0] / PITCH for p in pts]
        ys = [r + p[1] / PITCH for p in pts]
        parts[ref] = Part(ref, value, fpid, pads, (min(xs), min(ys), max(xs), max(ys)))
    missing = set(PLACEMENT) - set(parts)
    if missing:
        sys.exit(f"PLACEMENT entries with no schematic part: {sorted(missing)}")
    return parts


def nets_of(root):
    nets = {}
    for net in root.iter("net"):
        nodes = [
            (n.get("ref"), n.get("pin"))
            for n in net.iter("node")
            if n.get("ref") not in SKIP
        ]
        if len(nodes) > 1:
            nets[net.get("name").lstrip("/")] = nodes
    return nets


def overlaps(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def check_placement(parts):
    errors = []
    holes = {}
    for p in parts.values():
        for num, (c, r) in p.pads.items():
            if not (0 <= c < COLS and 0 <= r < ROWS):
                errors.append(f"{p.ref}.{num} at {hole_name(c, r)} is off the board")
            if (c, r) in holes:
                errors.append(
                    f"{p.ref}.{num} and {holes[(c, r)]} share hole {hole_name(c, r)}"
                )
            holes[(c, r)] = f"{p.ref}.{num}"
    refs = sorted(parts)
    for i, a in enumerate(refs):
        for b in refs[i + 1 :]:
            if not overlaps(parts[a].body, parts[b].body):
                continue
            ra, rb = a in RAISED, b in RAISED
            if ra and rb:
                errors.append(f"raised modules {a} and {b} overlap")
            elif ra != rb and (a in TALL or b in TALL):
                errors.append(
                    f"tall part {a if a in TALL else b} sits under raised {a if ra else b}"
                )
            elif not ra and not rb:
                errors.append(f"{a} and {b} overlap on the board")
    return errors


def hole_name(c, r):
    return f"{chr(ord('A') + c)}{r + 1}"


def plan_wires(parts, nets):
    """Minimum spanning tree per net, Manhattan distance, on pad holes only."""
    wires = []
    for name, nodes in sorted(nets.items()):
        pts = [(parts[ref].pads[pin], f"{ref}.{pin}") for ref, pin in nodes]
        tree, rest = [pts[0]], pts[1:]
        while rest:
            best = min(
                (
                    (abs(a[0][0] - b[0][0]) + abs(a[0][1] - b[0][1]), a, b)
                    for a in tree
                    for b in rest
                ),
                key=lambda t: (t[0], t[1][1], t[2][1]),
            )
            _, a, b = best
            wires.append((name, a, b))
            tree.append(b)
            rest.remove(b)
    return wires


def check_wires(parts, nets, wires):
    """Independent of plan_wires: union-find over the holes the wires join."""
    owner = {hole: f"{p.ref}.{n}" for p in parts.values() for n, hole in p.pads.items()}
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for _, (ha, _), (hb, _) in wires:
        parent[find(ha)] = find(hb)
    errors = []
    group_net = {}
    for name, nodes in nets.items():
        roots = {find(parts[ref].pads[pin]) for ref, pin in nodes}
        if len(roots) != 1:
            errors.append(f"net {name} is split into {len(roots)} groups")
        for root in roots:
            if root in group_net and group_net[root] != name:
                errors.append(f"nets {name} and {group_net[root]} are joined")
            group_net[root] = name
    for _, (ha, la), (hb, lb) in wires:
        for hole, label in ((ha, la), (hb, lb)):
            if owner.get(hole) != label:
                errors.append(
                    f"wire end {label} lands on {hole_name(*hole)} owned by {owner.get(hole)}"
                )
    return errors


# Build steps, one solder-side panel each. Every net belongs to exactly one;
# within a panel, nets take the categorical palette's slots in this order.
STEPS = [
    ("Power", "VBAT, GND and the 5 V feed", ["VBAT", "GND", "BUCK_5V", "XIAO_5V"]),
    ("Logic power", "3V3 straps and the shared nENABLE", ["+3V3", "NENABLE"]),
    (
        "Motor coils",
        "driver outputs to the motor headers",
        [f"MOT{s}_{c}" for s in "LR" for c in ("A1", "A2", "B1", "B2")],
    ),
    (
        "Signals",
        "XIAO to drivers, IMU and the pack divider",
        [
            "STEP_L",
            "DIR_L",
            "STEP_R",
            "DIR_R",
            "IMU_SDA",
            "IMU_SCL",
            "IMU_INT",
            "PACK_SENSE",
        ],
    ),
]
# Thick wire (20 AWG or heavier) for these nets.
HEAVY = {"VBAT", "GND"} | {
    f"MOT{s}_{c}" for s in "LR" for c in ("A1", "A2", "B1", "B2")
}
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


def number_wires(nets, wires):
    """Group the planned wires by build step and number them 1..N in that order."""
    grouped = {n for _, _, ns in STEPS for n in ns}
    missing, extra = set(nets) - grouped, grouped - set(nets)
    if missing or extra:
        sys.exit(
            f"STEPS out of step with the schematic: missing {sorted(missing)}, unknown {sorted(extra)}"
        )
    by_net = {}
    for w in wires:
        by_net.setdefault(w[0], []).append(w)
    numbered, n = [], 0
    for step, (_, _, step_nets) in enumerate(STEPS):
        for slot, net in enumerate(step_nets):
            for _, a, b in by_net[net]:
                n += 1
                numbered.append((n, step, net, PALETTE[slot], a, b))
    return numbered


# Maze routing runs on a half-pitch grid: even coordinates are holes, odd ones
# the channels between them. A wire leaves its hole by a half-pitch stub, runs
# in channels, and never passes over any other hole. Within one build step two
# wires may cross but never share a channel; a bend cannot sit on a crossing.
# Costs in half-pitch steps. Swept on this board, a crossing cost of 60 gives
# 18 crossings against 25 at 12, for 3 % more wire.
STEP_COST, BEND_COST, CROSS_COST = 1, 4, 60
ORDER_TRIES = 24  # wire orders tried per step; the cheapest result is kept
DIRS = {"E": (1, 0), "W": (-1, 0), "S": (0, 1), "N": (0, -1)}
AXIS = {"E": "H", "W": "H", "S": "V", "N": "V"}
BACK = {"E": "W", "W": "E", "S": "N", "N": "S"}


def route_wire(start, goal, used, stubs):
    """(cost, path) of the cheapest channel route from hole `start` to hole
    `goal`, or None if there is none.

    `used` maps a channel node to the axes ("H", "V") already run through it,
    `stubs` maps a hole to the directions its earlier wires leave by.
    """
    gx, gy = 2 * COLS - 2, 2 * ROWS - 2
    s, g = (2 * start[0], 2 * start[1]), (2 * goal[0], 2 * goal[1])
    heap = [(0, 0, s, "")]  # cost, tie-break, node, direction arrived in
    best, prev, tie = {(s, ""): 0}, {}, 0
    while heap:
        cost, _, node, came = heapq.heappop(heap)
        if node == g:
            path, key = [node], (node, came)
            while key in prev:
                key = prev[key]
                path.append(key[0])
            return cost, path[::-1]
        if cost > best[(node, came)]:
            continue
        for d, (dx, dy) in DIRS.items():
            if came and d == BACK[came]:
                continue
            nxt = (node[0] + dx, node[1] + dy)
            if not (0 <= nxt[0] <= gx and 0 <= nxt[1] <= gy):
                continue
            if node == s:
                if d in stubs.get(s, ()):
                    continue
            else:
                axes = {AXIS[came], AXIS[d]}
                if used.get(node, set()) & axes or (len(axes) == 2 and used.get(node)):
                    continue
            step = STEP_COST + (BEND_COST if came and d != came else 0)
            if nxt[0] % 2 == 0 and nxt[1] % 2 == 0:  # a hole
                if nxt != g or BACK[d] in stubs.get(g, ()):
                    continue
            elif used.get(nxt):
                step += CROSS_COST
            key = (nxt, d)
            if cost + step < best.get(key, math.inf):
                best[key] = cost + step
                prev[key] = (node, came)
                tie += 1
                heapq.heappush(heap, (cost + step, tie, nxt, d))
    return None


def route_step(wires):
    """Route one step's wires in the given order; a wire that finds no path
    moves to the front and the step starts over. Returns (total cost, routes)."""
    order = list(wires)
    for _ in range(len(order) + 1):
        used, stubs, routes, total = {}, {}, {}, 0
        for w in order:
            found = route_wire(w[4][0], w[5][0], used, stubs)
            if found is None:
                order.remove(w)
                order.insert(0, w)
                break
            cost, path = found
            total += cost
            routes[w[0]] = path
            for a, b in ((path[0], path[1]), (path[-1], path[-2])):
                d = next(k for k, v in DIRS.items() if (a[0] + v[0], a[1] + v[1]) == b)
                stubs.setdefault(a, set()).add(d)
            for i in range(1, len(path) - 1):
                axes = {
                    "H" if path[i - 1][1] == path[i][1] else "V",
                    "H" if path[i][1] == path[i + 1][1] else "V",
                }
                used.setdefault(path[i], set()).update(axes)
        else:
            return total, routes
    sys.exit(f"cannot route wires {[w[0] for w in wires]} without sharing a channel")


def route_wires(numbered, tries=ORDER_TRIES):
    """Route each step in several wire orders and keep the cheapest. Shortest
    first is tried first; the rest are seeded shuffles, so a run is repeatable."""
    routes = {}
    for step in range(len(STEPS)):
        wires = sorted(
            (w for w in numbered if w[1] == step),
            key=lambda w: (
                abs(w[4][0][0] - w[5][0][0]) + abs(w[4][0][1] - w[5][0][1]),
                w[0],
            ),
        )
        rng = random.Random(step)
        orders = [wires] + [rng.sample(wires, len(wires)) for _ in range(tries - 1)]
        routes.update(min((route_step(o) for o in orders), key=lambda r: r[0])[1])
    return routes


def check_routes(numbered, routes):
    """Independent of the router: re-walk each path against the routing rules."""
    errors = []
    for step in range(len(STEPS)):
        seen = {}  # half-pitch edge -> wire number
        visits = {}  # channel node -> [(wire number, bends there)]
        for n, st, _, _, (ha, _), (hb, _) in numbered:
            if st != step:
                continue
            path = routes.get(n)
            if not path:
                errors.append(f"wire {n} has no route")
                continue
            if path[0] != (2 * ha[0], 2 * ha[1]) or path[-1] != (2 * hb[0], 2 * hb[1]):
                errors.append(f"wire {n} does not run between its own holes")
            for a, b in zip(path, path[1:]):
                if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                    errors.append(f"wire {n} jumps from {a} to {b}")
                edge = frozenset((a, b))
                if edge in seen:
                    errors.append(f"wires {seen[edge]} and {n} share a channel at {a}")
                seen[edge] = n
            for a, p, b in zip(path, path[1:], path[2:]):
                if p[0] % 2 == 0 and p[1] % 2 == 0:
                    errors.append(
                        f"wire {n} passes over hole {hole_name(p[0] // 2, p[1] // 2)}"
                    )
                bend = (a[0] == p[0]) != (p[0] == b[0])
                visits.setdefault(p, []).append((n, bend))
        for p, v in visits.items():
            if len(v) > 1 and any(bend for _, bend in v):
                errors.append(f"wires {[n for n, _ in v]} meet at a bend at {p}")
    return errors


def svg(parts, numbered, routes):
    s = 17  # px per hole
    w, h = COLS * s, ROWS * s
    head = 74  # panel title, legend, column letters
    pad_x, gap_x, gap_y, margin = 24, 46, 30, 20
    pw, ph = w + pad_x, h + head
    ncols = 3
    panels = 1 + len(STEPS)
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
            return x0 + ((COLS - 1 - c) if mirror else c) * s + s / 2

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
                f'stroke-width="{3 if net in HEAVY else 2}" stroke-linecap="round"/>'
            )
            out.append(f'<text x="{lx + 16}" y="{ly}">{net}</text>')
            lx += label_w
        out.append(
            f'<rect x="{x0}" y="{top}" width="{w}" height="{h}" fill="white" stroke="{GRID}"/>'
        )
        for c in range(COLS):
            out.append(
                f'<text x="{X(c)}" y="{top - 4}" text-anchor="middle" fill="{MUTED}">{chr(ord("A") + c)}</text>'
            )
        for r in range(ROWS):
            nx = x0 - 4 if mirror else x0 + w + 4
            anchor = "end" if mirror else "start"
            out.append(
                f'<text x="{nx}" y="{Y(r) + 3}" text-anchor="{anchor}" fill="{MUTED}">{r + 1}</text>'
            )
            for c in range(COLS):
                out.append(f'<circle cx="{X(c)}" cy="{Y(r)}" r="1.6" fill="{GRID}"/>')
        if not mirror:
            for p in sorted(parts.values(), key=lambda p: p.ref in RAISED):
                c0, r0, c1, r1 = p.body
                xa, xb = X(c0), X(c1)
                raised = p.ref in RAISED
                fill = "none" if raised else "#eef3fa"
                dash = ' stroke-dasharray="4 2"' if raised else ""
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
            width_ = 3 if net in HEAVY else 2
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
    for step, (title, subtitle, step_nets) in enumerate(STEPS):
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


def wire_list(parts, numbered, routes):
    lines = [
        "# Balancebot perfboard wire list",
        "",
        "<!-- Generated by perfboard.py from the carrier schematic; do not edit. -->",
        "",
        f"Board: {COLS} × {ROWS} holes, columns A–{chr(ord('A') + COLS - 1)}, rows 1–{ROWS}, "
        "named as seen from the **component side** (flip the drawing, not the names, on the solder side).",
        "Wires are numbered as in `perfboard.svg`, one drawing panel per step.",
        "Each wire's drawn route runs between the holes, never over one, and its length is "
        "measured along that route.",
        "Heavy nets (VBAT, GND, motor coils) need 20 AWG or heavier; everything else 26–30 AWG.",
        "Neighbouring holes (length 1) can be a solder bridge instead of a wire.",
        "",
        "## Parts",
        "",
        "| Ref | Part | Pin 1 at | Footprint |",
        "|---|---|---|---|",
    ]
    for p in sorted(parts.values(), key=lambda p: p.ref):
        lines.append(
            f"| {p.ref} | {p.value} | {hole_name(*p.pads['1'])} | `{p.fpid.split(':')[1]}` |"
        )
    for step, (title, subtitle, _) in enumerate(STEPS):
        lines += [
            "",
            f"## Step {step + 1}: {title}",
            "",
            f"{subtitle[0].upper()}{subtitle[1:]}.",
            "",
            "| ✓ | # | Net | Gauge | From | To | Length (holes) |",
            "|---|---|---|---|---|---|---|",
        ]
        for n, st, net, _, (ha, la), (hb, lb) in numbered:
            if st != step:
                continue
            length = (len(routes[n]) - 1) // 2  # half-pitch steps to holes
            gauge = "20 AWG" if net in HEAVY else "26–30 AWG"
            lines.append(
                f"| ☐ | {n} | {net} | {gauge} | {la} ({hole_name(*ha)}) | {lb} ({hole_name(*hb)}) | {length} |"
            )
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--check", action="store_true", help="fail if the committed outputs differ"
    )
    args = ap.parse_args()
    root = export_netlist()
    parts = build_parts(root)
    nets = nets_of(root)
    errors = check_placement(parts)
    wires = plan_wires(parts, nets)
    errors += check_wires(parts, nets, wires)
    if errors:
        print("\n".join("FAIL " + e for e in errors))
        sys.exit(1)
    numbered = number_wires(nets, wires)
    routes = route_wires(numbered)
    errors = check_routes(numbered, routes)
    if errors:
        print("\n".join("FAIL " + e for e in errors))
        sys.exit(1)
    outputs = {
        HERE / "perfboard.svg": svg(parts, numbered, routes),
        HERE / "perfboard.md": wire_list(parts, numbered, routes),
    }
    if args.check:
        stale = [
            p.name
            for p, text in outputs.items()
            if not p.exists() or p.read_text() != text
        ]
        if stale:
            sys.exit(f"stale: {stale}; run just balancebot::perfboard")
        print(
            f"perfboard up to date: {len(parts)} parts, {len(wires)} wires, {len(nets)} nets connected"
        )
        return
    for path, text in outputs.items():
        path.write_text(text)
    print(f"wrote {len(parts)} parts, {len(wires)} wires, {len(nets)} nets connected")


PLACEMENT.update(
    {
        # Top band: XIAO with USB-C overhanging the top edge, GY-521 flat to its
        # right (J3 pin 1 at the right end, module extending down).
        "U1": (3, 3, 90),
        "J3": (15, 0, 270),
        # Middle band: both drivers unrotated, so both VMOT pins (pin 8) sit at
        # the bottom of their motor columns, next to the bulk caps.
        "A1": (1, 9, 0),
        "A2": (11, 9, 0),
        # Between the drivers: pull-up and the pack divider (low parts).
        "R1": (7, 10, 0),
        "R2": (8, 12, 0),
        "R3": (8, 14, 0),
        "C3": (8, 16, 0),
        # Bottom band: bulk caps under each VMOT, then connectors on the edge.
        "C1": (2, 19, 0),
        "C2": (11, 19, 0),
        "J2": (5, 21, 90),
        "D1": (10, 22, 0),
        "J4": (1, 23, 90),
        "J1": (7, 23, 90),
        "J5": (13, 23, 90),
    }
)

if __name__ == "__main__":
    main()
