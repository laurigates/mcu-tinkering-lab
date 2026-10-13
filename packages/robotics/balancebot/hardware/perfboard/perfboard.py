"""Lay the balancebot carrier out on a pad-per-hole perfboard.

The carrier schematic (../carrier/) is the source of truth for parts and nets.
This script places each part on a 2.54 mm hole grid, plans point-to-point wires
on the solder side, checks the result, and writes:

- perfboard.svg  component side (left) and the mirrored solder side (right)
- perfboard.md   the wire list, one line per connection, hole to hole

    just balancebot::perfboard          # regenerate
    just balancebot::perfboard-check    # fail if the committed outputs are stale

Pad offsets and courtyards are read from the KiCad footprint files, never typed
in. Parts that the PCB fits with off-grid or SMD footprints are swapped for
through-hole equivalents here (SWAPS); their pin numbers stay the schematic's.
"""

import argparse
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


POWER = {"VBAT", "GND"} | {
    f"MOT{s}_{c}" for s in "LR" for c in ("A1", "A2", "B1", "B2")
}
WIRE_COLOUR = {
    "VBAT": "#d62728",
    "GND": "#222222",
    "+3V3": "#ff7f0e",
    "BUCK_5V": "#e377c2",
    "XIAO_5V": "#e377c2",
}


def svg(parts, wires):
    s = 18  # px per hole
    m = 30
    top = 70  # room above the grid for the title and parts overhanging row 1
    w, h = COLS * s, ROWS * s
    gap = 60
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{2 * w + gap + 2 * m}" height="{h + top + m}" '
        f'font-family="monospace" font-size="9">',
        '<rect width="100%" height="100%" fill="white"/>',
    ]

    def view(x0, mirror, title):
        X = (
            (lambda c: x0 + (COLS - 1 - c) * s + s / 2)
            if mirror
            else (lambda c: x0 + c * s + s / 2)
        )
        Y = lambda r: top + r * s + s / 2  # noqa: E731
        out.append(
            f'<text x="{x0}" y="16" font-size="12" font-weight="bold">{title}</text>'
        )
        out.append(
            f'<rect x="{x0}" y="{top}" width="{w}" height="{h}" fill="#f4ecd8" stroke="#999"/>'
        )
        for c in range(COLS):
            out.append(f'<text x="{X(c) - 3}" y="{top - 3}">{chr(ord("A") + c)}</text>')
        for r in range(ROWS):
            lx = x0 + w + 3 if not mirror else x0 - 16
            out.append(f'<text x="{lx}" y="{Y(r) + 3}">{r + 1}</text>')
            for c in range(COLS):
                out.append(f'<circle cx="{X(c)}" cy="{Y(r)}" r="2" fill="#c9b27c"/>')
        if not mirror:
            for p in sorted(parts.values(), key=lambda p: p.ref in RAISED):
                c0, r0, c1, r1 = p.body
                xa, xb = X(c0) - s / 2 + s / 2, X(c1) - s / 2 + s / 2
                fill = "none" if p.ref in RAISED else "#dfe9f5"
                dash = ' stroke-dasharray="4 2"' if p.ref in RAISED else ""
                out.append(
                    f'<rect x="{min(xa, xb)}" y="{Y(r0)}" width="{abs(xb - xa)}" height="{Y(r1) - Y(r0)}" '
                    f'fill="{fill}" stroke="#336" stroke-width="1"{dash}/>'
                )
                out.append(
                    f'<text x="{min(xa, xb) + 2}" y="{Y(r0) + 9}" fill="#336">{p.ref} {p.value}</text>'
                )
        for p in parts.values():
            for num, (c, r) in p.pads.items():
                shape = "rect" if num == "1" else "circle"
                if shape == "rect":
                    out.append(
                        f'<rect x="{X(c) - 4}" y="{Y(r) - 4}" width="8" height="8" fill="#b87333"/>'
                    )
                else:
                    out.append(
                        f'<circle cx="{X(c)}" cy="{Y(r)}" r="4" fill="#b87333"/>'
                    )
        if mirror:
            for name, (ha, _), (hb, _) in wires:
                colour = WIRE_COLOUR.get(name, "#1f77b4")
                width = 3 if name in POWER else 1.6
                out.append(
                    f'<line x1="{X(ha[0])}" y1="{Y(ha[1])}" x2="{X(hb[0])}" y2="{Y(hb[1])}" '
                    f'stroke="{colour}" stroke-width="{width}" stroke-linecap="round" opacity="0.85"/>'
                )

    view(m, False, "Component side (parts)")
    view(m + w + gap, True, "Solder side (mirrored: wires)")
    out.append("</svg>")
    return "\n".join(out) + "\n"


def wire_list(parts, wires):
    lines = [
        "# Balancebot perfboard wire list",
        "",
        "<!-- Generated by perfboard.py from the carrier schematic; do not edit. -->",
        "",
        f"Board: {COLS} × {ROWS} holes, columns A–{chr(ord('A') + COLS - 1)}, rows 1–{ROWS}, "
        "named as seen from the **component side** (flip the drawing, not the names, on the solder side).",
        "Thick lines (VBAT, GND, motor coils) need 20 AWG or heavier; everything else 26–30 AWG.",
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
    lines += [
        "",
        "## Wires",
        "",
        "| ✓ | Net | From | To | Length (holes) |",
        "|---|---|---|---|---|",
    ]
    for name, (ha, la), (hb, lb) in wires:
        length = abs(ha[0] - hb[0]) + abs(ha[1] - hb[1])
        lines.append(
            f"| ☐ | {name} | {la} ({hole_name(*ha)}) | {lb} ({hole_name(*hb)}) | {length} |"
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
    outputs = {
        HERE / "perfboard.svg": svg(parts, wires),
        HERE / "perfboard.md": wire_list(parts, wires),
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
