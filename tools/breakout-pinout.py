#!/usr/bin/env python3
"""Read a breakout board's PHYSICAL pin order out of its vendor Eagle board file.

A wiring document is only useful if it says where a pin physically *is*, and
that is exactly the fact least likely to be right when it comes from memory. A
product photo is worse than memory: the labels are legible, the orientation is
not, and silkscreen is routinely printed on both faces so a mirrored read looks
correct. Vendors publish the board file; it is the authority, and this reads it.

Every number the robocar-unified motor wiring card prints came from this — the
PCA9685's row order (PWM nearest the chip, then V+, then GND at the edge), its
6-pin side headers (GND/OE/SCL/SDA/VCC/V+), and the TB6612FNG's two 8-pin rows
(VM…GND and PWMA…GND, both starting at the same end of the board).

    python3 tools/breakout-pinout.py <file.brd> [--element JP1] [--json]

Output is grouped by connector and sorted top-to-bottom then left-to-right in
BOARD coordinates, which for a top view is what you see with the components
facing you.

Two things the format will bite you with:

  * Pad coordinates in a `<package>` are relative to the element's origin, and
    the element carries a rotation (`R90`, `MR0`, …). `MR` is mirrored — the
    part is on the BOTTOM layer, so its x is negated before rotation. Skip that
    and a bottom-mounted connector comes out reversed, which reads as a
    plausible pinout.
  * Silkscreen text on layer 22 (bPlace) is the bottom face. Its stored x is a
    real board coordinate, so it agrees with layer 21 (tPlace) — but a reader
    holding the board upside down sees it mirrored. Report the layer, never
    flatten the two together.

Nets come from the `<signal>` section, so a pad's meaning is the vendor's own
net name (`PWMA`, `5.0V`) rather than a guess from a nearby label. Where a name
is generic (`N$7`) this falls back to the nearest silkscreen text on the pad's
own face — a HINT, and one that is wrong when a note happens to sit closer than
the label: the PCA9685's channel-11 pad is nearer the bottom-face "40-1000Hz"
than its own "11". The face is printed beside every such guess so a wrong one is
visible; never copy one into a document without reading it.

Only Eagle XML (.brd / .sch, Eagle 6+) is understood. KiCad's .kicad_pcb is
s-expressions, not XML, and is not handled — convert or read it directly.
"""

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

# Eagle layer numbers whose text is silkscreen a human can read on the board.
SILK_LAYERS = {"21": "top", "22": "bottom"}


def place(ex, ey, rot, px, py):
    """Element-relative pad coordinate -> absolute board coordinate.

    `rot` is Eagle's element rotation: an optional 'M' (mirrored onto the bottom
    layer) then 'R<degrees>'. Mirroring negates x BEFORE the rotation, which is
    the step that silently reverses a bottom-mounted connector when skipped.
    """
    mirrored = bool(rot) and rot.startswith("M")
    degrees = int("".join(ch for ch in (rot or "R0") if ch.isdigit()) or 0)
    if mirrored:
        px = -px
    theta = math.radians(degrees)
    return (
        ex + px * math.cos(theta) - py * math.sin(theta),
        ey + px * math.sin(theta) + py * math.cos(theta),
    )


def fnum(node, attr: str) -> float:
    """Required numeric attribute. Eagle omits nothing we read here, so a
    missing one means the file is not the shape this parser assumes — fail on
    it rather than defaulting to 0.0 and emitting a pinout stacked at the
    origin, which looks like a real answer."""
    raw = node.get(attr)
    if raw is None:
        raise ValueError(f"<{node.tag}> is missing required attribute '{attr}'")
    return float(raw)


def load(path: Path):
    root = ET.parse(path).getroot()

    packages = {}
    for lib in root.iter("library"):
        for pkg in lib.iter("package"):
            pads = {}
            for pad in list(pkg.iter("pad")) + list(pkg.iter("smd")):
                pads[pad.get("name")] = (fnum(pad, "x"), fnum(pad, "y"))
            if pads:
                packages[pkg.get("name")] = pads

    nets = defaultdict(dict)
    for signal in root.iter("signal"):
        for ref in signal.findall("contactref"):
            nets[ref.get("element")][ref.get("pad")] = signal.get("name")

    silk = []
    for text in root.iter("text"):
        layer = text.get("layer")
        if layer in SILK_LAYERS:
            silk.append(
                {
                    "face": SILK_LAYERS[layer],
                    "x": fnum(text, "x"),
                    "y": fnum(text, "y"),
                    "text": (text.text or "").strip(),
                }
            )

    elements = []
    for el in root.iter("element"):
        pads = packages.get(el.get("package"))
        if not pads:
            continue  # a part with no pads: a logo, a frame, a fiducial
        ex, ey, rot = fnum(el, "x"), fnum(el, "y"), el.get("rot")
        pins = []
        for pad_name, (px, py) in pads.items():
            ax, ay = place(ex, ey, rot, px, py)
            pins.append(
                {
                    "pad": pad_name,
                    "x": round(ax, 2),
                    "y": round(ay, 2),
                    "net": nets[el.get("name")].get(pad_name, "-"),
                }
            )
        # Top-to-bottom, then left-to-right: how a board is read in a top view.
        pins.sort(key=lambda p: (-p["y"], p["x"]))
        elements.append(
            {
                "element": el.get("name"),
                "package": el.get("package"),
                "rot": rot or "R0",
                "on": "bottom" if (rot or "").startswith("M") else "top",
                "pins": pins,
            }
        )
    elements.sort(key=lambda e: e["element"])
    return elements, silk


def nearest_silk(silk, x, y, face=None, limit=3.0):
    """Silkscreen label closest to a pad, for pads whose net name is generic.

    A HINT, not an authority. Proximity does not know what a label is *for*:
    on the PCA9685 the pad for channel 11 sits nearer the bottom-face note
    "40-1000Hz" than to its own "11", so an unfiltered nearest-match prints the
    frequency range as a channel name. Restricting to the pad's own face fixes
    that case and does not make the heuristic sound — always read the printed
    `silk[face]` and reject a label that is not the shape you expect.
    """
    best, best_d = None, limit
    for s in silk:
        if face and s["face"] != face:
            continue
        d = math.hypot(s["x"] - x, s["y"] - y)
        if d < best_d and s["text"]:
            best, best_d = s, d
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("board", type=Path, help="Eagle .brd (or .sch) file")
    ap.add_argument("--element", action="append", help="only this element (repeatable)")
    ap.add_argument(
        "--min-pads", type=int, default=2, help="skip parts with fewer pads"
    )
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    if not args.board.exists():
        print(f"no such file: {args.board}", file=sys.stderr)
        return 1

    elements, silk = load(args.board)
    if args.element:
        wanted = set(args.element)
        elements = [e for e in elements if e["element"] in wanted]
    elements = [e for e in elements if len(e["pins"]) >= args.min_pads]

    if not elements:
        print("no connectors matched — try --min-pads 1 or --element", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps({"elements": elements, "silkscreen": silk}, indent=2))
        return 0

    print(f"# {args.board.name} — physical pin order, board coordinates, top view\n")
    for el in elements:
        print(
            f"{el['element']}  ({el['package']}, rot={el['rot']}, mounted {el['on']})"
        )
        for pin in el["pins"]:
            label = ""
            if pin["net"] in ("-",) or pin["net"].startswith("N$"):
                near = nearest_silk(silk, pin["x"], pin["y"], face=el["on"])
                if near:
                    label = f"   silk[{near['face']}]: {near['text']!r}"
            print(
                f"    pad {pin['pad']:>3}  x={pin['x']:8.2f}  y={pin['y']:8.2f}  "
                f"{pin['net']}{label}"
            )
        print()

    print("Silkscreen text is printed on both faces on most breakouts; the 'face'")
    print("above says which. Bottom-face text reads mirrored when the board is")
    print("flipped, so quote the face alongside any label you copy into a doc.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
