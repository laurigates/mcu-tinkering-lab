"""Build balancebot-carrier.kicad_pcb from the schematic netlist and a placement table.

Run with KiCad's bundled Python (it provides ``pcbnew``):

    just balancebot::carrier-board

The schematic is the source of truth for parts, footprints and nets; this script
only decides where each part sits. It rewrites the board from scratch, so routing
(``route_board.py``) must run after it.
"""

import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PRJ = os.path.join(os.path.dirname(HERE), "balancebot-carrier")
SCH = os.path.join(PRJ, "balancebot-carrier.kicad_sch")
PCB = os.path.join(PRJ, "balancebot-carrier.kicad_pcb")
KICAD_FP = os.environ.get(
    "KICAD_FOOTPRINT_DIR",
    "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints",
)
PROJECT_LIBS = {"Seeed_XIAO": os.path.join(PRJ, "lib", "Seeed_XIAO.pretty")}

# Board: 90 x 62 mm, origin at the top-left corner. Robot forward is -y (up).
ORIGIN = (100.0, 100.0)
BOARD_W, BOARD_H = 90.0, 62.0

# ref: (x_mm, y_mm, rotation_deg) relative to ORIGIN. Footprint anchors are the
# library's own (pad 1 for the Pololu and JST parts, centre for the XIAO and the
# SMD passives).
PLACEMENT = {
    "U1": (45.0, 14.0, 90),  # XIAO, USB-C toward the top edge
    "A1": (14.0, 21.0, 0),  # left DRV8825, motor column toward the left edge
    "A2": (76.0, 38.78, 180),  # right DRV8825, motor column toward the right edge
    "J4": (5.0, 24.0, 270),  # left motor
    "J5": (85.0, 36.0, 90),  # right motor
    "J3": (36.11, 33.0, 90),  # GY-521 socket; the module lies flat over the centre
    # XT30 mating face is the body end away from the pins; 180 deg puts it at the
    # bottom edge with the pins 13.6 mm inboard.
    "J1": (22.0, 48.1, 180),
    "J2": (60.0, 56.0, 0),  # LM2596 module
    "D1": (71.0, 50.0, 0),  # B5819W, buck -> XIAO 5V
    "C1": (9.75, 44.0, 0),  # VMOT bulk, left driver
    "C2": (77.75, 14.0, 0),  # VMOT bulk, right driver
    "R1": (31.5, 31.0, 90),  # nENABLE pull-up, beside A1's EN pin
    "R2": (62.0, 4.0, 0),  # pack divider top
    "R3": (62.0, 9.0, 0),  # pack divider bottom
    "C3": (61.5, 14.5, 0),  # pack divider filter
    "H1": (4.0, 4.0, 0),
    "H2": (86.0, 4.0, 0),
    "H3": (4.0, 58.0, 0),
    "H4": (86.0, 58.0, 0),
}

# Where the GY-521 lies flat on its socket (21 x 16 mm, header along the long
# edge). Drawn on the silkscreen so nothing tall is placed under it later.
GY521_AREA = (34.5, 31.5, 55.5, 48.5)

# Reference designators that sit on the board edge and carry no assembly
# information (the holes are self-evident; J1 is labelled by its own +/-).
HIDDEN_REFS = {"H1", "H2", "H3", "H4", "J1"}

# Reference text moved off its library default: (x_mm, y_mm) relative to ORIGIN,
# where the default lands on a neighbouring part's silkscreen.
REF_POSITIONS = {"J3": (57.0, 33.0)}
# Seeed's reference text is below KiCad's minimum silkscreen height.
REF_SIZE_MM = {"U1": 1.0}


def mm(v):
    return pcbnew.FromMM(v)


def export_netlist():
    out = os.path.join(PRJ, "output", "carrier.net.xml")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    subprocess.run(
        [
            "kicad-cli",
            "sch",
            "export",
            "netlist",
            "--format",
            "kicadxml",
            "-o",
            out,
            SCH,
        ],
        check=True,
        capture_output=True,
    )
    return ET.parse(out).getroot()


# One reader for every load. pcbnew.FootprintLoad() works as well: the untyped
# SwigPyObject errors once blamed on it came from board.Remove() (see main() and
# .claude/rules/kicad-scripting.md), and 200 FootprintLoad() calls without it
# all return FOOTPRINT.
SEXPR_IO = pcbnew.PCB_IO_KICAD_SEXPR()


def load_footprint(fpid):
    lib, name = fpid.split(":", 1)
    path = PROJECT_LIBS.get(lib, os.path.join(KICAD_FP, f"{lib}.pretty"))
    fp = SEXPR_IO.FootprintLoad(path, name)
    if fp is None:
        sys.exit(f"footprint {fpid} not found in {path}")
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def rect(board, x1, y1, x2, y2, layer, width):
    x0, y0 = ORIGIN
    shape = pcbnew.PCB_SHAPE(board)
    shape.SetShape(pcbnew.SHAPE_T_RECT)
    shape.SetStart(pcbnew.VECTOR2I(mm(x0 + x1), mm(y0 + y1)))
    shape.SetEnd(pcbnew.VECTOR2I(mm(x0 + x2), mm(y0 + y2)))
    shape.SetLayer(layer)
    shape.SetWidth(mm(width))
    board.Add(shape)


def silk_text(board, text, x, y, size=1.0):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(pcbnew.VECTOR2I(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y)))
    t.SetLayer(pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    # JLCPCB's silkscreen minimums are ~1.0 mm height and ~0.15 mm stroke.
    t.SetTextThickness(mm(max(0.16, size * 0.15)))
    board.Add(t)


def main():
    root = export_netlist()
    # A fresh board every run. Clearing a loaded one does not work: after
    # board.Remove() of its footprints, SWIG returns untyped objects for every
    # later call (KiCad 10.0.6). NewBoard() also writes a .kicad_pro beside the
    # file, so it is created in a scratch directory and saved to PCB.
    scratch = tempfile.mkdtemp(prefix="carrier-")
    board = pcbnew.NewBoard(os.path.join(scratch, "carrier.kicad_pcb"))
    rect(board, 0, 0, BOARD_W, BOARD_H, pcbnew.Edge_Cuts, 0.1)
    rect(board, *GY521_AREA, pcbnew.F_SilkS, 0.15)

    footprints = {}
    for comp in root.iter("comp"):
        ref = comp.get("ref")
        if ref not in PLACEMENT:
            sys.exit(f"{ref} has no placement entry")
        fp = load_footprint(comp.findtext("footprint"))
        fp.SetReference(ref)
        fp.SetValue(comp.findtext("value"))
        fp.SetPath(pcbnew.KIID_PATH("/" + comp.findtext("tstamps")))
        # Custom symbol fields (LCSC) go onto the footprint for the BOM and DRC
        # parity. SetField() creates them visible on F.Silkscreen, so hide them.
        for field in comp.iter("field"):
            name, text = field.get("name"), field.text
            if name in ("Footprint", "Datasheet", "Description") or not text:
                continue
            fp.SetField(name, text)
            fp.GetField(name).SetLayer(pcbnew.F_Fab)
            fp.GetField(name).SetVisible(False)
        x, y, rot = PLACEMENT[ref]
        fp.SetPosition(pcbnew.VECTOR2I(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y)))
        fp.SetOrientationDegrees(rot)
        if ref in HIDDEN_REFS:
            fp.Reference().SetVisible(False)
        if ref in REF_POSITIONS:
            rx, ry = REF_POSITIONS[ref]
            fp.Reference().SetPosition(
                pcbnew.VECTOR2I(mm(ORIGIN[0] + rx), mm(ORIGIN[1] + ry))
            )
        if ref in REF_SIZE_MM:
            size = mm(REF_SIZE_MM[ref])
            fp.Reference().SetTextSize(pcbnew.VECTOR2I(size, size))
            fp.Reference().SetTextThickness(int(size * 0.15))
        board.Add(fp)
        footprints[ref] = fp
    missing = set(PLACEMENT) - set(footprints)
    if missing:
        sys.exit(f"placement entries with no schematic part: {sorted(missing)}")

    for net in root.iter("net"):
        name = net.get("name").lstrip("/")
        nodes = list(net.iter("node"))
        if not nodes:
            continue
        info = pcbnew.NETINFO_ITEM(board, name)
        board.Add(info)
        for node in nodes:
            pads = [
                p
                for p in footprints[node.get("ref")].Pads()
                if p.GetNumber() == node.get("pin")
            ]
            if not pads:
                sys.exit(f"{node.get('ref')} has no pad {node.get('pin')}")
            for pad in pads:
                pad.SetNet(info)

    silk_text(board, "balancebot carrier", 45.0, 52.0, 1.0)
    silk_text(board, "FWD ^", 45.0, 28.0, 1.2)
    silk_text(board, "GY-521: X arrow to FWD", 45.0, 45.0, 1.0)
    board.Save(PCB)
    shutil.rmtree(scratch)
    print(f"placed {len(footprints)} footprints, wrote {PCB}")


if __name__ == "__main__":
    main()
