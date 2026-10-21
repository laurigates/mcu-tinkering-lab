"""Place a carrier board from tools/kicad's board input: footprints, nets, silkscreen.

Run with KiCad's bundled Python (3.9; it provides ``pcbnew``):

    $KICAD_PYTHON tools/kicad/pcb/build_board.py <board-input.json>

``python3 -m kicad.boardinput <project>`` writes the JSON from the hardware
join's carrier netlist and the project's ``hardware/carrier/board.toml``;
``just <project>::carrier-board`` runs both, then route_board.py. This script
decides nothing: parts, footprints (always the vendored copies), symbol paths,
footprint UUIDs, nets (KiCad's ``unconnected-(...)`` net for each no-connect
pin included), placement, silkscreen, keepouts, cutouts and pre-routed tracks
all come from the JSON.

Each keepout becomes a KiCad rule area on every copper layer that allows no
tracks, vias, pads, footprints or zone fill, so DRC, the zone filler and
Freerouting (through the Specctra export) all see it. Each footprint's
silkscreen strokes under JLCPCB's 0.15 mm minimum (KiCad's stock footprints
draw 0.12 mm, polarity marks included) are widened to it (silk.py). Each silkscreen cut line
is drawn on both silkscreen layers with a "CUT" label beside it. Each cutout
is a circle on Edge.Cuts inside the outline: a hole milled through the board.
Each board.toml [[tracks]] entry is laid down as locked segments, which the
Specctra export marks protected, so Freerouting keeps them.

With a mouse-bite panel (board.toml ``[panel] mode = "mousebites"``, #809) the
cut lines are not drawn: each slot is a rectangle on Edge.Cuts, milled out, and
each tab a board-only footprint (MB1, MB2, ...) of non-plated holes, which go to
the drill file. A tab's ``clear`` rectangle is cut out of every keepout rule
area it overlaps: a rule area that forbids pads forbids a non-plated hole too.

It rewrites the board from scratch, so routing (route_board.py) must run after
it. Python 3.9 and the stdlib plus ``pcbnew`` and its sibling ``silk`` only; it
imports nothing from tools/kicad (see tools/kicad/test_pcb_py39.py).
"""

import json
import math
import os
import shutil
import sys
import tempfile

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import silk


def mm(v):
    return pcbnew.FromMM(v)


# One reader for every load. pcbnew.FootprintLoad() works as well: the untyped
# SwigPyObject errors once blamed on it came from board.Remove() (see build()
# and the carrier-pcb skill, trap 1), and 200 FootprintLoad() calls without
# it all return FOOTPRINT.
SEXPR_IO = pcbnew.PCB_IO_KICAD_SEXPR()


def load_footprint(comp):
    lib, name = comp["footprint"].split(":", 1)
    fp = SEXPR_IO.FootprintLoad(comp["fp_dir"], name)
    if fp is None:
        sys.exit(f"footprint {comp['footprint']} not found in {comp['fp_dir']}")
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def widen_silk(fp):
    """Each silkscreen line, shape and text of a footprint at the 0.15 mm floor.

    Returns how many items were widened.
    """
    layers = (pcbnew.F_SilkS, pcbnew.B_SilkS)
    n = 0
    for item in [*fp.GraphicalItems(), fp.Reference(), fp.Value()]:
        if item.GetLayer() not in layers:
            continue
        if item.GetClass() == "PCB_SHAPE":
            w = item.GetWidth()
            if silk.widened(w) != w:
                item.SetWidth(silk.widened(w))
                n += 1
        elif hasattr(item, "GetTextThickness"):
            w = item.GetTextThickness()
            if silk.widened(w) != w:
                item.SetTextThickness(silk.widened(w))
                n += 1
    return n


def rect(board, origin, x1, y1, x2, y2, layer, width):
    x0, y0 = origin
    shape = pcbnew.PCB_SHAPE(board)
    shape.SetShape(pcbnew.SHAPE_T_RECT)
    shape.SetStart(pcbnew.VECTOR2I(mm(x0 + x1), mm(y0 + y1)))
    shape.SetEnd(pcbnew.VECTOR2I(mm(x0 + x2), mm(y0 + y2)))
    shape.SetLayer(layer)
    shape.SetWidth(mm(width))
    board.Add(shape)


def silk_text(board, origin, text, x, y, size=1.0):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(pcbnew.VECTOR2I(mm(origin[0] + x), mm(origin[1] + y)))
    t.SetLayer(pcbnew.F_SilkS)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    # JLCPCB's silkscreen minimums are ~1.0 mm height and ~0.15 mm stroke.
    t.SetTextThickness(mm(max(0.16, size * 0.15)))
    board.Add(t)


# A cut line's "CUT" label sits this far from the line, on its left-hand side.
CUT_LABEL_OFFSET = 1.0
CUT_LABEL_SIZE = 1.0
CUT_LINE_WIDTH = 0.15


def keepout(board, origin, k, clears=()):
    """A rule area over board.toml keepout `k`: no copper of any kind inside,
    less each mouse-bite tab's `clears` rectangle."""
    x0, y0 = origin
    zone = pcbnew.ZONE(board)
    zone.SetIsRuleArea(True)
    zone.SetDoNotAllowTracks(True)
    zone.SetDoNotAllowVias(True)
    zone.SetDoNotAllowPads(True)
    zone.SetDoNotAllowFootprints(True)
    zone.SetDoNotAllowZoneFills(True)
    zone.SetLayerSet(pcbnew.LSET.AllCuMask(board.GetCopperLayerCount()))
    zone.SetZoneName(k["name"])
    rx0, ry0, rx1, ry1 = k["rect"]
    outline = zone.Outline()
    outline.NewOutline()
    for x, y in ((rx0, ry0), (rx1, ry0), (rx1, ry1), (rx0, ry1)):
        outline.Append(mm(x0 + x), mm(y0 + y))
    for cx0, cy0, cx1, cy1 in clears:
        if cx0 >= rx1 or rx0 >= cx1 or cy0 >= ry1 or ry0 >= cy1:
            continue
        cut = pcbnew.SHAPE_POLY_SET()
        cut.NewOutline()
        for x, y in ((cx0, cy0), (cx1, cy0), (cx1, cy1), (cx0, cy1)):
            cut.Append(mm(x0 + x), mm(y0 + y))
        outline.BooleanSubtract(cut)
    board.Add(zone)


def mouse_bite_tab(board, origin, tab, hole_mm):
    """A board-only footprint of non-plated holes: one mouse-bite tab."""
    x0, y0 = origin
    fp = pcbnew.FOOTPRINT(board)
    # A named, library-less id: with an empty one KiCad's Specctra export names
    # the image "", Freerouting writes it back unquoted, and the session import
    # fails. No library nickname, so DRC's library check skips it.
    fp.SetFPID(pcbnew.LIB_ID("", "MouseBites"))
    fp.SetReference(tab["name"])
    fp.SetValue("MouseBites")
    fp.SetAttributes(
        pcbnew.FP_BOARD_ONLY
        | pcbnew.FP_EXCLUDE_FROM_BOM
        | pcbnew.FP_EXCLUDE_FROM_POS_FILES
    )
    fp.SetUuid(pcbnew.KIID(tab["uuid"]))
    ax, ay = tab["at"]
    fp.SetPosition(pcbnew.VECTOR2I(mm(x0 + ax), mm(y0 + ay)))
    board.Add(fp)
    fp.Reference().SetVisible(False)
    fp.Value().SetVisible(False)
    for hx, hy in tab["holes"]:
        pad = pcbnew.PAD(fp)
        pad.SetAttribute(pcbnew.PAD_ATTRIB_NPTH)
        pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
        pad.SetSize(pcbnew.VECTOR2I(mm(hole_mm), mm(hole_mm)))
        pad.SetDrillSize(pcbnew.VECTOR2I(mm(hole_mm), mm(hole_mm)))
        pad.SetLayerSet(pcbnew.PAD.UnplatedHoleMask())
        pad.SetNumber("")
        fp.Add(pad)
        pad.SetPosition(pcbnew.VECTOR2I(mm(x0 + hx), mm(y0 + hy)))


def fixed_track(board, origin, tr):
    """board.toml [[tracks]] entry `tr`, as locked segments: Freerouting keeps
    a locked track (the DSN marks it protected) and routes around it."""
    x0, y0 = origin
    net = board.FindNet(tr["net"])
    if net is None:
        sys.exit(f"[[tracks]] {tr['name']!r}: net {tr['net']!r} is not on the board")
    layer = board.GetLayerID(tr["layer"])
    pts = tr["points"]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        seg = pcbnew.PCB_TRACK(board)
        seg.SetStart(pcbnew.VECTOR2I(mm(x0 + ax), mm(y0 + ay)))
        seg.SetEnd(pcbnew.VECTOR2I(mm(x0 + bx), mm(y0 + by)))
        seg.SetWidth(mm(tr["width_mm"]))
        seg.SetLayer(layer)
        seg.SetNet(net)
        seg.SetLocked(True)
        board.Add(seg)


def cutout(board, origin, c):
    """A circular hole through the board: board.toml cutout `c` on Edge.Cuts."""
    x0, y0 = origin
    cx, cy = c["center"]
    shape = pcbnew.PCB_SHAPE(board)
    shape.SetShape(pcbnew.SHAPE_T_CIRCLE)
    shape.SetCenter(pcbnew.VECTOR2I(mm(x0 + cx), mm(y0 + cy)))
    shape.SetEnd(pcbnew.VECTOR2I(mm(x0 + cx + c["diameter"] / 2), mm(y0 + cy)))
    shape.SetLayer(pcbnew.Edge_Cuts)
    shape.SetWidth(mm(0.1))
    board.Add(shape)


def cut_line(board, origin, line):
    """The saw line on F.SilkS and B.SilkS, each with a "CUT" label beside it."""
    x0, y0 = origin
    ax, ay, bx, by = line
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    # Readable text: an angle in (-90, 90]; KiCad angles run counter-clockwise
    # with y down.
    angle = -math.degrees(math.atan2(dy, dx))
    if angle <= -90:
        angle += 180
    elif angle > 90:
        angle -= 180
    nx, ny = dy / length, -dx / length
    mx = (ax + bx) / 2 + nx * CUT_LABEL_OFFSET
    my = (ay + by) / 2 + ny * CUT_LABEL_OFFSET
    for layer, mirrored in ((pcbnew.F_SilkS, False), (pcbnew.B_SilkS, True)):
        shape = pcbnew.PCB_SHAPE(board)
        shape.SetShape(pcbnew.SHAPE_T_SEGMENT)
        shape.SetStart(pcbnew.VECTOR2I(mm(x0 + ax), mm(y0 + ay)))
        shape.SetEnd(pcbnew.VECTOR2I(mm(x0 + bx), mm(y0 + by)))
        shape.SetLayer(layer)
        shape.SetWidth(mm(CUT_LINE_WIDTH))
        board.Add(shape)
        t = pcbnew.PCB_TEXT(board)
        t.SetText("CUT")
        t.SetPosition(pcbnew.VECTOR2I(mm(x0 + mx), mm(y0 + my)))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(mm(CUT_LABEL_SIZE), mm(CUT_LABEL_SIZE)))
        t.SetTextThickness(mm(0.16))
        t.SetTextAngleDegrees(angle)
        t.SetMirrored(mirrored)
        board.Add(t)


def place(board, spec):
    """Load, annotate and place every component; ref -> footprint."""
    origin = spec["origin"]
    silk = spec["silk"]
    hidden = set(silk["hidden_refs"])
    footprints = {}
    for comp in spec["components"]:
        ref = comp["ref"]
        fp = load_footprint(comp)
        widen_silk(fp)
        fp.SetReference(ref)
        fp.SetValue(comp["value"])
        fp.SetPath(pcbnew.KIID_PATH(comp["path"]))
        # KiCad saves footprints, and exports the DSN, in UUID order; a fixed
        # UUID makes every run order them the same (#763).
        fp.SetUuid(pcbnew.KIID(comp["uuid"]))
        # Custom symbol fields (LCSC) go onto the footprint for the BOM and DRC
        # parity. SetField() creates them visible on F.Silkscreen, so hide them.
        for name, text in comp["fields"]:
            if not text:
                continue
            fp.SetField(name, text)
            fp.GetField(name).SetLayer(pcbnew.F_Fab)
            fp.GetField(name).SetVisible(False)
        x, y, rot = comp["at"][:3]
        fp.SetPosition(pcbnew.VECTOR2I(mm(origin[0] + x), mm(origin[1] + y)))
        fp.SetOrientationDegrees(rot)
        board.Add(fp)
        if comp["at"][3:] == ["back"]:
            # KiCad's own flip, about the anchor; with the rotation then set as
            # seen from the top, the footprint ends up mirrored top to bottom.
            # Flip() before Add() segfaults: it needs the board's layer stack.
            # The silk below comes after it, or the flip would mirror a
            # reference position too.
            fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
            fp.SetOrientationDegrees(rot)
        if ref in hidden:
            fp.Reference().SetVisible(False)
        if ref in silk["ref_positions"]:
            rx, ry = silk["ref_positions"][ref]
            fp.Reference().SetPosition(
                pcbnew.VECTOR2I(mm(origin[0] + rx), mm(origin[1] + ry))
            )
        if ref in silk["ref_sizes"]:
            size = mm(silk["ref_sizes"][ref])
            fp.Reference().SetTextSize(pcbnew.VECTOR2I(size, size))
            fp.Reference().SetTextThickness(max(mm(0.16), int(size * 0.15)))
        footprints[ref] = fp
    return footprints


def assign_nets(board, spec, footprints):
    for net in spec["nets"]:
        info = pcbnew.NETINFO_ITEM(board, net["name"])
        board.Add(info)
        for ref, pin in net["nodes"]:
            if ref not in footprints:
                sys.exit(f"net {net['name']}: {ref} is not a component")
            pads = [p for p in footprints[ref].Pads() if p.GetNumber() == pin]
            if not pads:
                sys.exit(f"{ref} has no pad {pin}")
            for pad in pads:
                pad.SetNet(info)


def save(board, pcb):
    """board.Save(pcb), keeping the hand-kept .kicad_pro beside it byte for byte.

    Saving a NewBoard() board over an existing project rewrites that project's
    schematic.top_level_sheets to [] (KiCad 10.0.6), which unlinks the
    schematic and fails `kicad.schematic --check`.
    """
    pro = os.path.splitext(pcb)[0] + ".kicad_pro"
    kept = None
    if os.path.exists(pro):
        with open(pro, "rb") as f:
            kept = f.read()
    board.Save(pcb)
    if kept is not None:
        with open(pro, "wb") as f:
            f.write(kept)


def build(spec):
    origin = spec["origin"]
    width, height = spec["size"]
    # A fresh board every run. Clearing a loaded one does not work: after
    # board.Remove() of its footprints, SWIG returns untyped objects for every
    # later call (KiCad 10.0.6). NewBoard() also writes a .kicad_pro beside the
    # file, so it is created in a scratch directory and saved to the pcb path.
    scratch = tempfile.mkdtemp(prefix="carrier-")
    try:
        board = pcbnew.NewBoard(os.path.join(scratch, "carrier.kicad_pcb"))
        rect(board, origin, 0, 0, width, height, pcbnew.Edge_Cuts, 0.1)
        for c in spec["cutouts"]:
            cutout(board, origin, c)
        panel = spec.get("panel")
        for x1, y1, x2, y2 in panel["slots"] if panel else ():
            rect(board, origin, x1, y1, x2, y2, pcbnew.Edge_Cuts, 0.1)
        for x1, y1, x2, y2 in spec["silk"]["rects"]:
            rect(board, origin, x1, y1, x2, y2, pcbnew.F_SilkS, 0.15)
        footprints = place(board, spec)
        assign_nets(board, spec, footprints)
        for text in spec["silk"]["texts"]:
            x, y = text["at"]
            silk_text(board, origin, text["text"], x, y, text["size"])
        if panel:
            for tab in panel["tabs"]:
                mouse_bite_tab(board, origin, tab, panel["hole_mm"])
        else:
            for line in spec["silk"]["cut_lines"]:
                cut_line(board, origin, line)
        clears = [tab["clear"] for tab in panel["tabs"]] if panel else []
        for k in spec["keepouts"]:
            keepout(board, origin, k, clears)
        for tr in spec.get("tracks", []):
            fixed_track(board, origin, tr)
        save(board, spec["pcb"])
    finally:
        shutil.rmtree(scratch)
    return len(footprints)


def main(argv):
    if len(argv) != 2:
        sys.exit(__doc__)
    with open(argv[1]) as f:
        spec = json.load(f)
    n = build(spec)
    print(f"placed {n} footprints, wrote {spec['pcb']}")


if __name__ == "__main__":
    main(sys.argv)
