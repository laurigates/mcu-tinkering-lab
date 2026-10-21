"""Report the routed board's layout metrics a pre-fab review checks by hand.

Run with KiCad's bundled Python (3.9) after route_board.py:

    $KICAD_PYTHON tools/kicad/pcb/review.py <board-input.json> [--dump <out.json>]

``just <project>::carrier-review`` writes the JSON (``python3 -m
kicad.boardinput``) and runs it. It reads the committed ``.kicad_pcb`` and
prints, without failing on any of it:

- vias per poured net, and each SMD pad of a poured net with no via of its net
  inside it or within ``REACH_MM`` of it;
- the pour's islands per net and layer;
- per net class net, its narrowest track and how many tracks it has;
- each decoupling capacitor's (two pads, value at most ``DECAP_MAX_F``)
  distance from its non-ground pad to the nearest pad of the same net on an IC
  (``U`` reference, or ``A``: a breakout module socket), flagged past ``DECAP_MM``; bulk capacitors are placed by
  power path, not by pin, and are left out;
- for each [silk] cut line, the nearest copper (pad, track, via, zone fill) and
  courtyard, flagged inside ``CUT_MM``;
- the smallest silkscreen text height and stroke, flagged under JLCPCB's
  1.0 mm / 0.15 mm.

``--dump`` also writes the board's copper as JSON (``dump.py``, from the board
already loaded), and the recipe then prints the plane-connectivity section
from it with ``python3 -m kicad.measure <out.json> plane --review``: per
poured net, the pieces of pour that vias and through-hole pads alone do not
join, flagged when more than one is 5 mm2 or larger. That geometry lives in
``kicad.measure`` (stdlib, Python 3.11), not here: KiCad's Python has no
polygon library, and one implementation serves both reports.

The integrated robocar carrier passed every gate with one GND via on its main
board (the carrier-pcb skill, trap 29): the gates check what is connected, not
how well. This is the "how well", for a person or a reviewing agent to read.
"""

import json
import math
import sys

import dump
import pcbnew

REACH_MM = 2.5
DECAP_MM = 3.0
DECAP_MAX_F = 1e-6
CUT_MM = 1.0
SILK_H_MM = 1.0
SILK_W_MM = 0.15


def mm(v):
    return pcbnew.ToMM(v)


def ref(item):
    fp = item.GetParentFootprint()
    return fp.GetReference() if fp else "?"


def dist_point_seg(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    n = dx * dx + dy * dy
    t = 0.0 if n == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / n))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def dist_box_seg(box, a, b):
    """Distance in mm between a BOX2I (board units) and segment a-b (mm)."""
    x0, y0 = mm(box.GetLeft()), mm(box.GetTop())
    x1, y1 = mm(box.GetRight()), mm(box.GetBottom())
    ax, ay = a
    bx, by = b
    # inside or crossing: sample the segment against the box
    for k in range(65):
        t = k / 64.0
        x, y = ax + t * (bx - ax), ay + t * (by - ay)
        if x0 <= x <= x1 and y0 <= y <= y1:
            return 0.0
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    best = min(dist_point_seg(cx, cy, ax, ay, bx, by) for cx, cy in corners)
    for px, py in (a, b):
        dx = max(x0 - px, 0.0, px - x1)
        dy = max(y0 - py, 0.0, py - y1)
        best = min(best, math.hypot(dx, dy))
    return best


def farads(value):
    """'100nF' / '22uF 16V' / '4.7u' -> farads, or None."""
    word = value.split()[0] if value.split() else ""
    word = word.replace("\u00b5", "u").rstrip("Ff")
    scale = {"p": 1e-12, "n": 1e-9, "u": 1e-6, "m": 1e-3}
    try:
        if word and word[-1] in scale:
            return float(word[:-1]) * scale[word[-1]]
        return float(word)
    except ValueError:
        return None


def review(spec, dump_to=None):
    board = pcbnew.LoadBoard(spec["pcb"])
    if dump_to:
        print(dump.write(board, spec, spec["pcb"], dump_to))
    ox, oy = spec["origin"]
    pads = [p for fp in board.GetFootprints() for p in fp.Pads()]
    tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    poured = sorted({r["net"] for r in spec["pour"]["regions"]})
    flags = 0

    print("== vias per poured net")
    for net in poured:
        print(f"   {net}: {sum(1 for v in vias if v.GetNetname() == net)}")

    print(f"== SMD pads of poured nets with no via within {REACH_MM} mm")
    reach = pcbnew.FromMM(REACH_MM)
    bare = []
    for p in pads:
        if p.GetNetname() not in poured or p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
            continue
        box = p.GetBoundingBox()
        box.Inflate(reach)
        if not any(
            v.GetNetCode() == p.GetNetCode() and box.Contains(v.GetPosition())
            for v in vias
        ):
            bare.append(f"{ref(p)}:{p.GetNumber()}")
    print("   " + (", ".join(sorted(bare)) if bare else "none"))
    flags += bool(bare)

    print("== pour islands per net and layer")
    for z in board.Zones():
        if z.GetIsRuleArea():
            continue
        for layer in z.GetLayerSet().Seq():
            n = z.GetFilledPolysList(layer).OutlineCount()
            print(f"   {z.GetNetname()} {board.GetLayerName(layer)}: {n}")

    print("== narrowest track per class net")
    for cls in spec["classes"]:
        for net in cls["nets"]:
            ws = [mm(t.GetWidth()) for t in tracks if t.GetNetname() == net]
            if ws:
                note = (
                    "" if min(ws) >= cls["track_um"] / 1000.0 - 1e-6 else "  (necked)"
                )
                print(
                    f"   {cls['name']} {net}: {min(ws):.3f} mm over {len(ws)} tracks{note}"
                )

    print(
        f"== decoupling (<= 1 uF): capacitor to nearest IC pin of its net "
        f"(flag > {DECAP_MM} mm)"
    )
    ic_pads = [p for p in pads if ref(p)[:1] in ("U", "A")]
    for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
        if not fp.GetReference().startswith("C") or len(list(fp.Pads())) != 2:
            continue
        f = farads(fp.GetValue())
        if f is None or f > DECAP_MAX_F:
            continue
        for p in fp.Pads():
            if p.GetNetname() in poured or p.GetNetname() == "":
                continue
            same = [q for q in ic_pads if q.GetNetCode() == p.GetNetCode()]
            if not same:
                continue
            q = min(
                same, key=lambda q: (q.GetPosition() - p.GetPosition()).EuclideanNorm()
            )
            d = mm(int((q.GetPosition() - p.GetPosition()).EuclideanNorm()))
            mark = "  <-- far" if d > DECAP_MM else ""
            flags += d > DECAP_MM
            print(
                f"   {fp.GetReference()} {p.GetNetname()} -> {ref(q)}:{q.GetNumber()} {d:.1f} mm{mark}"
            )

    lines = spec["silk"].get("cut_lines", [])
    if lines:
        print(f"== distance to each cut line (flag < {CUT_MM} mm)")
    for line in lines:
        a = (ox + line[0], oy + line[1])
        b = (ox + line[2], oy + line[3])
        copper = [
            (dist_box_seg(p.GetBoundingBox(), a, b), f"pad {ref(p)}:{p.GetNumber()}")
            for p in pads
            if p.GetNetname()
        ]
        copper += [
            (
                dist_box_seg(t.GetBoundingBox(), a, b),
                f"{t.GetClass()[4:].lower()} {t.GetNetname()}",
            )
            for t in tracks + vias
        ]
        for z in board.Zones():
            if z.GetIsRuleArea():
                continue
            for layer in z.GetLayerSet().Seq():
                fill = z.GetFilledPolysList(layer)
                for i in range(fill.OutlineCount()):
                    o = fill.Outline(i)
                    for k in range(o.PointCount()):
                        pt = o.CPoint(k)
                        d = dist_point_seg(mm(pt.x), mm(pt.y), a[0], a[1], b[0], b[1])
                        copper.append((d, f"{z.GetNetname()} fill"))
        court = []
        for fp in board.GetFootprints():
            box = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
            if box.GetWidth() == 0:
                box = fp.GetCourtyard(pcbnew.B_CrtYd).BBox()
            if box.GetWidth():
                court.append((dist_box_seg(box, a, b), fp.GetReference()))
        dc, wc = min(copper) if copper else (math.inf, "-")
        dk, wk = min(court) if court else (math.inf, "-")
        bad = dc < CUT_MM or dk < CUT_MM
        flags += bad
        print(
            f"   {line}: copper {dc:.2f} mm ({wc}), courtyard {dk:.2f} mm ({wk}){'  <-- close' if bad else ''}"
        )

    print(
        f"== silkscreen text (JLCPCB legible: {SILK_H_MM} mm tall, {SILK_W_MM} mm stroke)"
    )
    texts = []
    for fp in board.GetFootprints():
        for t in (fp.Reference(), fp.Value()):
            if t.IsVisible() and t.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS):
                texts.append(
                    (mm(t.GetTextHeight()), mm(t.GetTextThickness()), fp.GetReference())
                )
    for d in board.GetDrawings():
        if d.GetClass() == "PCB_TEXT" and d.GetLayer() in (
            pcbnew.F_SilkS,
            pcbnew.B_SilkS,
        ):
            texts.append(
                (mm(d.GetTextHeight()), mm(d.GetTextThickness()), repr(d.GetText()))
            )
    if texts:
        h = min(texts)
        w = min(texts, key=lambda t: t[1])
        small = sorted({n for hh, _, n in texts if hh < SILK_H_MM - 1e-6})
        print(
            f"   smallest height {h[0]:.2f} mm ({h[2]}), thinnest stroke {w[1]:.2f} mm ({w[2]})"
        )
        if small:
            print(f"   under {SILK_H_MM} mm: {', '.join(small)}")
        flags += w[1] < SILK_W_MM - 1e-6
    print(f"== {flags} flag(s); this report does not gate (carrier-check does)")


def main(argv):
    args = list(argv[1:])
    dump_to = None
    if "--dump" in args:
        i = args.index("--dump")
        if i + 1 >= len(args):
            sys.exit(__doc__)
        dump_to = args[i + 1]
        del args[i : i + 2]
    if len(args) != 1:
        sys.exit(__doc__)
    with open(args[0]) as f:
        review(json.load(f), dump_to)


if __name__ == "__main__":
    main(sys.argv)
