"""Write a routed carrier board's copper geometry as JSON for kicad.measure.

Run with KiCad's bundled Python (3.9):

    $KICAD_PYTHON tools/kicad/pcb/dump.py <board-input.json> <out.json> [--pcb <file>]

``just <project>::carrier-measure`` writes the board input (``python3 -m
kicad.boardinput``), runs this, then ``python3 -m kicad.measure`` on the
result. ``--pcb`` reads another ``.kicad_pcb`` than the input's (the recipe's
``--ref``: the board as committed at a git ref, through ``git show``).

Coordinates are millimetres relative to board.toml's ``[outline] origin``, y
down as in KiCad. The JSON holds, for copper layers F.Cu and B.Cu only:

- ``tracks``: net, layer, width, ends ``a`` and ``b`` (an arc adds ``mid``);
- ``vias``: net, centre, ring ``size``, ``drill`` and the layers it spans;
- ``pads``: reference, number, net, centre, ``tht`` (plated through hole),
  ``drill``, and per copper layer the pad's polygon as KiCad fills it (so a
  rotated or rounded pad is its real shape, not its bounding box);
- ``zones``: per zone and copper layer, the filled islands, each an outline
  and its holes;
- ``holes``: non-plated holes (``MB<n>`` mouse bites, mounting holes) with
  centre and drill; NPTH pads carry no copper and are not in ``pads``;
- ``outline``: the Edge.Cuts outlines and their holes (slots);
- ``spec``: the board input's ``pour``, ``fanout`` and ``silk`` entries, so
  the JSON alone answers every measurement.
"""

import json
import sys

import pcbnew

COPPER = ("F.Cu", "B.Cu")


def mm(v):
    return pcbnew.ToMM(v)


def point(p, origin):
    return [round(mm(p.x) - origin[0], 4), round(mm(p.y) - origin[1], 4)]


def chain(c, origin):
    return [point(c.CPoint(k), origin) for k in range(c.PointCount())]


def poly_set(ps, origin):
    """SHAPE_POLY_SET -> [{"outline": [...], "holes": [[...], ...]}, ...]"""
    out = []
    for i in range(ps.OutlineCount()):
        out.append(
            {
                "outline": chain(ps.Outline(i), origin),
                "holes": [chain(ps.Hole(i, h), origin) for h in range(ps.HoleCount(i))],
            }
        )
    return out


def copper_layers(board, layer_set):
    return [
        board.GetLayerName(lay)
        for lay in layer_set.Seq()
        if board.GetLayerName(lay) in COPPER
    ]


def dump(board, origin):
    d = {
        "origin": list(origin),
        "tracks": [],
        "vias": [],
        "pads": [],
        "zones": [],
        "holes": [],
    }
    for t in board.GetTracks():
        cls = t.GetClass()
        if cls == "PCB_VIA":
            layers = [
                board.GetLayerName(t.TopLayer()),
                board.GetLayerName(t.BottomLayer()),
            ]
            d["vias"].append(
                {
                    "net": t.GetNetname(),
                    "at": point(t.GetPosition(), origin),
                    "size": round(mm(t.GetWidth(pcbnew.F_Cu)), 4),
                    "drill": round(mm(t.GetDrillValue()), 4),
                    "layers": layers,
                    "locked": t.IsLocked(),
                }
            )
            continue
        item = {
            "net": t.GetNetname(),
            "layer": t.GetLayerName(),
            "width": round(mm(t.GetWidth()), 4),
            "a": point(t.GetStart(), origin),
            "b": point(t.GetEnd(), origin),
            "locked": t.IsLocked(),
        }
        if cls == "PCB_ARC":
            item["mid"] = point(t.GetMid(), origin)
        d["tracks"].append(item)

    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for p in fp.Pads():
            attr = p.GetAttribute()
            if attr == pcbnew.PAD_ATTRIB_NPTH:
                d["holes"].append(
                    {
                        "ref": ref,
                        "num": p.GetNumber(),
                        "at": point(p.GetPosition(), origin),
                        "drill": round(mm(p.GetDrillSizeX()), 4),
                    }
                )
                continue
            layers = copper_layers(board, p.GetLayerSet())
            if not layers:
                continue
            shapes = {}
            for name in layers:
                poly = p.GetEffectivePolygon(
                    board.GetLayerID(name), pcbnew.ERROR_INSIDE
                )
                shapes[name] = poly_set(poly, origin)
            d["pads"].append(
                {
                    "ref": ref,
                    "num": p.GetNumber(),
                    "net": p.GetNetname(),
                    "at": point(p.GetPosition(), origin),
                    "tht": attr == pcbnew.PAD_ATTRIB_PTH,
                    "drill": round(mm(p.GetDrillSizeX()), 4),
                    "layers": layers,
                    "shapes": shapes,
                }
            )

    for z in board.Zones():
        if z.GetIsRuleArea():
            continue
        for name in copper_layers(board, z.GetLayerSet()):
            fill = z.GetFilledPolysList(board.GetLayerID(name))
            d["zones"].append(
                {
                    "net": z.GetNetname(),
                    "layer": name,
                    "priority": z.GetAssignedPriority(),
                    "islands": poly_set(fill, origin),
                }
            )

    outline = pcbnew.SHAPE_POLY_SET()
    board.GetBoardPolygonOutlines(outline, False)
    d["outline"] = poly_set(outline, origin)
    return d


def write(board, spec, pcb, out):
    """Dump `board` (loaded from `pcb`) to `out`; returns a one-line summary."""
    d = dump(board, spec["origin"])
    d["pcb"] = pcb
    d["spec"] = {k: spec.get(k) for k in ("pour", "fanout", "silk")}
    with open(out, "w") as f:
        json.dump(d, f)
    return (
        f"dumped {pcb}: {len(d['tracks'])} tracks, {len(d['vias'])} vias, "
        f"{len(d['pads'])} pads, {len(d['zones'])} zone layers, "
        f"{len(d['holes'])} NPTH holes"
    )


def main(argv):
    args = list(argv[1:])
    pcb = None
    if "--pcb" in args:
        i = args.index("--pcb")
        if i + 1 >= len(args):
            sys.exit(__doc__)
        pcb = args[i + 1]
        del args[i : i + 2]
    if len(args) != 2:
        sys.exit(__doc__)
    with open(args[0]) as f:
        spec = json.load(f)
    pcb = pcb or spec["pcb"]
    print(write(pcbnew.LoadBoard(pcb), spec, pcb, args[1]))


if __name__ == "__main__":
    main(sys.argv)
