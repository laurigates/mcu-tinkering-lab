"""Autoroute a placed carrier board with Freerouting, then pour copper.

Run with KiCad's bundled Python (3.9) after build_board.py:

    $KICAD_PYTHON tools/kicad/pcb/route_board.py <board-input.json>

``just <project>::carrier-board`` writes the JSON (``python3 -m
kicad.boardinput``) and runs both scripts. Steps: export Specctra DSN, rewrite
its single net class into board.toml's width classes (dsn.py), run Freerouting
headless (board.toml's ``[[tracks]]`` and ``[[vias]]`` reach it as the export's
fixed wiring, each end snapped onto its pin's exact centre; each [fanout]
in_pad pad leaves its net's pin list, since its in-pad via joins it to the
pour), import the session, add the [fanout] vias, check every pad of
each pour net has a routed track, then add a zone per board.toml ``[pour]``
region on each of its layers
(the region's own ``layers``, else every ``[pour]`` layer) and fill them, then
add ``[pour] stitch`` vias where the fill takes them and fill again. A
region's ``rect`` (relative to the outline origin) is its zone outline; a null
``rect`` is the whole board. Keepouts are already rule areas
on the board (build_board.py), so the export and the fill respect them.

Freerouting does not route the same way twice: commit the board that passed
``just <project>::carrier-check``, not a re-run of it. Needs java >= 25 and the
Freerouting 2.5.0 jar (``freerouting_jar`` in the JSON, from $FREEROUTING_JAR).
"""

import json
import math
import os
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dsn
import fanout as fanout_geom
import stitch as stitch_geom


def region_box(board, origin, rect):
    """(left, top, right, bottom) of a pour region in board units."""
    if rect is None:
        box = board.GetBoardEdgesBoundingBox()
        return box.GetLeft(), box.GetTop(), box.GetRight(), box.GetBottom()
    x0, y0 = origin
    return tuple(
        pcbnew.FromMM(v)
        for v in (x0 + rect[0], y0 + rect[1], x0 + rect[2], y0 + rect[3])
    )


def pour_zone(board, layer, pour, net_name, box, priority):
    net = board.FindNet(net_name)
    zone = pcbnew.ZONE(board)
    zone.SetLayer(layer)
    # Regions may share an edge (two rectangles of one net around a corner),
    # and KiCad's DRC reports touching zones of equal priority as intersecting.
    # Region n gets priority n; a single region keeps KiCad's default, 0.
    zone.SetAssignedPriority(priority)
    zone.SetNetCode(net.GetNetCode())
    zone.SetLocalClearance(pcbnew.FromMM(pour["clearance_mm"]))
    zone.SetMinThickness(pcbnew.FromMM(pour["min_thickness_mm"]))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    # Narrower than KiCad's 0.5 mm defaults so two spokes still fit between
    # 2.54 mm-pitch header pins and their signal tracks (balancebot's J3 pad 7
    # was reported as a starved thermal at the defaults).
    zone.SetThermalReliefGap(pcbnew.FromMM(pour["thermal_gap_mm"]))
    zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(pour["spoke_mm"]))
    left, top, right, bottom = box
    outline = zone.Outline()
    outline.NewOutline()
    for x, y in ((left, top), (right, top), (right, bottom), (left, bottom)):
        outline.Append(x, y)
    board.Add(zone)
    return zone


def pour_layers(board, pour):
    layers = []
    for name in pour["layers"]:
        layer = board.GetLayerID(name)
        if layer < 0:
            sys.exit(f"[pour] layer {name!r} is not a layer of this board")
        layers.append(layer)
    return layers


def _at(origin, point):
    return pcbnew.VECTOR2I(
        pcbnew.FromMM(origin[0] + point[0]), pcbnew.FromMM(origin[1] + point[1])
    )


def end_pads(board, origin, tr, point):
    """Every pad of `tr`'s net and layer under `point`.

    Usually one; a footprint can stack two pads of one net at one spot (a
    USB-C receptacle's A1/B12), and all of them are under the point.
    """
    at = _at(origin, point)
    layer = board.GetLayerID(tr["layer"])
    return [
        pad
        for fp in board.GetFootprints()
        for pad in fp.Pads()
        if pad.GetNetname() == tr["net"] and pad.IsOnLayer(layer) and pad.HitTest(at)
    ]


def end_vias(board, origin, tr, point):
    """The locked [[vias]] of `tr`'s net under `point`."""
    at = _at(origin, point)
    return [
        t
        for t in board.GetTracks()
        if t.GetClass() == "PCB_VIA"
        and t.IsLocked()
        and t.GetNetname() == tr["net"]
        and t.HitTest(at)
    ]


def check_track_ends(board, origin, tracks):
    """Exit unless each track's two ends sit on a pad or a [[vias]] via of its
    net, and not both on one pad: a fixed track joins two things, and an end
    in open copper is a stub nothing routes to."""
    for tr in tracks:
        ends = []
        for point in (tr["points"][0], tr["points"][-1]):
            pads = end_pads(board, origin, tr, point)
            if not pads and not end_vias(board, origin, tr, point):
                sys.exit(
                    f"[[tracks]] {tr['name']!r}: {point} is on no {tr['net']} pad "
                    f"on {tr['layer']} and no [[vias]] via"
                )
            ends.append(
                {(p.GetParentFootprint().GetReference(), p.GetNumber()) for p in pads}
            )
        if ends[0] & ends[1]:
            sys.exit(f"[[tracks]] {tr['name']!r} starts and ends on one pad")


def _bbox_near(item, x, y, reach):
    box = item.GetBoundingBox()
    return (
        box.GetLeft() - reach <= x <= box.GetRight() + reach
        and box.GetTop() - reach <= y <= box.GetBottom() + reach
    )


def fanout(board, fan, clearance):
    """Add a via, and a stub track to it, beside each SMD pad of a [fanout] net.

    Runs on the routed board, before the pour: Freerouting has already joined
    every pad with tracks, so a via is a second connection, to the other
    layer's pour, wherever there is room for one. A pad already within `reach_mm`
    of a via of its net keeps that one; an `in_pad` pad (an exposed pad) takes
    its via at its centre; an `exempt` pad takes none; an `own` pad takes a via
    of its own whatever chain already reaches one. Each candidate from
    fanout.py must clear every item of another net, and every via and
    through-hole pad of any net, by `clearance` on both layers, keep out of the
    rule areas and 0.5 mm in from the board edge; its stub (`track_um` wide,
    or the pad's narrow side if that is less) must clear every
    item of another net on the pad's layer. A pad counts as having a via when
    one is inside it, or a chain of fixed tracks and fanout stubs of its net,
    through any pads of its net on the way, joins it to a via (an exposed
    pad's ground pins, the buck's GND pin through C11's). A candidate pointing
    away from the
    pad's own part is tried first. Returns (vias added, the pads that found no
    place).
    """
    if not fan:
        return 0, []
    mm = pcbnew.FromMM
    via_d = mm(fan["via_um"][0] / 1000.0)
    drill = mm(fan["via_um"][1] / 1000.0)
    r = via_d // 2
    width = mm(fan["track_um"] / 1000.0)
    reach = mm(fan["reach_mm"])
    gap = mm(0.2)  # solder-mask web between the pad and the via
    step = mm(0.25)
    copper = (pcbnew.F_Cu, pcbnew.B_Cu)
    edge = board.GetBoardEdgesBoundingBox()
    inset = mm(0.5) + r
    rule_areas = [z for z in board.Zones() if z.GetIsRuleArea()]
    in_pad = set(fan["in_pad"])
    exempt = set(fan["exempt"])
    own = set(fan["own"])
    nets = set(fan["nets"])

    pads = [p for fp in board.GetFootprints() for p in fp.Pads()]
    obstacles = pads + list(board.GetTracks())

    def blocks(item, netcode, through, own):
        # Whether `item` must be cleared by a new via (through) or stub. A via
        # clears every pad but its own, its own net's SMD pads included: in a
        # neighbouring pin's pad the hole drains that pin's paste at reflow
        # (U7:4's via landed in U7:3; second pre-fab layout review, S3).
        if item.GetNetCode() != netcode:
            return True
        if not through:
            return False
        if item.GetClass() == "PCB_VIA":
            return True
        return item.GetClass() == "PAD" and item is not own

    def clear(shape, layers, netcode, x, y, through, own=None):
        for item in obstacles:
            if not blocks(item, netcode, through, own):
                continue
            if not _bbox_near(item, x, y, reach + via_d + clearance):
                continue
            for layer in layers:
                if item.IsOnLayer(layer) and item.GetEffectiveShape(layer).Collide(
                    shape, clearance
                ):
                    return False
        return True

    def via_fits(x, y, netcode, own):
        if not (edge.GetLeft() + inset <= x <= edge.GetRight() - inset):
            return False
        if not (edge.GetTop() + inset <= y <= edge.GetBottom() - inset):
            return False
        at = pcbnew.VECTOR2I(int(x), int(y))
        if any(z.Outline().Collide(at, r) for z in rule_areas):
            return False
        return clear(pcbnew.SHAPE_CIRCLE(at, r), copper, netcode, x, y, True, own)

    stub_ids = set()

    def add_via(x, y, net, size=None):
        diameter, hole = size if size else (via_d, drill)
        via = pcbnew.PCB_VIA(board)
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetPosition(pcbnew.VECTOR2I(int(x), int(y)))
        via.SetWidth(pcbnew.F_Cu, diameter)
        via.SetDrill(hole)
        via.SetNet(net)
        board.Add(via)
        obstacles.append(via)

    def name_of(p):
        return f"{p.GetParentFootprint().GetReference()}:{p.GetNumber()}"

    def vias_of(netcode):
        return [
            t
            for t in obstacles
            if t.GetClass() == "PCB_VIA" and t.GetNetCode() == netcode
        ]

    def has_via(pad):
        # A via inside the pad, or a chain of the pad's own net's tracks (and
        # the pads they pass through) from the pad to a via.
        netcode = pad.GetNetCode()
        vias = vias_of(netcode)
        same = [p for p in pads if p.GetNetCode() == netcode]
        # board.toml's fixed tracks and the stubs added here, not Freerouting's
        # tracks: through those every pad of the net reaches some via
        tracks = [
            t
            for t in obstacles
            if t.GetClass() == "PCB_TRACK"
            and t.GetNetCode() == netcode
            and (t.IsLocked() or id(t) in stub_ids)
        ]
        seen_pads, todo = set(), [pad]
        while todo:
            node = todo.pop()
            if id(node) in seen_pads:
                continue
            seen_pads.add(id(node))
            if any(node.HitTest(v.GetPosition()) for v in vias):
                return True
            # walk each track chain leaving this pad
            ends = [
                b
                for t in tracks
                for a, b in ((t.GetStart(), t.GetEnd()), (t.GetEnd(), t.GetStart()))
                if node.HitTest(a)
            ]
            seen = set()
            while ends:
                at = ends.pop()
                key = (at.x, at.y)
                if key in seen:
                    continue
                seen.add(key)
                if any(v.GetPosition() == at for v in vias):
                    return True
                todo += [q for q in same if q.HitTest(at)]
                for t in tracks:
                    a, b = t.GetStart(), t.GetEnd()
                    if (a.x, a.y) == key:
                        ends.append(b)
                    elif (b.x, b.y) == key:
                        ends.append(a)
        return False

    added, failed = 0, []
    targets = sorted(
        (
            p
            for p in pads
            if p.GetNetname() in nets and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
        ),
        # in-pad vias first: the pins tied to an exposed pad rely on its via
        key=lambda p: (
            name_of(p) not in in_pad,
            p.GetParentFootprint().GetReference(),
            p.GetNumber(),
        ),
    )
    for pad in targets:
        name = name_of(pad)
        if name in exempt:
            continue
        netcode = pad.GetNetCode()
        box = pad.GetBoundingBox()
        cx, cy = pad.GetPosition().x, pad.GetPosition().y
        if name in in_pad:
            several = (fan.get("in_pad_vias") or {}).get(name)
            if several:
                # several vias of their own size, turned with the part; each
                # must land inside the pad (#1280: one via carried a charger's
                # whole ground)
                size = (
                    mm(several["via_um"][0] / 1000.0),
                    mm(several["via_um"][1] / 1000.0),
                )
                deg = pad.GetParentFootprint().GetOrientationDegrees()
                points = fanout_geom.in_pad_points(
                    pcbnew.ToMM(cx), pcbnew.ToMM(cy), deg, several["at"]
                )
                for x, y in points:
                    at = pcbnew.VECTOR2I(mm(x), mm(y))
                    if not pad.HitTest(at):
                        sys.exit(
                            f"[fanout] in_pad_vias {name!r}: ({x:.2f}, {y:.2f}) mm is "
                            "outside the pad"
                        )
                    if not any(v.GetPosition() == at for v in vias_of(netcode)):
                        add_via(at.x, at.y, pad.GetNet(), size)
                        added += 1
            elif not any(box.Contains(v.GetPosition()) for v in vias_of(netcode)):
                add_via(cx, cy, pad.GetNet())
                added += 1
            continue
        if name in own:
            # its own via: one inside the pad's bounding box counts; a stub
            # from an earlier fanout run does not, so run fanout once per board
            if any(box.Contains(v.GetPosition()) for v in vias_of(netcode)):
                continue
        elif has_via(pad):
            continue
        layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        placed = False
        part = pad.GetParentFootprint().GetPosition()
        # no wider than the pad: a fine-pitch pad's neighbours are closer
        # than a full-width stub's end cap allows
        w = int(fanout_geom.stub_width(width, box.GetWidth() / 2, box.GetHeight() / 2))
        for x, y in fanout_geom.candidates(
            cx,
            cy,
            box.GetWidth() / 2,
            box.GetHeight() / 2,
            r,
            gap,
            reach,
            step,
            prefer=(cx - part.x, cy - part.y),
        ):
            if not via_fits(x, y, netcode, pad):
                continue
            stub = pcbnew.SHAPE_SEGMENT(
                pad.GetPosition(), pcbnew.VECTOR2I(int(x), int(y)), w
            )
            if not clear(stub, (layer,), netcode, x, y, False):
                continue
            add_via(x, y, pad.GetNet())
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(pad.GetPosition())
            track.SetEnd(pcbnew.VECTOR2I(int(x), int(y)))
            track.SetWidth(w)
            track.SetLayer(layer)
            track.SetNet(pad.GetNet())
            board.Add(track)
            obstacles.append(track)
            stub_ids.add(id(track))
            added += 1
            placed = True
            break
        if not placed:
            failed.append(name)
    return added, failed


def _via_where(v, spec):
    at = v.GetPosition()
    return (
        f"({pcbnew.ToMM(at.x) - spec['origin'][0]:.2f}, "
        f"{pcbnew.ToMM(at.y) - spec['origin'][1]:.2f})"
    )


def grow_vias(board, spec):
    """Each routed via smaller than its net class's `via_um`, grown to it.

    Freerouting treats every via padstack in the DSN as available to every net
    and drops to the smallest where the class's does not fit where it wants
    it: the integrated carrier's buck input took a 0.6/0.3 mm via against
    input_power's 1.0/0.5 mm, deterministically, on every route. Growing it
    in place is checked by DRC, so a via that does not fit fails the route's
    check rather than shipping undersized. Returns where each one was.
    """
    names = [
        n.GetNetname()
        for n in board.GetNetInfo().NetsByNetcode().values()
        if n.GetNetname()
    ]
    via_of = {}
    for cls in spec["classes"]:
        try:
            nets = dsn.expand(cls["nets"], names, f"class {cls['name']!r}")
        except dsn.DsnError as e:
            # after Freerouting: report it as the DSN rewrite does, not as a traceback
            sys.exit(f"grow_vias: {e}")
        for n in nets:
            via_of[n] = cls["via_um"]
    grown = []
    for v in board.GetTracks():
        if v.GetClass() != "PCB_VIA" or v.GetNetname() not in via_of:
            continue
        want_d, want_drill = (pcbnew.FromMM(u / 1000.0) for u in via_of[v.GetNetname()])
        # GetDrillValue: a routed via's own drill is -1 ("the net class's"),
        # which resolves to the board's Default class
        if v.GetWidth(pcbnew.F_Cu) < want_d - 1 or v.GetDrillValue() < want_drill - 1:
            grown.append(f"{v.GetNetname()} {_via_where(v, spec)}")
            v.SetWidth(want_d)
            v.SetDrill(want_drill)
    return grown


def vias_in_pads(board, spec):
    """Vias whose copper overlaps an SMD pad: inside it the hole drains the
    pad's paste at reflow, and a ring over its edge leaves the pad no mask web
    and takes paste too. DRC does not see either on the pad's own net. Only an
    `[fanout] in_pad` pad (an exposed pad) may hold one; the DSN also tells
    Freerouting `(via_at_smd off)`. A grown via can reach a pad its hole does
    not (fourth pre-fab layout review, F4), so the ring is what is tested."""
    in_pad = set((spec.get("fanout") or {}).get("in_pad", []))
    smd = [
        p
        for fp in board.GetFootprints()
        for p in fp.Pads()
        if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
    ]
    problems = []
    for v in board.GetTracks():
        if v.GetClass() != "PCB_VIA":
            continue
        ring = pcbnew.SHAPE_CIRCLE(v.GetPosition(), v.GetWidth(pcbnew.F_Cu) // 2)
        for p in smd:
            name = f"{p.GetParentFootprint().GetReference()}:{p.GetNumber()}"
            if name in in_pad:
                continue
            if any(
                p.IsOnLayer(layer) and p.GetEffectiveShape(layer).Collide(ring, 0)
                for layer in (pcbnew.F_Cu, pcbnew.B_Cu)
            ):
                problems.append(
                    f"{v.GetNetname()} via at {_via_where(v, spec)} overlaps SMD pad {name}"
                )
                break
    return problems


def in_pad_pins(board, fan, origin, tracks):
    """(net, "REF-PAD") of each [fanout] in_pad pad no [[tracks]] end reaches.

    Its in-pad via joins it to the other layer's pour, so it leaves Freerouting's
    netlist: an exposed pad inside a ring of fine-pitch pins is otherwise a pad
    Freerouting cannot reach, and one unreachable pad can make it abandon the
    whole net (carrier-pcb skill trap 28). A pad a fixed track already ties to
    its net stays in: left out it is a netless obstacle, and the fixed tracks
    into it read as clearance violations (#1280).
    """
    if not fan:
        return []
    tied = {
        (p.GetParentFootprint().GetReference(), p.GetNumber())
        for tr in tracks
        for point in (tr["points"][0], tr["points"][-1])
        for p in end_pads(board, origin, tr, point)
    }
    pins = []
    for name in fan["in_pad"]:
        ref, num = name.split(":", 1)
        fp = board.FindFootprintByReference(ref)
        pad = fp.FindPadByNumber(num) if fp else None
        if pad is None:
            sys.exit(f"[fanout] in_pad {name!r} is not a pad on the board")
        if (ref, num) not in tied:
            pins.append((pad.GetNetname(), f"{ref}-{num}"))
    return pins


def stitch(board, spec, zones):
    """A via of each pour net at each [pour] stitch grid point where that net's
    fill holds the whole via, plus the pour clearance, on every layer the net
    pours on, and no other via or hole is within that reach. Returns how many.

    The fill already stands the pour's clearance off every other net's copper,
    so a via inside it keeps that clearance too; DRC checks the result.
    """
    st = spec["pour"].get("stitch")
    if not st:
        return 0
    x0, y0 = spec["origin"]
    width, height = spec["size"]
    d_um, drill_um = st["via_um"]
    radius = d_um / 2000.0 + spec["pour"]["clearance_mm"]
    fills = {}
    for z in zones:
        for layer in z.GetLayerSet().Seq():
            fills.setdefault(z.GetNetname(), {}).setdefault(layer, []).append(
                z.GetFilledPolysList(layer)
            )
    holes = [
        (v.GetPosition(), v.GetWidth(pcbnew.F_Cu) / 2)
        for v in board.GetTracks()
        if v.GetClass() == "PCB_VIA"
    ]
    holes += [
        (p.GetPosition(), max(p.GetSizeX(), p.GetSizeY()) / 2)
        for fp in board.GetFootprints()
        for p in fp.Pads()
        if p.GetDrillSizeX() > 0
    ]
    added = 0
    for x, y in stitch_geom.grid(width, height, st["pitch_mm"], st.get("exclude", [])):
        at = pcbnew.VECTOR2I(pcbnew.FromMM(x0 + x), pcbnew.FromMM(y0 + y))
        reach = pcbnew.FromMM(radius)
        if any(
            (at.x - c.x) ** 2 + (at.y - c.y) ** 2 < (reach + r) ** 2 for c, r in holes
        ):
            continue
        ring = [
            pcbnew.VECTOR2I(pcbnew.FromMM(x0 + px), pcbnew.FromMM(y0 + py))
            for px, py in stitch_geom.ring(x, y, radius)
        ]
        for net, layers in sorted(fills.items()):
            if all(
                all(any(f.Contains(pt) for f in polys) for pt in ring)
                for polys in layers.values()
            ):
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(at)
                via.SetWidth(pcbnew.FromMM(d_um / 1000.0))
                via.SetDrill(pcbnew.FromMM(drill_um / 1000.0))
                via.SetNet(board.FindNet(net))
                board.Add(via)
                holes.append((at, pcbnew.FromMM(d_um / 2000.0)))
                added += 1
                break
    return added


def island_vias(board, spec, zones):
    """A via in each piece of a pour net's fill cut off from that zone's main
    piece on its layer, where one fits with the pour clearance and the other
    pour layer's fill of the net holds it too. Returns how many.

    Routed tracks can fence a pocket of fill round a ground pad; the pad's
    thermal spokes then reach only that pocket, which DRC reports as a starved
    thermal into an isolated island (#1280: A5:1, J1:15, J7:4 in most routes).
    A via ties the pocket to the other layer's plane. Uses [pour] stitch's via.
    """
    st = spec["pour"].get("stitch")
    if not st:
        return 0
    d_um, drill_um = st["via_um"]
    # Inside the fill is enough: its edge already stands the pour clearance off
    # every other net. Holes keep the board's 0.25 mm hole to hole.
    reach = pcbnew.FromMM(d_um / 2000.0 + 0.02)
    drill = pcbnew.FromMM(drill_um / 2000.0)
    gap = pcbnew.FromMM(0.25)
    step = pcbnew.FromMM(0.05)
    fills = {}
    for z in zones:
        for layer in z.GetLayerSet().Seq():
            fills.setdefault(z.GetNetname(), {}).setdefault(layer, []).append(
                z.GetFilledPolysList(layer)
            )
    holes = [
        (v.GetPosition(), v.GetDrill() / 2)
        for v in board.GetTracks()
        if v.GetClass() == "PCB_VIA"
    ] + [
        (p.GetPosition(), max(p.GetDrillSizeX(), p.GetDrillSizeY()) / 2)
        for fp in board.GetFootprints()
        for p in fp.Pads()
        if p.GetDrillSizeX() > 0
    ]

    def ring(at):
        return [
            pcbnew.VECTOR2I(
                at.x + int(reach * math.cos(a)), at.y + int(reach * math.sin(a))
            )
            for a in (k * math.pi / 8 for k in range(16))
        ] + [at]

    added = 0
    for z in zones:
        net = z.GetNetname()
        for layer in z.GetLayerSet().Seq():
            polys = z.GetFilledPolysList(layer)
            n = polys.OutlineCount()
            if n < 2:
                continue
            areas = [abs(polys.Outline(i).Area()) for i in range(n)]
            main = areas.index(max(areas))
            others = [fl for other, fl in fills[net].items() if other != layer]
            for i in range(n):
                if i == main:
                    continue
                chain = polys.Outline(i)
                box = chain.BBox()
                found = None
                y = box.GetTop() + reach
                while found is None and y <= box.GetBottom() - reach:
                    x = box.GetLeft() + reach
                    while x <= box.GetRight() - reach:
                        at = pcbnew.VECTOR2I(int(x), int(y))
                        pts = ring(at)
                        if (
                            all(
                                chain.PointInside(pt) and polys.Contains(pt)
                                for pt in pts
                            )
                            and all(
                                all(any(f.Contains(pt) for f in fl) for pt in pts)
                                for fl in others
                            )
                            and not any(
                                (at.x - c.x) ** 2 + (at.y - c.y) ** 2
                                < (drill + gap + r) ** 2
                                for c, r in holes
                            )
                        ):
                            found = at
                            break
                        x += step
                    y += step
                if found is None:
                    continue
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(found)
                via.SetWidth(pcbnew.FromMM(d_um / 1000.0))
                via.SetDrill(pcbnew.FromMM(drill_um / 1000.0))
                via.SetNet(board.FindNet(net))
                board.Add(via)
                holes.append((found, drill))
                added += 1
    return added


def route(spec):
    pcb = spec["pcb"]
    out = spec["out_dir"]
    pour = spec["pour"]
    os.makedirs(out, exist_ok=True)
    dsn_path = os.path.join(out, "carrier.dsn")
    ses_path = os.path.join(out, "carrier.ses")
    board = pcbnew.LoadBoard(pcb)
    # build_board.py lays down board.toml [[tracks]] locked; anything else is a
    # previous route.
    if any(not t.IsLocked() for t in board.GetTracks()):
        sys.exit("board already has tracks; run build_board.py first")
    regions = pour["regions"]
    for region in regions:
        if board.FindNet(region["net"]) is None:
            sys.exit(f"[pour] net {region['net']!r} is not on the board")
    layers = pour_layers(board, pour)

    if not pcbnew.ExportSpecctraDSN(board, dsn_path):
        sys.exit("DSN export failed")
    with open(dsn_path) as f:
        try:
            text = dsn.rewrite(f.read(), spec["classes"], spec["signal"])
            check_track_ends(board, spec["origin"], spec.get("tracks", []))
            text = dsn.rect_keepouts(
                text, spec.get("route_keepouts", []), spec["origin"]
            )
            text = dsn.no_via_at_smd(text)
            pins = in_pad_pins(
                board, spec.get("fanout"), spec["origin"], spec.get("tracks", [])
            )
            text = dsn.drop_pins(text, sorted(set(pins)))
            text, snapped = dsn.snap_wire_ends(text)
            dsn.check_balanced(text)
        except dsn.DsnError as e:
            sys.exit(f"DSN rewrite: {e}")
    with open(dsn_path, "w") as f:
        f.write(text)
    print(f"snapped {snapped} fixed wire end(s) onto their pins' centres")

    if os.path.exists(ses_path):
        os.remove(ses_path)
    log_path = os.path.join(out, "freerouting.log")
    done = subprocess.run(
        [
            "java",
            # no window, no Dock icon: on macOS a JVM otherwise takes the
            # focus each run, though Freerouting's GUI is off
            "-Djava.awt.headless=true",
            "-Dapple.awt.UIElement=true",
            "-jar",
            spec["freerouting_jar"],
            "-de",
            dsn_path,
            "-do",
            ses_path,
            "-mp",
            "100",
            "--gui.enabled=false",
            # Not in --help; a LoggingSettings key, like gui.enabled. The
            # DEBUG lines name what the router could not reach.
            "--logging.console.level=DEBUG",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    with open(log_path, "w") as f:
        f.write(done.stdout)
    # The console keeps Freerouting's usual INFO-and-above output.
    usual = [line for line in done.stdout.splitlines(True) if " DEBUG " not in line]
    print("".join(usual), end="")
    stuck = dsn.stuck_points(done.stdout, spec["origin"])
    if stuck:
        print(
            "Freerouting could not reach (board mm, from its DEBUG log, "
            f"{log_path}): " + ", ".join(f"({x:.2f}, {y:.2f})" for x, y in stuck)
        )
    with open(dsn_path) as f:
        failed = dsn.failed_connections(done.stdout, f.read())

    def shown(pads):
        head = ", ".join(pads[:8])
        return f"{head} (+{len(pads) - 8} more)" if len(pads) > 8 else head or "tracks"

    for reason, a, b in failed:
        print(f"Freerouting: {reason}: {shown(a)} || {shown(b)}")
    if done.returncode != 0:
        sys.exit(f"Freerouting exited {done.returncode}; see {log_path}")
    if not os.path.exists(ses_path):
        sys.exit("Freerouting wrote no session file")
    if not pcbnew.ImportSpecctraSES(board, ses_path):
        sys.exit("SES import failed")

    grown = grow_vias(board, spec)
    if grown:
        print(
            f"grew {len(grown)} via(s) to their class's size (DRC checks the fit): "
            f"{', '.join(grown)}"
        )

    # Freerouting necks a track to half its width to reach a fine-pitch pad,
    # often below the board's minimum (0.125 mm against 0.15 mm, over a few
    # hundredths of a millimetre). Widen those to the minimum; DRC still
    # checks the clearance that leaves.
    min_width = board.GetDesignSettings().m_TrackMinWidth
    widened = 0
    for t in board.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.GetWidth() < min_width:
            t.SetWidth(min_width)
            widened += 1
    if widened:
        print(f"widened {widened} necked track(s) to {pcbnew.ToMM(min_width)} mm")

    # Vias beside the poured nets' SMD pads, so the pour on the other layer
    # reaches them: Freerouting adds a via only where a track changes layer.
    # Each `required` pad must get one; the rest keep their routed track if
    # there is no room.
    fan = spec.get("fanout")
    clearance = max(c["clearance_um"] for c in [*spec["classes"], spec["signal"]])
    added, failed = fanout(board, fan, pcbnew.FromMM(clearance / 1000.0))
    if added:
        print(f"fanout: added {added} vias")
    missing = [p for p in failed if p in (fan or {}).get("required", [])]
    if missing:
        sys.exit(
            f"[fanout] found no place for a via beside required pad(s): "
            f"{', '.join(missing)} (move a part, or reroute)"
        )
    if failed:
        print(
            f"fanout: no room for a via beside {', '.join(failed)}; routed track only"
        )
    # fanout's vias take [fanout] via_um; a net whose class asks for more
    # gets them grown too, and DRC checks the fit
    grown = grow_vias(board, spec)
    if grown:
        print(
            f"grew {len(grown)} fanout via(s) to their class's size: {', '.join(grown)}"
        )
    # after fanout, so it covers Freerouting's vias and fanout's alike
    bad = vias_in_pads(board, spec)
    if bad:
        sys.exit("vias inside SMD pads:\n  " + "\n  ".join(bad))

    # The project's .kicad_dru lets a pour thermal resolve to one spoke on the
    # premise that every pad of a pour net also has a routed track. Enforce
    # the premise.
    for net in sorted({r["net"] for r in regions}):
        tracks = [t for t in board.GetTracks() if t.GetNetname() == net]
        bare = [
            f"{fp.GetReference()}:{pad.GetNumber()}"
            for fp in board.GetFootprints()
            for pad in fp.Pads()
            if pad.GetNetname() == net
            and not any(
                pad.HitTest(t.GetStart()) or pad.HitTest(t.GetEnd()) for t in tracks
            )
        ]
        if bare:
            sys.exit(
                f"{net} pads with no routed track (the .kicad_dru rule needs one): {bare}"
            )

    # A region pours on its own `layers` if it names them, else on every
    # [pour] layer, at its own `priority` if it sets one, else at its index.
    zones = [
        pour_zone(
            board,
            layer,
            pour,
            r["net"],
            region_box(board, spec["origin"], r["rect"]),
            r.get("priority", n),
        )
        for n, r in enumerate(regions)
        for layer in (
            pour_layers(board, {"layers": r["layers"]}) if "layers" in r else layers
        )
    ]
    for name in pour.get("solid_pads", []):
        ref, num = name.split(":", 1)
        fp = board.FindFootprintByReference(ref)
        pad = fp.FindPadByNumber(num) if fp else None
        if pad is None:
            sys.exit(f"[pour] solid_pads {name!r} is not a pad on the board")
        for p in fp.Pads():  # a stacked pair shares the number's spot
            if p.GetNumber() == num:
                p.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    pcbnew.ZONE_FILLER(board).Fill(zones)
    added = stitch(board, spec, zones)
    if added:
        bad = vias_in_pads(board, spec)
        if bad:
            sys.exit("stitching vias inside SMD pads:\n  " + "\n  ".join(bad))
        pcbnew.ZONE_FILLER(board).Fill(zones)
        print(f"stitch: added {added} vias")
    islands = island_vias(board, spec, zones)
    if islands:
        bad = vias_in_pads(board, spec)
        if bad:
            sys.exit("island vias inside SMD pads:\n  " + "\n  ".join(bad))
        pcbnew.ZONE_FILLER(board).Fill(zones)
        print(f"islands: added {islands} vias")
    board.Save(pcb)
    return len(board.GetTracks())


def main(argv):
    if len(argv) != 2:
        sys.exit(__doc__)
    with open(argv[1]) as f:
        spec = json.load(f)
    n = route(spec)
    print(f"routed {n} track/via items, wrote {spec['pcb']}")


if __name__ == "__main__":
    main(sys.argv)
