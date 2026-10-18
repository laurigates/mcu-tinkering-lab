"""Autoroute a placed carrier board with Freerouting, then pour copper.

Run with KiCad's bundled Python (3.9) after build_board.py:

    $KICAD_PYTHON tools/kicad/pcb/route_board.py <board-input.json>

``just <project>::carrier-board`` writes the JSON (``python3 -m
kicad.boardinput``) and runs both scripts. Steps: export Specctra DSN, rewrite
its single net class into board.toml's width classes (dsn.py), run Freerouting
headless (each board.toml ``[[tracks]]`` segment a keepout in the DSN, since
Freerouting drops the exported wires, and the pad under each track's first
point out of its netlist, since the track already joins it to the pad under
its last point), import the session, check every pad of
each pour net has a routed track, then add a zone per board.toml ``[pour]``
region on each of its layers
(the region's own ``layers``, else every ``[pour]`` layer) and fill them. A
region's ``rect`` (relative to the outline origin) is its zone outline; a null
``rect`` is the whole board. Keepouts are already rule areas
on the board (build_board.py), so the export and the fill respect them.

Freerouting does not route the same way twice: commit the board that passed
``just <project>::carrier-check``, not a re-run of it. Needs java >= 25 and the
Freerouting 2.5.0 jar (``freerouting_jar`` in the JSON, from $FREEROUTING_JAR).
"""

import json
import os
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dsn


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


def end_pad(board, origin, tr, point):
    """The pad of `tr`'s net and layer under `point`, else exit."""
    at = pcbnew.VECTOR2I(
        pcbnew.FromMM(origin[0] + point[0]), pcbnew.FromMM(origin[1] + point[1])
    )
    layer = board.GetLayerID(tr["layer"])
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if (
                pad.GetNetname() == tr["net"]
                and pad.IsOnLayer(layer)
                and pad.HitTest(at)
            ):
                return pad
    sys.exit(
        f"[[tracks]] {tr['name']!r}: {point} is on no {tr['net']} pad on {tr['layer']}"
    )


def track_pins(board, origin, tracks):
    """(net, "REF-PAD") of the pad under each track's first point.

    Both ends must be on a pad of the track's net. The first pad leaves
    Freerouting's netlist (dsn.drop_pins): the track joins it to the last.
    """
    pins = []
    for tr in tracks:
        first = end_pad(board, origin, tr, tr["points"][0])
        last = end_pad(board, origin, tr, tr["points"][-1])
        if first.GetParentFootprint() is last.GetParentFootprint() and (
            first.GetNumber() == last.GetNumber()
        ):
            sys.exit(f"[[tracks]] {tr['name']!r} starts and ends on one pad")
        ref = first.GetParentFootprint().GetReference()
        pins.append((tr["net"], f"{ref}-{first.GetNumber()}"))
    return pins


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
            # Freerouting ignores the exported [[tracks]] wires; keep it off them.
            margin = max(c["clearance_um"] for c in [*spec["classes"], spec["signal"]])
            tracks = spec.get("tracks", [])
            text = dsn.track_keepouts(text, tracks, spec["origin"], margin)
            text = dsn.drop_pins(text, track_pins(board, spec["origin"], tracks))
        except dsn.DsnError as e:
            sys.exit(f"DSN rewrite: {e}")
    with open(dsn_path, "w") as f:
        f.write(text)

    if os.path.exists(ses_path):
        os.remove(ses_path)
    subprocess.run(
        [
            "java",
            "-jar",
            spec["freerouting_jar"],
            "-de",
            dsn_path,
            "-do",
            ses_path,
            "-mp",
            "100",
            "--gui.enabled=false",
        ],
        check=True,
    )
    if not os.path.exists(ses_path):
        sys.exit("Freerouting wrote no session file")
    if not pcbnew.ImportSpecctraSES(board, ses_path):
        sys.exit("SES import failed")

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
    pcbnew.ZONE_FILLER(board).Fill(zones)
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
