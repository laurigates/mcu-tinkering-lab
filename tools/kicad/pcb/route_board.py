"""Autoroute a placed carrier board with Freerouting, then pour copper.

Run with KiCad's bundled Python (3.9) after build_board.py:

    $KICAD_PYTHON tools/kicad/pcb/route_board.py <board-input.json>

``just <project>::carrier-board`` writes the JSON (``python3 -m
kicad.boardinput``) and runs both scripts. Steps: export Specctra DSN, rewrite
its single net class into board.toml's width classes (dsn.py), run Freerouting
headless, import the session, check every pad of the pour net has a routed
track, then add the board.toml ``[pour]`` zone on each of its layers and fill
them.

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
import dsn  # noqa: E402


def pour_zone(board, layer, pour):
    net = board.FindNet(pour["net"])
    zone = pcbnew.ZONE(board)
    zone.SetLayer(layer)
    zone.SetNetCode(net.GetNetCode())
    zone.SetLocalClearance(pcbnew.FromMM(pour["clearance_mm"]))
    zone.SetMinThickness(pcbnew.FromMM(pour["min_thickness_mm"]))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    # Narrower than KiCad's 0.5 mm defaults so two spokes still fit between
    # 2.54 mm-pitch header pins and their signal tracks (balancebot's J3 pad 7
    # was reported as a starved thermal at the defaults).
    zone.SetThermalReliefGap(pcbnew.FromMM(pour["thermal_gap_mm"]))
    zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(pour["spoke_mm"]))
    box = board.GetBoardEdgesBoundingBox()
    outline = zone.Outline()
    outline.NewOutline()
    for x, y in (
        (box.GetLeft(), box.GetTop()),
        (box.GetRight(), box.GetTop()),
        (box.GetRight(), box.GetBottom()),
        (box.GetLeft(), box.GetBottom()),
    ):
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


def route(spec):
    pcb = spec["pcb"]
    out = spec["out_dir"]
    pour = spec["pour"]
    os.makedirs(out, exist_ok=True)
    dsn_path = os.path.join(out, "carrier.dsn")
    ses_path = os.path.join(out, "carrier.ses")
    board = pcbnew.LoadBoard(pcb)
    if len(board.GetTracks()):
        sys.exit("board already has tracks; run build_board.py first")
    if board.FindNet(pour["net"]) is None:
        sys.exit(f"[pour] net {pour['net']!r} is not on the board")
    layers = pour_layers(board, pour)
    if not pcbnew.ExportSpecctraDSN(board, dsn_path):
        sys.exit("DSN export failed")
    with open(dsn_path) as f:
        try:
            text = dsn.rewrite(f.read(), spec["classes"], spec["signal"])
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
    # premise that every pad of the pour net also has a routed track. Enforce
    # the premise.
    tracks = [t for t in board.GetTracks() if t.GetNetname() == pour["net"]]
    bare = [
        f"{fp.GetReference()}:{pad.GetNumber()}"
        for fp in board.GetFootprints()
        for pad in fp.Pads()
        if pad.GetNetname() == pour["net"]
        and not any(
            pad.HitTest(t.GetStart()) or pad.HitTest(t.GetEnd()) for t in tracks
        )
    ]
    if bare:
        sys.exit(
            f"{pour['net']} pads with no routed track (the .kicad_dru rule needs one): {bare}"
        )

    zones = [pour_zone(board, layer, pour) for layer in layers]
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
