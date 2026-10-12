"""Autoroute balancebot-carrier.kicad_pcb with Freerouting, then pour GND.

Run with KiCad's bundled Python after build_board.py:

    just balancebot::carrier-board

Steps: export Specctra DSN, rewrite its single net class into three width
classes, run Freerouting headless, import the session, add a GND zone on both
copper layers and fill them.
"""

import os
import re
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PRJ = os.path.join(os.path.dirname(HERE), "balancebot-carrier")
PCB = os.path.join(PRJ, "balancebot-carrier.kicad_pcb")
OUT = os.path.join(PRJ, "output")
FREEROUTING_JAR = os.environ.get(
    "FREEROUTING_JAR", os.path.expanduser("~/.cache/freerouting/freerouting-2.5.0.jar")
)

MOTOR_NETS = [f"MOT{side}_{coil}" for side in "LR" for coil in ("A1", "A2", "B1", "B2")]
# (name, nets, track width um, clearance um, via padstack)
CLASSES = [
    ("power", ["VBAT", "GND", *MOTOR_NETS], 1000, 300, "Via[0-1]_1000:500_um"),
    ("logic_power", ["+3V3", "BUCK_5V", "XIAO_5V"], 500, 250, "Via[0-1]_800:400_um"),
]
SIGNAL = ("signal", 300, 250, "Via[0-1]_600:300_um")
EXTRA_VIAS = {"Via[0-1]_1000:500_um": 1000, "Via[0-1]_800:400_um": 800}


def class_block(name, nets, width, clearance, via):
    return (
        f"    (class {name} {' '.join(nets)}\n"
        f'      (circuit\n        (use_via "{via}")\n      )\n'
        f"      (rule\n        (width {width})\n        (clearance {clearance})\n      )\n    )\n"
    )


def rewrite_dsn(text):
    m = re.search(r"    \(class kicad_default (.*?)\n      \(circuit", text, re.S)
    if not m:
        sys.exit("DSN has no kicad_default class to rewrite")
    all_nets = m.group(1).split()
    claimed = {n for _, nets, *_ in CLASSES for n in nets}
    unknown = claimed - set(all_nets)
    if unknown:
        sys.exit(f"class nets not in the board: {sorted(unknown)}")
    signal_nets = [n for n in all_nets if n not in claimed]
    start = m.start()
    end = text.index("    )\n", text.index("(rule", m.end())) + len("    )\n")
    blocks = [class_block(*c) for c in CLASSES]
    blocks.append(class_block(SIGNAL[0], signal_nets, *SIGNAL[1:]))
    text = text[:start] + "".join(blocks) + text[end:]

    lib_end = text.index("  )\n  (network")
    pads = "".join(
        f'    (padstack "{name}"\n      (shape (circle F.Cu {d}))\n'
        f"      (shape (circle B.Cu {d}))\n      (attach off)\n    )\n"
        for name, d in EXTRA_VIAS.items()
    )
    text = text[:lib_end] + pads + text[lib_end:]
    vias = " ".join(f'"{v}"' for v in [SIGNAL[3], *EXTRA_VIAS])
    return text.replace('(via "Via[0-1]_600:300_um")', f"(via {vias})", 1)


def gnd_zone(board, layer):
    net = board.FindNet("GND")
    zone = pcbnew.ZONE(board)
    zone.SetLayer(layer)
    zone.SetNetCode(net.GetNetCode())
    zone.SetLocalClearance(pcbnew.FromMM(0.4))
    zone.SetMinThickness(pcbnew.FromMM(0.25))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    # Narrower than KiCad's 0.5 mm defaults so two spokes still fit between the
    # 2.54 mm-pitch header pins and their 0.3 mm signal tracks (J3 pad 7 was
    # reported as a starved thermal at the defaults).
    zone.SetThermalReliefGap(pcbnew.FromMM(0.3))
    zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.35))
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


def main():
    os.makedirs(OUT, exist_ok=True)
    dsn = os.path.join(OUT, "carrier.dsn")
    ses = os.path.join(OUT, "carrier.ses")
    board = pcbnew.LoadBoard(PCB)
    if len(board.GetTracks()):
        sys.exit("board already has tracks; run build_board.py first")
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        sys.exit("DSN export failed")
    with open(dsn) as f:
        text = rewrite_dsn(f.read())
    with open(dsn, "w") as f:
        f.write(text)

    if os.path.exists(ses):
        os.remove(ses)
    subprocess.run(
        [
            "java",
            "-jar",
            FREEROUTING_JAR,
            "-de",
            dsn,
            "-do",
            ses,
            "-mp",
            "100",
            "--gui.enabled=false",
        ],
        check=True,
    )
    if not os.path.exists(ses):
        sys.exit("Freerouting wrote no session file")
    if not pcbnew.ImportSpecctraSES(board, ses):
        sys.exit("SES import failed")

    # balancebot-carrier.kicad_dru lets a GND thermal resolve to one spoke on the
    # premise that every GND pad also has a routed track. Enforce the premise.
    gnd_tracks = [t for t in board.GetTracks() if t.GetNetname() == "GND"]
    bare = [
        f"{fp.GetReference()}:{pad.GetNumber()}"
        for fp in board.GetFootprints()
        for pad in fp.Pads()
        if pad.GetNetname() == "GND"
        and not any(
            pad.HitTest(t.GetStart()) or pad.HitTest(t.GetEnd()) for t in gnd_tracks
        )
    ]
    if bare:
        sys.exit(
            f"GND pads with no routed track (the .kicad_dru rule needs one): {bare}"
        )

    zones = [gnd_zone(board, pcbnew.F_Cu), gnd_zone(board, pcbnew.B_Cu)]
    pcbnew.ZONE_FILLER(board).Fill(zones)
    board.Save(PCB)
    print(f"routed {len(board.GetTracks())} track/via items, wrote {PCB}")


if __name__ == "__main__":
    main()
