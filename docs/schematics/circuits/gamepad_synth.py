"""Gamepad Synth — wiring schematic.

ESP32-S3 dev board driving a MAX98357A I2S amplifier and a status LED.
Source of truth: packages/audio/gamepad-synth/WIRING.md
"""

import schemdraw
import schemdraw.elements as elm

from components import esp32_s3_zero, max98357a
from routing import Router


def draw() -> schemdraw.Drawing:
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=12)

    # === Components first, so every net below routes with full obstacle
    # awareness (the auto-router only avoids components already placed). ===
    esp = d.add(esp32_s3_zero().label("ESP32-S3-Zero", loc="bot", ofst=0.4))
    amp = d.add(
        max98357a()
        .at((esp.center.x + 9, esp.center.y))
        .anchor("center")
        .label("MAX98357A", loc="top", ofst=0.4)
    )

    # Speaker: small symbol is centered below amp; OUT-/OUT+ drop vertically
    # and turn inward to meet its in1/in2 terminals.
    spk_y = amp["OUT-"].y - 2.5
    spk = d.add(
        elm.Speaker()
        .right()
        .at((amp.center.x - 0.25, spk_y))
        .label("4-8 Ω  2-3 W", loc="bot", ofst=0.4)
    )

    # Optional Drone-mode piezo pair: GPIO8/GPIO9 on the bottom drive two
    # piezo discs (second leg to GND). Speaker symbols stand in for piezos.
    piezo_y = esp.GPIO8.y - 2.5
    pz_a = d.add(
        elm.Speaker()
        .right()
        .at((esp.GPIO8.x - 0.25, piezo_y))
        .label("Piezo A", loc="bot", ofst=0.3)
    )
    pz_b = d.add(
        elm.Speaker()
        .right()
        .at((esp.GPIO9.x - 0.25, piezo_y))
        .label("Piezo B", loc="bot", ofst=0.3)
    )

    # === Nets: auto-routed orthogonal, obstacle-avoiding wires. ===
    router = Router(d)

    # I2S signal bus — ESP right side ↔ amp left side.
    router.wire(esp.GPIO5, amp.BCLK, color="steelblue")
    router.wire(esp.GPIO6, amp.LRC, color="steelblue")
    router.wire(esp.GPIO7, amp.DIN, color="steelblue")

    router.wire(amp["OUT-"], spk.in1)
    router.wire(amp["OUT+"], spk.in2)

    router.wire(esp.GPIO8, pz_a.in1, color="darkorange")
    router.wire(esp.GPIO9, pz_b.in1, color="darkorange")

    # === Local stubs (power tags, LED branch, piezo grounds) stay
    # hand-drawn — these aren't point-to-point nets between two components,
    # so the router adds nothing here. ===

    # ESP power: +3V3 and Ground tags on the outward-facing left side.
    d.add(elm.Line().left(0.5).at(esp["3V3"]))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(0.5).at(esp.GND))
    d.add(elm.Ground())

    # Amp power: +3V3 and Ground tags on the outward-facing right side.
    d.add(elm.Line().right(0.5).at(amp.VIN))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().right(0.5).at(amp.GND))
    d.add(elm.Ground())

    # Status LED branch from GPIO2 (top-left of ESP): out, up, through R + LED
    # to ground — routed clear of the power rails below.
    d.add(elm.Line().left(0.5).at(esp.GPIO2))
    d.add(elm.Resistor().up().label("220 Ω"))
    d.add(elm.LED().up().label("Status", loc="left"))
    d.add(elm.Line().left(0.5))
    d.add(elm.Ground().label("GND", loc="left"))

    # Piezo grounds.
    d.add(elm.Line().down(0.5).at(pz_a.in2))
    d.add(elm.Ground())
    d.add(elm.Line().down(0.5).at(pz_b.in2))
    d.add(elm.Ground())

    return d


if __name__ == "__main__":
    import sys
    from pathlib import Path

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("gamepad_synth.svg")
    draw().save(str(out))
    print(f"wrote {out}")
