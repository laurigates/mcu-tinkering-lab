"""Gamepad Synth — wiring schematic.

ESP32-S3 dev board driving a MAX98357A I2S amplifier and a status LED.
Source of truth: packages/audio/gamepad-synth/WIRING.md
"""

import schemdraw
import schemdraw.elements as elm

from components import esp32_s3_zero, max98357a


def draw() -> schemdraw.Drawing:
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=12)

    esp = d.add(esp32_s3_zero().label("ESP32-S3-Zero", loc="bot", ofst=0.4))
    amp = d.add(
        max98357a()
        .at((esp.center.x + 9, esp.center.y))
        .anchor("center")
        .label("MAX98357A", loc="top", ofst=0.4)
    )

    # I2S signal bus — ESP right side ↔ amp left side, straight horizontals.
    d.add(elm.Wire("-").at(esp.GPIO5).to(amp.BCLK).color("steelblue"))
    d.add(elm.Wire("-").at(esp.GPIO6).to(amp.LRC).color("steelblue"))
    d.add(elm.Wire("-").at(esp.GPIO7).to(amp.DIN).color("steelblue"))

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

    # Speaker: small symbol is centered below amp; OUT-/OUT+ drop vertically
    # and turn inward to meet its in1/in2 terminals.
    spk_y = amp["OUT-"].y - 2.5
    spk = d.add(
        elm.Speaker()
        .right()
        .at((amp.center.x - 0.25, spk_y))
        .label("4-8 Ω  2-3 W", loc="bot", ofst=0.4)
    )
    d.add(elm.Wire("|-").at(amp["OUT-"]).to(spk.in1))
    d.add(elm.Wire("|-").at(amp["OUT+"]).to(spk.in2))

    # Status LED branch from GPIO2 (top-left of ESP): out, up, through R + LED
    # to ground — routed clear of the power rails below.
    d.add(elm.Line().left(0.5).at(esp.GPIO2))
    d.add(elm.Resistor().up().label("220 Ω"))
    d.add(elm.LED().up().label("Status", loc="left"))
    d.add(elm.Line().left(0.5))
    d.add(elm.Ground().label("GND", loc="left"))

    return d


if __name__ == "__main__":
    import sys
    from pathlib import Path

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("gamepad_synth.svg")
    draw().save(str(out))
    print(f"wrote {out}")
