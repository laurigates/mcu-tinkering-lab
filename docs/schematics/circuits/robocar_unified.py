"""Robocar Unified — single-board wiring schematic.

XIAO ESP32-S3 Sense driving everything via an I2C multiplexer:
  - TCA9548A ch0 → PCA9685 → 2x RGB LEDs, 2x SG90 servos, TB6612FNG → motors
  - TCA9548A ch1 → SSD1306 OLED
Direct GPIO: STBY (motor enable), piezo, ultrasonic TRIG/ECHO,
and I2S (D8-D10) → MAX98357A → speaker for the robot's voice (ADR-019).

Source of truth: packages/robocar/unified/WIRING.md and main/pin_config.h
"""

import schemdraw
import schemdraw.elements as elm

from components import (
    hc_sr04p,
    max98357a,
    pca9685,
    ssd1306_oled,
    tb6612fng,
    tca9548a,
    xiao_esp32s3_sense,
)
from routing import Router


def draw() -> schemdraw.Drawing:
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=11)

    # === Components first, so every net below routes with full obstacle
    # awareness (the auto-router only avoids components already placed). ===

    # Top row: MCU → I2C mux → PCA9685 (chained left-to-right). TB6612FNG
    # sits below-and-right of PCA so PCA's right side stays clear for the
    # LED/servo stub tags.
    xiao = d.add(xiao_esp32s3_sense().label("XIAO ESP32-S3 Sense", loc="bot", ofst=0.4))

    mux = d.add(
        tca9548a()
        .at((xiao.center.x + 9, xiao.center.y))
        .anchor("center")
        .label("TCA9548A\n0x70", loc="bot", ofst=0.4)
    )

    pca = d.add(
        pca9685()
        .at((mux.center.x + 9, mux.center.y))
        .anchor("center")
        .label("PCA9685\n0x40 @ 200Hz", loc="bot", ofst=0.4)
    )

    # TB6612FNG: shifted down 8 units so PCA's right side is unobstructed.
    tb = d.add(
        tb6612fng()
        .at((pca.center.x + 7, pca.center.y - 8))
        .anchor("center")
        .label("TB6612FNG", loc="bot", ofst=0.4)
    )

    # Motors on the far right, driven by TB outputs.
    motor_l = d.add(
        elm.Motor()
        .right()
        .at((tb.center.x + 5, tb.BO1.y - 0.5))
        .label("Left motor", loc="bot", ofst=0.4)
    )
    motor_r = d.add(
        elm.Motor()
        .right()
        .at((tb.center.x + 5, tb.AO1.y - 0.5))
        .label("Right motor", loc="bot", ofst=0.4)
    )

    # Mux ch1 → SSD1306 OLED. Explicit .right() locks orientation — without
    # it, the OLED inherits the previous element's "up" direction and gets
    # rotated 90°.
    oled = d.add(
        ssd1306_oled()
        .right()
        .at((mux.center.x + 4, mux.center.y - 8))
        .anchor("center")
        .label("SSD1306 OLED\n0x3C, 128x64", loc="bot", ofst=0.4)
    )

    # Ultrasonic below the MCU.
    us = d.add(
        hc_sr04p()
        .right()
        .at((xiao.center.x + 4, xiao.GPIO3.y - 5))
        .anchor("center")
        .label("HC-SR04P\nultrasonic", loc="bot", ofst=0.4)
    )

    # MAX98357A above the MCU: its I2S pins are on the left, and the XIAO's
    # I2S trio is at the top of its right side, so the bus runs up-and-over
    # without crossing the I2C wires heading right to the mux.
    amp = d.add(
        max98357a()
        .right()
        .at((xiao.center.x + 5, xiao.center.y + 11))
        .anchor("center")
        .label("MAX98357A", loc="top", ofst=0.4)
    )

    # Speaker below the amp, clear of the XIAO body. 8 Ω is the safer starting
    # point — it roughly halves peak current versus 4 Ω on a rail that already
    # has brownout detection disabled for motor inrush.
    spk = d.add(
        elm.Speaker()
        .right()
        .at((amp.center.x - 0.25, amp["OUT-"].y - 2.5))
        .label("8 Ω  2-3 W", loc="bot", ofst=0.4)
    )

    # Piezo buzzer on GPIO2 — small branch through resistor to ground. Placed
    # before routing (GPIO2 isn't a routed net in this circuit): the
    # resistor/speaker are real components, not cosmetic tags, and the STBY
    # run below happens to pass nearby.
    d.add(elm.Line().right(0.5).at(xiao.GPIO2))
    d.add(elm.Resistor().right().label("100 Ω"))
    buz = d.add(elm.Speaker().right().label("Piezo", loc="top", ofst=0.3))
    d.add(elm.Line().down(0.5).at(buz.in2))
    d.add(elm.Ground())

    # === Nets: auto-routed orthogonal, obstacle-avoiding wires. ===
    router = Router(d)

    # I2C bus: XIAO right side ↔ mux left side (top two pins).
    router.wire(xiao.GPIO5, mux.SDA, color="steelblue")
    router.wire(xiao.GPIO6, mux.SCL, color="steelblue")

    # Mux ch0 (SD0/SC0) → PCA9685 SDA/SCL.
    router.wire(mux.SD0, pca.SDA, color="steelblue")
    router.wire(mux.SC0, pca.SCL, color="steelblue")

    # PCA9685 PWM 8-13 group → TB6612FNG control cluster.
    # 6 logical signals (PWMA, AIN1/2, PWMB, BIN1/2) drawn as one trunk.
    router.wire(pca["PWM 8-13"], tb.PWMA, color="steelblue")

    router.wire(tb.BO1, motor_l.start)
    router.wire(tb.BO2, motor_l.end)
    router.wire(tb.AO1, motor_r.start)
    router.wire(tb.AO2, motor_r.end)

    # STBY direct from MCU GPIO1 — the router finds its own way around the
    # mux/PCA/motor obstacles now that every component is already placed.
    router.wire(xiao.GPIO1, tb.STBY, color="steelblue")

    router.wire(mux.SD1, oled.SDA, color="steelblue")
    router.wire(mux.SC1, oled.SCL, color="steelblue")

    router.wire(xiao.GPIO3, us.TRIG, color="steelblue")
    router.wire(xiao.GPIO4, us.ECHO, color="steelblue")

    # I2S bus → amplifier. 24 kHz mono, matching Gemini TTS's native rate.
    router.wire(xiao.GPIO7, amp.BCLK, color="steelblue")
    router.wire(xiao.GPIO8, amp.LRC, color="steelblue")
    router.wire(xiao.GPIO9, amp.DIN, color="steelblue")

    router.wire(amp["OUT-"], spk.in1)
    router.wire(amp["OUT+"], spk.in2)

    # === Local stubs (power tags, servo/LED arrows, piezo branch) stay
    # hand-drawn — these aren't point-to-point nets between two components,
    # so the router adds nothing here. ===

    # PCA9685 servo + LED stubs — extend right into the cleared space.
    # elm.Arrow renders the arrowhead as an SVG path, not a glyph, so the
    # destination marker survives PNG rendering on hosts whose default sans
    # font lacks U+2192 (e.g. macOS Verdana).
    d.add(
        elm.Arrow()
        .right(2.5)
        .at(pca["PWM 6-7"])
        .label("Pan / Tilt SG90", loc="right", ofst=0.1, fontsize=10)
        .color("steelblue")
    )
    d.add(
        elm.Arrow()
        .right(2.5)
        .at(pca["PWM 0-5"])
        .label("2× RGB LED", loc="right", ofst=0.1, fontsize=10)
        .color("steelblue")
    )

    # === Power rails. ===
    # MCU 3V3 / 5V / GND tags on its outward (left) side.
    d.add(elm.Line().left(0.5).at(xiao["3V3"]))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(0.5).at(xiao.GND))
    d.add(elm.Ground())
    d.add(elm.Line().left(0.5).at(xiao["5V"]))
    d.add(elm.Vdd().label("+5V"))

    # Mux power (3V3 logic) — left side, away from I2C bus on right.
    d.add(elm.Line().left(0.5).at(mux.VCC))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(0.5).at(mux.GND))
    d.add(elm.Ground())

    # PCA9685: power tags pulled FAR left (1.5 units) so the Vdd labels clear
    # the I2C wires entering on the right.
    d.add(elm.Line().left(1.5).at(pca.VCC))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(1.5).at(pca["V+"]))
    d.add(elm.Vdd().label("+5V"))
    d.add(elm.Line().left(1.5).at(pca.GND))
    d.add(elm.Ground())

    # TB6612FNG: VCC = 3V3 logic, VM = 5V motor supply.
    d.add(elm.Line().left(0.5).at(tb.VCC))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(0.5).at(tb.VM))
    d.add(elm.Vdd().label("+5V"))
    d.add(elm.Line().left(0.5).at(tb.GND))
    d.add(elm.Ground())

    # OLED + ultrasonic only have pins on the LEFT side, so power tags also
    # extend leftward — going right would draw INTO the chip body. The Vdd
    # label sits well below the I2C wires entering at SDA/SCL.
    d.add(elm.Line().left(1.0).at(oled.VCC))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(1.0).at(oled.GND))
    d.add(elm.Ground())

    d.add(elm.Line().left(1.0).at(us.VCC))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(1.0).at(us.GND))
    d.add(elm.Ground())

    # Amp power on its outward-facing right side. VIN is 5 V — prefer a
    # separate feed from the boost converter rather than daisy-chaining off
    # the motor rail, and fit >=470 uF of bulk here (see WIRING.md).
    d.add(elm.Line().right(0.5).at(amp.VIN))
    d.add(elm.Vdd().label("+5V"))
    d.add(elm.Line().right(0.5).at(amp.GND))
    d.add(elm.Ground())

    # SD_MODE floating = (L+R)/2, which is what the firmware expects: it
    # duplicates the mono sample into both I2S slots. Tying it low shuts the
    # amplifier down.
    d.add(
        elm.Arrow()
        .right(2.0)
        .at(amp.SD)
        .label("float = (L+R)/2", loc="right", ofst=0.1, fontsize=10)
        .color("gray")
    )

    return d


if __name__ == "__main__":
    import sys
    from pathlib import Path

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("robocar_unified.svg")
    draw().save(str(out))
    print(f"wrote {out}")
