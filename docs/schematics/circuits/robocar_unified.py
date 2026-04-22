"""Robocar Unified — single-board wiring schematic.

XIAO ESP32-S3 Sense driving everything via an I2C multiplexer:
  - TCA9548A ch0 → PCA9685 → 2x RGB LEDs, 2x SG90 servos, TB6612FNG → motors
  - TCA9548A ch1 → SSD1306 OLED
Direct GPIO: STBY (motor enable), piezo, ultrasonic TRIG/ECHO.

Source of truth: packages/robocar/unified/WIRING.md and main/pin_config.h
"""

import schemdraw
import schemdraw.elements as elm

from components import (
    hc_sr04p,
    pca9685,
    ssd1306_oled,
    tb6612fng,
    tca9548a,
    xiao_esp32s3_sense,
)


def draw() -> schemdraw.Drawing:
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=11)

    # === Top row: MCU → I2C mux → PCA9685 (chained left-to-right). ===
    # TB6612FNG sits below-and-right of PCA so PCA's right side stays clear
    # for the LED/servo stub tags.
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

    # I2C bus: XIAO right side ↔ mux left side (top two pins).
    d.add(elm.Wire("-").at(xiao.GPIO5).to(mux.SDA).color("steelblue"))
    d.add(elm.Wire("-").at(xiao.GPIO6).to(mux.SCL).color("steelblue"))

    # Mux ch0 (SD0/SC0) → PCA9685 SDA/SCL.
    d.add(elm.Wire("-").at(mux.SD0).to(pca.SDA).color("steelblue"))
    d.add(elm.Wire("-").at(mux.SC0).to(pca.SCL).color("steelblue"))

    # PCA9685 PWM 8-13 group → TB6612FNG control cluster.
    # 6 logical signals (PWMA, AIN1/2, PWMB, BIN1/2) drawn as one trunk.
    d.add(elm.Wire("-|").at(pca["PWM 8-13"]).to(tb.PWMA).color("steelblue"))

    # === Motors on the far right, driven by TB outputs. ===
    motor_l = d.add(
        elm.Motor()
        .right()
        .at((tb.center.x + 5, tb.BO1.y - 0.5))
        .label("Left motor", loc="bot", ofst=0.4)
    )
    d.add(elm.Wire("-|").at(tb.BO1).to(motor_l.start))
    d.add(elm.Wire("|-").at(tb.BO2).to(motor_l.end))

    motor_r = d.add(
        elm.Motor()
        .right()
        .at((tb.center.x + 5, tb.AO1.y - 0.5))
        .label("Right motor", loc="bot", ofst=0.4)
    )
    d.add(elm.Wire("-|").at(tb.AO1).to(motor_r.start))
    d.add(elm.Wire("|-").at(tb.AO2).to(motor_r.end))

    # === STBY direct from MCU GPIO1 — routed UNDER everything. ===
    # Below TB's bottom edge so it can't collide with I2C buses or PCA stubs.
    stby_y = tb.GND.y - 2.5
    d.add(
        elm.Wire("|-").at(xiao.GPIO1).to((tb.STBY.x - 1.5, stby_y)).color("steelblue")
    )
    d.add(elm.Wire("-|").at((tb.STBY.x - 1.5, stby_y)).to(tb.STBY).color("steelblue"))

    # === Mux ch1 → SSD1306 OLED. ===
    # Explicit .right() locks orientation — without it, the OLED inherits the
    # previous wire's "up" direction and gets rotated 90°. Place OLED so its
    # left edge is to the RIGHT of mux's right edge; otherwise -| / |- wires
    # would route backward through the mux chip body.
    oled = d.add(
        ssd1306_oled()
        .right()
        .at((mux.center.x + 4, mux.center.y - 8))
        .anchor("center")
        .label("SSD1306 OLED\n0x3C, 128x64", loc="bot", ofst=0.4)
    )
    # -| (right-then-down) drops through the gap between mux right edge and
    # OLED left edge, then enters each pin cleanly from the left.
    d.add(elm.Wire("-|").at(mux.SD1).to(oled.SDA).color("steelblue"))
    d.add(elm.Wire("-|").at(mux.SC1).to(oled.SCL).color("steelblue"))

    # === PCA9685 servo + LED stubs — extend right into the cleared space. ===
    # Plain labelled lines (not Tag elements) keep the text readable when the
    # stub is short.
    d.add(
        elm.Line()
        .right(2.5)
        .at(pca["PWM 6-7"])
        .label("→ Pan / Tilt SG90", loc="right", ofst=0.1, fontsize=10)
        .color("steelblue")
    )
    d.add(
        elm.Line()
        .right(2.5)
        .at(pca["PWM 0-5"])
        .label("→ 2× RGB LED", loc="right", ofst=0.1, fontsize=10)
        .color("steelblue")
    )

    # === Ultrasonic + buzzer below the MCU. ===
    us = d.add(
        hc_sr04p()
        .right()
        .at((xiao.center.x + 4, xiao.GPIO3.y - 5))
        .anchor("center")
        .label("HC-SR04P\nultrasonic", loc="bot", ofst=0.4)
    )
    d.add(elm.Wire("|-").at(xiao.GPIO3).to(us.TRIG).color("steelblue"))
    d.add(elm.Wire("|-").at(xiao.GPIO4).to(us.ECHO).color("steelblue"))

    # Piezo buzzer on GPIO2 — small branch through resistor to ground.
    d.add(elm.Line().right(0.5).at(xiao.GPIO2))
    d.add(elm.Resistor().right().label("100 Ω"))
    buz = d.add(elm.Speaker().right().label("Piezo", loc="top", ofst=0.3))
    d.add(elm.Line().down(0.5).at(buz.in2))
    d.add(elm.Ground())

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

    return d


if __name__ == "__main__":
    import sys
    from pathlib import Path

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("robocar_unified.svg")
    draw().save(str(out))
    print(f"wrote {out}")
