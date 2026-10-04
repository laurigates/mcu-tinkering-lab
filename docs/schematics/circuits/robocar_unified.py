"""Robocar Unified — single-board wiring schematic.

XIAO ESP32-S3 Sense driving everything via an I2C multiplexer:
  - TCA9548A ch0 → PCA9685 → 2x RGB LEDs, 2x SG90 servos, TB6612FNG → motors
  - TCA9548A ch1 → SSD1306 OLED
  - TCA9548A ch2 → MCP23017 GPIO expander (optional; no roles assigned yet)
Direct GPIO: STBY (motor enable), piezo, ultrasonic TRIG/ECHO,
and I2S (D8-D10) → MAX98357A → speaker for the robot's voice (ADR-019).
On-module, drawn dashed: the Sense board's PDM microphone (GPIO42/41).

Every board with a vendor-sourced layout is drawn physically (#495): the
XIAO, TCA9548A, PCA9685, TB6612FNG and MAX98357A show every pad of their real
headers, on the real edge, in the real order, as seen from the component side,
read from docs/reference/boards/. None of them is rotated or mirrored — that
would draw a board nobody can hold — so placement works around the boards'
own pin order rather than choosing it. The SSD1306, HC-SR04P and MCP23017
keep schematic symbols: no vendor board file identifies the modules fitted.

Source of truth: packages/robocar/unified/WIRING.md and main/pin_config.h
"""

import sys
from pathlib import Path

import schemdraw
import schemdraw.elements as elm

from components import (
    hc_sr04p,
    max98357a,
    mcp23017,
    pca9685,
    pdm_microphone,
    ssd1306_oled,
    tb6612fng,
    tca9548a,
    xiao_esp32s3_sense,
)
from routing import Router, net_color

_REPO = Path(__file__).resolve().parents[3]
if str(_REPO / "tools") not in sys.path:
    sys.path.insert(0, str(_REPO / "tools"))
from hardware import parse_defines  # noqa: E402

# PCA9685 channel numbers come from the firmware, not from this file: a
# renumbered channel in pin_config.h moves the wire to the new pad instead of
# leaving the drawing on the old one.
_DEFINES = parse_defines([_REPO / "packages/robocar/unified/main/pin_config.h"])


def _channel(macro: str) -> str:
    return str(int(_DEFINES[macro]))


# PCA9685 channel -> TB6612FNG pin. The channel half is the firmware's; the
# pin half is which motor-driver input each define says it drives.
MOTOR_LINES = (
    ("MOTOR_RIGHT_PWM_CHANNEL", "PWMA"),
    ("MOTOR_RIGHT_IN2_CHANNEL", "AIN2"),
    ("MOTOR_RIGHT_IN1_CHANNEL", "AIN1"),
    ("MOTOR_LEFT_IN1_CHANNEL", "BIN1"),
    ("MOTOR_LEFT_IN2_CHANNEL", "BIN2"),
    ("MOTOR_LEFT_PWM_CHANNEL", "PWMB"),
)
LED_CHANNELS = (
    "LED_LEFT_R_CHANNEL",
    "LED_LEFT_G_CHANNEL",
    "LED_LEFT_B_CHANNEL",
    "LED_RIGHT_R_CHANNEL",
    "LED_RIGHT_G_CHANNEL",
    "LED_RIGHT_B_CHANNEL",
)
SERVO_CHANNELS = ("SERVO_PAN_CHANNEL", "SERVO_TILT_CHANNEL")


def _tag(d, pin, direction: str, length: float, kind: str, label: str = "") -> None:
    """A power or ground tag on ``pin``, leading ``direction`` for ``length``."""
    line = getattr(elm.Line(), direction)(length).at(pin)
    d.add(line.color(net_color("power" if kind == "power" else "ground")))
    if kind == "power":
        d.add(elm.Vdd().label(label).color(net_color("power")))
    else:
        d.add(elm.Ground().color(net_color("ground")))


def _stubs(d, ic, channels, label: str, net: str) -> None:
    """Short leads down from PCA9685 channel pads, one label for the group."""
    xs = []
    for macro in channels:
        pad = ic[_channel(macro)]
        d.add(elm.Arrow().down(1.0).at(pad).color(net_color(net)))
        xs.append(pad[0])
    y = ic[_channel(channels[0])][1] - 1.0
    d.add(
        elm.Label()
        .at(((min(xs) + max(xs)) / 2, y - 0.45))
        .label(label, fontsize=10, color=net_color(net))
    )


def draw() -> schemdraw.Drawing:
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=11)

    # === Components first, so every net below routes with full obstacle
    # awareness (the auto-router only avoids components already placed). ===
    #
    # The XIAO's signal pads are on its left edge (D0-D6) and its power and
    # I2S pads on the right, so the I2C chain runs off to the left: mux below
    # and left of the module, PCA9685 further left, motor driver under the
    # PCA9685's channel row. The amplifier sits above-right, facing the I2S
    # trio.
    xiao = d.add(
        xiao_esp32s3_sense(layout="physical")
        .right()
        .label("XIAO ESP32-S3 Sense\n(USB-C at top)", loc="bot", ofst=0.4)
    )

    mux = d.add(
        tca9548a(layout="physical")
        .right()
        .at((xiao.center.x - 5, xiao.center.y - 11))
        .anchor("center")
        .label("TCA9548A\n0x70", loc="bot", ofst=0.4)
    )

    pca = d.add(
        pca9685(layout="physical")
        .right()
        .at((mux.center.x - 20, mux.center.y - 3))
        .anchor("center")
        .label("PCA9685\n0x40 @ 200Hz", loc="top", ofst=0.4)
    )

    tb = d.add(
        tb6612fng(layout="physical")
        .right()
        .at((pca.center.x - 4, pca.center.y - 15))
        .anchor("center")
        .label("TB6612FNG", loc="bot", ofst=0.4)
    )

    # Motors on the far left, driven by the outputs down TB's left edge.
    # A01/A02 are channel A, the right motor; B01/B02 channel B, the left.
    # Stood upright and drawn one pad pitch long (scaled down so the body
    # fits), so each motor spans exactly its own pair of output pads and
    # neither pair of wires has to pass the other motor. The left motor stands
    # further out so its top terminal's lead clears the right motor's bottom
    # one.
    tb_left = tb["A01"].x
    pitch = tb["A01"].y - tb["A02"].y
    motor_r = d.add(
        elm.Motor()
        .down(pitch)
        .scale(0.625)
        .at((tb_left - 2, tb["A01"].y))
        .label("Right motor", loc="bot", ofst=0.3)
    )
    motor_l = d.add(
        elm.Motor()
        .up(pitch)
        .scale(0.625)
        .at((tb_left - 3.5, tb["B01"].y))
        .label("Left motor", loc="top", ofst=0.3)
    )

    # Mux ch1 → SSD1306 OLED. Explicit .right() locks orientation — without
    # it, the OLED inherits the previous element's "up" direction and gets
    # rotated 90°.
    oled = d.add(
        ssd1306_oled()
        .right()
        .at((mux.center.x + 1, mux.center.y - 12))
        .anchor("center")
        .label("SSD1306 OLED\n0x3C, 128x64", loc="bot", ofst=0.4)
    )

    # MCP23017 on mux ch2, which leaves the mux's right edge at the bottom.
    # Optional hardware: the firmware boots fine without the board fitted.
    mcp = d.add(
        mcp23017()
        .right()
        .at((mux.center.x + 10, mux.center.y - 9))
        .anchor("center")
        .label("MCP23017\n0x20 (optional)", loc="bot", ofst=0.4)
    )

    # Onboard PDM microphone (issue #486): on the Sense expansion board, wired
    # to GPIO42/GPIO41 there and not to any header pad, so it is drawn dashed
    # with dashed gray leads into the XIAO body instead of routed nets — there
    # is nothing for a builder to connect. Up and to the left of the module,
    # clear of the I2S bus that leaves the right edge for the amplifier. It
    # shares I2S0 with the MAX98357A: PDM RX exists only on I2S0 on the
    # ESP32-S3, and the RX channel needs its own i2s_new_channel() call or the
    # 16 kHz mic is clocked off the 24 kHz amp.
    xiao_box = xiao.get_bbox(transform=True, includetext=False)
    mic = d.add(
        pdm_microphone()
        .right()
        .at((xiao.center.x - 5, xiao_box.ymax + 3))
        .anchor("center")
        .label(
            "PDM mic (MSM261D)\non Sense board, no wiring\nI2S0 RX, same port as amp",
            loc="top",
            ofst=0.4,
        )
    )
    # CLK (the upper pin) drops the further right, so the two leads nest.
    for pin, drop_x in (
        (mic.DATA, xiao.center.x - 0.75),
        (mic.CLK, xiao.center.x - 0.25),
    ):
        d.add(
            elm.Wire("-|")
            .at(pin)
            .to((drop_x, xiao_box.ymax))
            .linestyle("--")
            .color("gray")
        )

    # Ultrasonic left of the XIAO, level with TRIG/ECHO (D2/D3).
    us = d.add(
        hc_sr04p()
        .right()
        .reverse()
        .at((xiao.center.x - 11, xiao["GPIO3"].y))
        .anchor("TRIG")
        .label("HC-SR04P\nultrasonic", loc="bot", ofst=0.4)
    )

    # MAX98357A above-right: its header is the bottom edge, so the I2S trio
    # leaves the XIAO's right edge and climbs into it from below. The speaker
    # terminal is the top edge.
    amp = d.add(
        max98357a(layout="physical")
        .right()
        .at((xiao.center.x + 8.5, xiao.center.y + 7))
        .anchor("center")
        .label("MAX98357A", loc="right", ofst=0.4)
    )

    # Speaker above the amp's terminal block. 8 Ω is the safer starting
    # point — it roughly halves peak current versus 4 Ω on a rail that already
    # has brownout detection disabled for motor inrush.
    spk = d.add(
        elm.Speaker()
        .up()
        .at((amp.center.x, amp["VO-"].y + 1.5))
        .label("8 Ω  2-3 W", loc="bot", ofst=0.4)
    )

    # Piezo buzzer on GPIO2 (D1) — a small branch through a resistor to
    # ground, leaving the XIAO's left edge. Placed before routing: the
    # resistor/speaker are real components, not cosmetic tags.
    d.add(elm.Line().left(0.5).at(xiao.GPIO2))
    d.add(elm.Resistor().left().label("100 Ω"))
    buz = d.add(elm.Speaker().left().label("Piezo", loc="lft", ofst=0.3))
    d.add(elm.Line().down(0.5).at(buz.in2).color(net_color("ground")))
    d.add(elm.Ground().color(net_color("ground")))

    # === Power rails. ===
    # Drawn before the nets so the router sees the tags (#591): it charges a
    # wire for running over a power/ground tag it is not wired to, and a tag
    # added after routing is invisible to it.
    # XIAO 5V / GND / 3V3 sit together at the top of its right edge.
    # The 5V/3V3 tags reach just past the router's stub point, so the I2S
    # wires leaving the pads below pay the tag charge if they climb through
    # them and run out along their own rows instead. GND reaches further so
    # its symbol clears the +3V3 label one pad down.
    _tag(d, xiao["5V"], "right", 0.75, "power", "+5V")
    _tag(d, xiao["GND"], "right", 1.5, "ground")
    _tag(d, xiao["3V3"], "right", 0.75, "power", "+3V3")

    # Mux power at the top of its left edge.
    _tag(d, mux.VIN, "left", 0.5, "power", "+3V3")
    _tag(d, mux.GND, "left", 0.5, "ground")

    # PCA9685: logic, servo rail and ground come in on the same right-edge
    # header as the I2C feed from the mux; the left header chains onward and
    # is left open. V+ is 5 V. The terminal block (top) is the alternative,
    # reverse-protected V+ input and is not used here.
    _tag(d, pca["VCC.R5"], "right", 0.5, "power", "+3V3")
    _tag(d, pca["V+.R6"], "right", 0.5, "power", "+5V")
    _tag(d, pca["GND.R1"], "right", 0.5, "ground")

    # TB6612FNG: VCC = 3V3 logic, VM = 5V motor supply, at the top of its
    # left edge.
    _tag(d, tb.VM, "left", 0.5, "power", "+5V")
    _tag(d, tb.VCC, "left", 0.5, "power", "+3V3")
    _tag(d, tb["GND.L3"], "left", 0.5, "ground")

    # OLED, ultrasonic and MCP23017 have their pins on the left, so their tags
    # extend leftward — going right would draw into the chip body.
    _tag(d, oled.VCC, "left", 1.0, "power", "+3V3")
    _tag(d, oled.GND, "left", 1.0, "ground")
    _tag(d, us.VCC, "right", 1.0, "power", "+3V3")
    _tag(d, us.GND, "right", 1.0, "ground")
    _tag(d, mcp.VCC, "left", 1.0, "power", "+3V3")
    _tag(d, mcp.GND, "left", 1.0, "ground")

    # Amp power on its bottom header. VIN is 5 V — take a separate feed from
    # the LM2596 regulator's output terminal rather than daisy-chaining off
    # the motor rail, and fit >=470 uF of bulk here (see WIRING.md).
    _tag(d, amp.Vin, "down", 0.5, "power", "+5V")
    _tag(d, amp.GND, "down", 0.5, "ground")

    # === Nets: auto-routed orthogonal, obstacle-avoiding wires. ===
    router = Router(d)

    # I2C bus: XIAO D4/D5 → the mux's upstream pins near the top of its left
    # edge.
    router.wire(xiao.GPIO5, mux.SDA, net="i2c")
    router.wire(xiao.GPIO6, mux.SCL, net="i2c")

    # Mux ch0 (bottom of its left edge) → the PCA9685's right-edge header.
    router.wire(mux.SD0, pca["SDA.R4"], net="i2c")
    router.wire(mux.SC0, pca["SCL.R3"], net="i2c")

    # PCA9685 channel pads → TB6612FNG control row. The driver's right edge
    # runs PWMA, AIN2, AIN1, STBY, BIN1, BIN2, PWMB top to bottom, and the
    # channels run left to right in the same order, so the six wires nest.
    for macro, pin in MOTOR_LINES:
        router.wire(pca[_channel(macro)], tb[pin], net="pwm")

    router.wire(tb["A01"], motor_r.start, net="load")
    router.wire(tb["A02"], motor_r.end, net="load")
    router.wire(tb["B01"], motor_l.start, net="load")
    router.wire(tb["B02"], motor_l.end, net="load")

    # STBY direct from MCU GPIO1 (D0) to the gap in the control row.
    router.wire(xiao.GPIO1, tb.STBY, net="signal")

    router.wire(mux.SD1, oled.SDA, net="i2c")
    router.wire(mux.SC1, oled.SCL, net="i2c")

    router.wire(mux.SD2, mcp.SDA, net="i2c")
    router.wire(mux.SC2, mcp.SCL, net="i2c")

    router.wire(xiao.GPIO3, us.TRIG, net="sensor")
    router.wire(xiao.GPIO4, us.ECHO, net="sensor")

    # I2S bus → amplifier. 24 kHz mono, matching Gemini TTS's native rate.
    router.wire(xiao.GPIO7, amp.BCLK, net="i2s")
    router.wire(xiao.GPIO8, amp.LRC, net="i2s")
    router.wire(xiao.GPIO9, amp.DIN, net="i2s")

    router.wire(amp["VO-"], spk.in1, net="load")
    router.wire(amp["VO+"], spk.in2, net="load")

    # === Local stubs (servo/LED and spare-GPIO arrows) stay hand-drawn —
    # these aren't point-to-point nets between two components, so the router
    # adds nothing here. ===

    # PCA9685 servo + LED channels: short leads down from each pad in use.
    # elm.Arrow renders the arrowhead as an SVG path, not a glyph, so the
    # destination marker survives PNG rendering on hosts whose default sans
    # font lacks U+2192 (e.g. macOS Verdana).
    _stubs(d, pca, LED_CHANNELS, "2× RGB LED", "pwm")
    _stubs(d, pca, SERVO_CHANNELS, "Pan / Tilt\nSG90", "pwm")

    # MCP23017 ports: 16 generic GPIOs, no roles assigned yet — direction is
    # set per pin at runtime. (A0-A2 are strapped to GND for 0x20; that's in
    # the component label rather than drawn, since they carry no signal.)
    d.add(
        elm.Arrow()
        .right(2.5)
        .at(mcp["GPA0-7"])
        .label("8 spare GPIO", loc="right", ofst=0.1, fontsize=10)
        .color(net_color("signal"))
    )
    d.add(
        elm.Arrow()
        .right(2.5)
        .at(mcp["GPB0-7"])
        .label("8 spare GPIO", loc="right", ofst=0.1, fontsize=10)
        .color(net_color("signal"))
    )

    # SD_MODE floating = (L+R)/2, which is what the firmware expects: it
    # duplicates the mono sample into both I2S slots. Tying it low shuts the
    # amplifier down. Gray, not a net class: the arrow is an annotation on a
    # pin left unconnected, and a class colour would claim it carries a net.
    d.add(
        elm.Arrow()
        .down(2.5)
        .at(amp.SD)
        .label("float = (L+R)/2", loc="end", ofst=(0, -0.3), fontsize=10)
        .color("gray")
    )

    # Draw the routed nets last: finish() hops every hand-drawn lead already
    # in the drawing and dots every junction with one (#493), and the power
    # stubs drawn before routing cross routed wires. Routing itself was fixed
    # at wire() time, so this moves only the Paths' place in the SVG's paint
    # order.
    router.finish()

    return d


if __name__ == "__main__":
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("robocar_unified.svg")
    draw().save(str(out))
    print(f"wrote {out}")
