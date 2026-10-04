"""Reusable schematic component blocks for MCU Tinkering Lab projects.

Each factory returns a fresh schemdraw :class:`~schemdraw.elements.Ic`. Pin
names match the firmware defines (e.g. `GPIO5`, `BCLK`) so anchors can be
referenced by name when wiring:

    esp = d.add(esp32_s3_zero())
    amp = d.add(max98357a().at((esp.center.x + 9, esp.center.y)).anchor('center'))
    d.add(elm.Wire('-').at(esp.GPIO5).to(amp.BCLK))

Conventions kept consistent across factories so chips line up:

- Pins are listed *bottom-to-top* on the L/R sides (schemdraw's convention —
  the first listed pin renders at the bottom).
- Chips that connect to each other keep the same pin count on facing sides so
  default auto-spacing aligns the pins when the chips share a y-center.
- Factories do not set a center label; let the circuit add one via
  ``ic.label('Name', loc='top')`` to avoid collisions with pin labels.

Those conventions describe the ``layout="schematic"`` symbols, whose pin order
was chosen for the router. A board with a vendor-sourced layout also has a
``layout="physical"`` symbol (ADR-023 stage 6, #495): every pad of the real
header, on the real edge, in the real order, at ``PITCH`` per pad. Its pad list
is read from ``docs/reference/boards/<board>.md`` through
``tools/hardware/layout.py`` — never typed here — so the drawing and the board
reference cannot disagree. Physical order forfeits the routing conveniences
above; the crossing marks (#493) carry the cost.

Add a new component by writing another factory.
"""

from __future__ import annotations

import sys
from collections.abc import Callable
from pathlib import Path

import schemdraw.elements as elm
from schemdraw.backends.svg import text_size

_TOOLS = Path(__file__).resolve().parents[2] / "tools"
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from hardware import ModuleLayout, Pad, board_layout  # noqa: E402

# Drawing units per pad on a physical symbol. True 2.54 mm would be 0.35
# units — 1.4 router cells, too tight to fit a wire between adjacent pins —
# so the symbols are stylised-physical: true order and side, not true scale.
PITCH = 1.0

LAYOUTS = ("schematic", "physical")

# Ic draws pin labels at its default `lsize`; the box is sized for that.
_LABEL_SIZE = elm.Ic._element_defaults["lsize"]
_LABEL_OFST = elm.Ic._element_defaults["lofst"]


def _check_layout(layout: str) -> None:
    if layout not in LAYOUTS:
        raise ValueError(f"layout must be one of {LAYOUTS}, got {layout!r}")


def _text_width(text: str) -> float:
    return text_size(text, size=_LABEL_SIZE)[0] / 72 * 2


def _text_height() -> float:
    return _LABEL_SIZE / 72 * 2


def physical_module(
    layout: ModuleLayout,
    *,
    name: Callable[[Pad], str] = lambda pad: pad.name,
    number: Callable[[Pad], str] = lambda pad: "",
    anchor: Callable[[Pad], str] = lambda pad: pad.anchor,
) -> elm.Ic:
    """An :class:`~schemdraw.elements.Ic` with one pin per pad of ``layout``.

    Pads keep their edge and their order along it — top to bottom on the left
    and right, left to right on the top and bottom, as the board is seen from
    its component side — at ``PITCH`` apart, centred on the edge. ``name`` is
    the label inside the box, ``number`` the text outside it, ``anchor`` the
    name a circuit wires to (by default the layout's own, which is unique per
    board even where the silkscreen repeats ``GND``).

    The box is sized from the pad counts, not chosen per board: the long edge
    holds its pads at ``PITCH``, and an edge carrying top or bottom pads gets
    enough margin that their labels clear the first left/right label.
    """

    def widest(side: str) -> float:
        return max((_text_width(name(p)) for p in layout.side(side)), default=0.0)

    # A top/bottom label wider than most of a pitch would run into its
    # neighbour, so that edge's labels stand on end instead.
    upright = {s: widest(s) > 0.8 * PITCH for s in ("T", "B")}

    pins: list[elm.IcPin] = []
    for side in ("L", "R", "T", "B"):
        pads = layout.side(side)
        # schemdraw lists left/right pins bottom to top; the layout runs top
        # to bottom. Top/bottom pins run left to right in both.
        for pad in reversed(pads) if side in ("L", "R") else pads:
            pins.append(
                elm.IcPin(
                    name=name(pad),
                    pin=number(pad),
                    side=side,
                    anchorname=anchor(pad),
                    rotation=90 if upright.get(side) else 0,
                )
            )

    n_lr = max(len(layout.side("L")), len(layout.side("R")), 1)
    n_tb = max(len(layout.side("T")), len(layout.side("B")), 1)
    # Vertical margin: a top/bottom label sits `lofst` inside the edge and is
    # one text-height tall, or its full width when stood on end; the nearest
    # left/right label must start clear of it.
    reach = [
        widest(s) if upright[s] else _text_height()
        for s in ("T", "B")
        if layout.side(s)
    ]
    edge_v = _LABEL_OFST + max(reach) + 0.3 if reach else 0.5
    height = max((n_lr - 1) * PITCH + 2 * edge_v, 2.0)
    width = max(
        (n_tb - 1) * PITCH + 2 * 0.5,
        widest("L") + widest("R") + 4 * _LABEL_OFST + 0.5,
        2.0,
    )
    return elm.Ic(pins=pins, size=(width, height), pinspacing=PITCH)


def _gpio_label(pad: Pad) -> str:
    """Firmware name for a GPIO pad (``GPIO5``), the silkscreen for the rest."""
    return f"GPIO{pad.gpio}" if pad.gpio is not None else pad.name


def _silkscreen_if_gpio(pad: Pad) -> str:
    """The ``Dn`` silkscreen, printed outside the box beside a GPIO pad."""
    return pad.name if pad.gpio is not None else ""


def esp32_s3_zero() -> elm.Ic:
    """Waveshare ESP32-S3-Zero dev board (compact, castellated).

    Right side holds the I2S pins (GPIO5/6/7, top-to-bottom) to align with
    :func:`max98357a`. GPIO2 (status LED) is routed out the top. GPIO8/GPIO9
    sit on the bottom for the optional Drone-mode piezo pair.
    """
    return elm.Ic(
        pins=[
            # Left (bottom → top): GND, 3V3, 5V, GPIO2
            elm.IcPin(name="GND", side="L", pin="3"),
            elm.IcPin(name="3V3", side="L", pin="2"),
            elm.IcPin(name="5V", side="L", pin="1"),
            elm.IcPin(name="GPIO2", side="L", pin="4"),
            # Right (bottom → top): GPIO7, GPIO6, GPIO5
            elm.IcPin(name="GPIO7", side="R", pin="7"),
            elm.IcPin(name="GPIO6", side="R", pin="6"),
            elm.IcPin(name="GPIO5", side="R", pin="5"),
            # Bottom — optional piezo-pair outputs (Drone mode only)
            elm.IcPin(name="GPIO8", side="B", pin="8", pos=0.3),
            elm.IcPin(name="GPIO9", side="B", pin="9", pos=0.7),
        ],
        size=(3, 5),
    )


def max98357a(layout: str = "schematic") -> elm.Ic:
    """MAX98357A mono I2S Class-D amplifier breakout (Adafruit #3006 pinout).

    ``layout="physical"``: the board as built — the 7-pin header along the
    bottom edge (LRC, BCLK, DIN, GAIN, SD, GND, Vin, left to right) and the
    speaker terminal (VO-, VO+) along the top, from
    ``docs/reference/boards/adafruit-max98357a.md``.

    ``layout="schematic"``: I2S pins are placed on the *left* side so they face
    the MCU when the amp is drawn to the right of it — wires don't have to
    cross the chip body. This order is chosen for the router, not the board.
    """
    _check_layout(layout)
    if layout == "physical":
        return physical_module(board_layout("adafruit-max98357a"))
    return elm.Ic(
        pins=[
            # Left (bottom → top) — I2S bus, faces MCU
            elm.IcPin(name="DIN", side="L"),
            elm.IcPin(name="LRC", side="L"),
            elm.IcPin(name="BCLK", side="L"),
            # Right (bottom → top) — power + config, faces outward
            elm.IcPin(name="GAIN", side="R"),
            elm.IcPin(name="SD", side="R"),
            elm.IcPin(name="GND", side="R"),
            elm.IcPin(name="VIN", side="R"),
            # Bottom — speaker outputs. Their labels have to clear DIN and GAIN
            # (bottom corners of the adjacent sides, same height) *and* each
            # other. At the original 4-wide box there was no gap that did both:
            # 0.15/0.85 overlapped DIN/GAIN, 0.35/0.65 overlapped each other.
            # Widening the body to 6 opens enough room for both clearances.
            elm.IcPin(name="OUT-", side="B", pin="-", pos=0.3),
            elm.IcPin(name="OUT+", side="B", pin="+", pos=0.7),
        ],
        size=(6, 5),
    )


def xiao_esp32s3_sense(layout: str = "schematic") -> elm.Ic:
    """Seeed XIAO ESP32-S3 Sense — ESP32-S3 + OV3660 camera + PDM mic + 8MB PSRAM.

    Camera/PSRAM pins are internal to the Sense module and don't conflict with
    the header pins. D8-D10 double as the Sense expansion board's microSD SPI
    bus — using them for I2S gives up the card slot. Further digital I/O must
    go through the MCP23017 on the I2C mux.

    ``layout="physical"``: all 14 header pads, 7 per side, read from the
    pin-mapping table in ``docs/reference/boards/xiao-esp32s3.md`` (the Sense
    board states the same external pinout). USB-C is at the top: D0-D6 down the
    left, 5V/GND/3V3 then D10-D7 down the right. GPIO pads are labelled and
    anchored by their firmware name (``GPIO5``) with the ``Dn`` silkscreen
    outside; D6/D7 (GPIO43/44) are drawn though nothing uses them.

    ``layout="schematic"``: 12 pins, 3 left / 9 right, ordered for the router —
    SCL above SDA to run parallel to a mux on the right, and the I2S trio at
    the top towards an amplifier above-right. It is not the board's pinout.
    """
    _check_layout(layout)
    if layout == "physical":
        return physical_module(
            board_layout("xiao-esp32s3"),
            name=_gpio_label,
            number=_silkscreen_if_gpio,
            anchor=_gpio_label,
        )
    return elm.Ic(
        pins=[
            # Left (bottom → top): power rails facing outward
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="3V3", side="L"),
            elm.IcPin(name="5V", side="L"),
            # Right (bottom → top): GPIOs facing peripherals
            elm.IcPin(name="GPIO4", side="R", pin="D3"),  # ECHO
            elm.IcPin(name="GPIO3", side="R", pin="D2"),  # TRIG
            elm.IcPin(name="GPIO2", side="R", pin="D1"),  # Buzzer
            elm.IcPin(name="GPIO1", side="R", pin="D0"),  # STBY
            elm.IcPin(name="GPIO5", side="R", pin="D4"),  # SDA
            elm.IcPin(name="GPIO6", side="R", pin="D5"),  # SCL
            elm.IcPin(name="GPIO9", side="R", pin="D10"),  # I2S DIN
            elm.IcPin(name="GPIO8", side="R", pin="D9"),  # I2S LRCLK
            elm.IcPin(name="GPIO7", side="R", pin="D8"),  # I2S BCLK
        ],
        size=(3.5, 10),
    )


def pdm_microphone() -> elm.Ic:
    """MSM261D PDM microphone on the XIAO ESP32-S3 Sense expansion board.

    Nothing here is soldered: GPIO42 (CLK) and GPIO41 (DATA) run to the mic on
    the Sense board itself and are not brought out to a header. The outline is
    dashed so the block reads as on-module rather than as another breakout;
    the circuit should still say so in its label. Pin numbers carry the GPIOs
    from ``pin_config.h`` because there is no header ``Dn`` to print.

    Pins face right so the block can sit up and to the left of the XIAO, clear
    of the I2S bus that runs over the module's top edge to the amplifier.
    The list runs bottom to top, so CLK is drawn above DATA; leads that turn
    down into the XIAO then nest instead of crossing when CLK drops the
    further right of the two.
    """
    return elm.Ic(
        pins=[
            # Right (bottom → top)
            elm.IcPin(name="DATA", side="R", pin="GPIO41"),
            elm.IcPin(name="CLK", side="R", pin="GPIO42"),
        ],
        size=(3, 2),
    ).linestyle("--")


def tca9548a(layout: str = "schematic") -> elm.Ic:
    """TCA9548A 8-channel I2C multiplexer (Adafruit #2717 / generic breakout).

    ``layout="physical"``: both 12-pin headers of the Adafruit board, from
    ``docs/reference/boards/adafruit-tca9548a.md`` — power, upstream bus,
    RST, A0-A2 and channels 0-1 down the left; channels 7 to 2 down the right.

    ``layout="schematic"``: only the channels used by robocar-unified are
    exposed (ch0 = PCA9685, ch1 = OLED, ch2 = MCP23017). Upstream I2C + power
    on the left, downstream channels on the right, ordered for the router.
    """
    _check_layout(layout)
    if layout == "physical":
        return physical_module(board_layout("adafruit-tca9548a"))
    return elm.Ic(
        pins=[
            # Left (bottom → top): power + upstream I2C
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="VCC", side="L"),
            elm.IcPin(name="SDA", side="L"),
            elm.IcPin(name="SCL", side="L"),
            # Right (bottom → top): downstream channel pairs ordered SDx/SCx so
            # SCL ends up *above* SDA on each pair — matches the canonical
            # PCA9685/SSD1306 left-side pin order and avoids bus crossings.
            # Channels descend top-to-bottom (ch0 highest) so each peripheral
            # can be drawn progressively further down the page.
            elm.IcPin(name="SD2", side="R"),
            elm.IcPin(name="SC2", side="R"),
            elm.IcPin(name="SD1", side="R"),
            elm.IcPin(name="SC1", side="R"),
            elm.IcPin(name="SD0", side="R"),
            elm.IcPin(name="SC0", side="R"),
        ],
        size=(3.5, 7),
    )


def mcp23017() -> elm.Ic:
    """MCP23017 16-bit I2C GPIO expander (1953W breakout).

    Reached through TCA9548A ch2 rather than the primary bus. The 16 GPIOs
    are grouped by port on the right — drawing all 16 would swamp the
    schematic, and no role is assigned to any of them yet (they are exercised
    only from the serial console's ``gpio`` command).

    A0-A2 are not drawn: they are strapped to GND for address 0x20, which the
    circuit annotates as a label rather than three stub pins.
    """
    return elm.Ic(
        pins=[
            # Left (bottom → top): power + I2C from the mux
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="VCC", side="L"),
            elm.IcPin(name="SDA", side="L"),
            elm.IcPin(name="SCL", side="L"),
            # Right (bottom → top): grouped ports
            elm.IcPin(name="GPB0-7", side="R"),  # pins 8-15
            elm.IcPin(name="GPA0-7", side="R"),  # pins 0-7
        ],
        size=(4, 5),
    )


def pca9685(layout: str = "schematic") -> elm.Ic:
    """PCA9685 16-channel 12-bit I2C PWM driver (Adafruit #815 breakout).

    ``layout="physical"``: the board as built, from
    ``docs/reference/boards/adafruit-pca9685.md`` — the two identical 6-pin
    side headers (GND, OE, SCL, SDA, VCC, V+, top to bottom), the power
    terminal (V+, GND) on top and the sixteen channels 0-15 along the bottom.
    Each channel is drawn as its PWM pad; the V+/GND pads under it are the
    terminal's rails. Repeated names are anchored by edge and position
    (``SCL.R3``, ``GND.T2``).

    ``layout="schematic"``: 16 PWM outputs are grouped on the right by
    destination, with labels recording the channel ranges from
    ``pin_config.h``. Not the board's pinout.
    """
    _check_layout(layout)
    if layout == "physical":
        return physical_module(board_layout("adafruit-pca9685"))
    return elm.Ic(
        pins=[
            # Left (bottom → top): power + I2C from mux
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="VCC", side="L"),
            elm.IcPin(name="V+", side="L"),
            elm.IcPin(name="SDA", side="L"),
            elm.IcPin(name="SCL", side="L"),
            # Right (bottom → top): grouped PWM channels by function
            elm.IcPin(name="PWM 8-13", side="R"),  # motor IN1/IN2/PWM x2
            elm.IcPin(name="PWM 6-7", side="R"),  # pan/tilt servos
            elm.IcPin(name="PWM 0-5", side="R"),  # 2x RGB LED (R/G/B x2)
        ],
        size=(4, 5),
    )


def tb6612fng(layout: str = "schematic") -> elm.Ic:
    """TB6612FNG dual H-bridge motor driver (SparkFun ROB-14451 / generic breakout).

    PWMA/AIN1/AIN2 drive channel A (right motor), PWMB/BIN1/BIN2 drive B
    (left motor), STBY is a global enable from the MCU.

    ``layout="physical"``: both 8-pin rows of the SparkFun board, from
    ``docs/reference/boards/sparkfun-tb6612fng.md`` — supplies and outputs
    down the left (VM, VCC, GND, A01, A02, B02, B01, GND), control down the
    right (PWMA, AIN2, AIN1, STBY, BIN1, BIN2, PWMB, GND). Outputs carry the
    board's zero (``A01``); the three GNDs are ``GND.L3``, ``GND.L8``,
    ``GND.R8``.

    ``layout="schematic"``: control signals on the left face the PCA9685;
    motor outputs on the right. Ordered for the router, not the board.
    """
    _check_layout(layout)
    if layout == "physical":
        return physical_module(board_layout("sparkfun-tb6612fng"))
    return elm.Ic(
        pins=[
            # Left (bottom → top): power, enable, then per-channel control
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="VCC", side="L"),
            elm.IcPin(name="VM", side="L"),
            elm.IcPin(name="STBY", side="L"),
            elm.IcPin(name="PWMB", side="L"),
            elm.IcPin(name="BIN2", side="L"),
            elm.IcPin(name="BIN1", side="L"),
            elm.IcPin(name="PWMA", side="L"),
            elm.IcPin(name="AIN2", side="L"),
            elm.IcPin(name="AIN1", side="L"),
            # Right (bottom → top): motor outputs (B = left motor, A = right)
            elm.IcPin(name="BO2", side="R"),
            elm.IcPin(name="BO1", side="R"),
            elm.IcPin(name="AO2", side="R"),
            elm.IcPin(name="AO1", side="R"),
        ],
        size=(4, 8),
    )


def ssd1306_oled() -> elm.Ic:
    """SSD1306 128x64 I2C OLED display (4-pin breakout)."""
    return elm.Ic(
        pins=[
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="VCC", side="L"),
            elm.IcPin(name="SDA", side="L"),
            elm.IcPin(name="SCL", side="L"),
        ],
        size=(3.5, 3),
    )


def hc_sr04p() -> elm.Ic:
    """HC-SR04P / RCWL-1601 3.3V-compatible ultrasonic rangefinder."""
    return elm.Ic(
        pins=[
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="ECHO", side="L"),
            elm.IcPin(name="TRIG", side="L"),
            elm.IcPin(name="VCC", side="L"),
        ],
        size=(3.5, 3),
    )


def xiao_rp2350() -> elm.Ic:
    """Seeed XIAO RP2350 — RP2350 in the XIAO form factor.

    Left side carries the power rails (facing outward). Right side lists the
    side-header GPIOs in the order they connect to peripherals top-to-bottom:
    MPU6050 I2C + INT at the top, then the two DRV8825 STEP/DIR pairs and the
    shared nENABLE below. Pin labels show the XIAO ``Dn`` silkscreen numbers.

    Source of truth: packages/robotics/balancebot/src/pin_config.h
    """
    return elm.Ic(
        pins=[
            # Left (bottom → top): power rails facing outward
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="3V3", side="L"),
            elm.IcPin(name="5V", side="L"),
            # Right (bottom → top): GPIOs facing peripherals
            elm.IcPin(name="GPIO0", side="R", pin="D6"),  # right DRV8825 DIR
            elm.IcPin(name="GPIO3", side="R", pin="D10"),  # right DRV8825 STEP
            elm.IcPin(name="GPIO1", side="R", pin="D7"),  # shared nENABLE
            elm.IcPin(name="GPIO4", side="R", pin="D9"),  # left DRV8825 DIR
            elm.IcPin(name="GPIO2", side="R", pin="D8"),  # left DRV8825 STEP
            elm.IcPin(name="GPIO5", side="R", pin="D3"),  # MPU6050 INT
            elm.IcPin(name="GPIO6", side="R", pin="D4"),  # MPU6050 SDA
            elm.IcPin(name="GPIO7", side="R", pin="D5"),  # MPU6050 SCL
        ],
        size=(3.5, 9),
    )


def mpu6050() -> elm.Ic:
    """GY-521 MPU6050 6-axis IMU breakout.

    All pins on the left so they face the MCU when the breakout is drawn to
    the right of it — wires don't cross the chip body. Bottom→top ordered so
    SCL sits above SDA, matching the XIAO's right-side I2C order (no bus
    crossing). The breakout's onboard regulator is bypassed by feeding VCC
    from 3V3 directly (see WIRING.md).
    """
    return elm.Ic(
        pins=[
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="VCC", side="L"),
            elm.IcPin(name="INT", side="L"),
            elm.IcPin(name="SDA", side="L"),
            elm.IcPin(name="SCL", side="L"),
        ],
        size=(3.5, 4),
    )


def drv8825() -> elm.Ic:
    """DRV8825 stepper driver carrier (Pololu / generic breakout).

    Control + power on the left (facing the MCU); the two bipolar coil-output
    pairs on the right (facing the stepper). ``VMOT`` is the motor supply
    (battery), ``VDD`` the 3.3 V logic rail. ``nENABLE`` is active-low with an
    external 10 kΩ pull-up. Coil pins are ordered to align with
    :func:`stepper_nema17` when placed at the same y-center.
    """
    return elm.Ic(
        pins=[
            # Left (bottom → top): power, enable, direction, step
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="VDD", side="L"),
            elm.IcPin(name="VMOT", side="L"),
            elm.IcPin(name="nENABLE", side="L"),
            elm.IcPin(name="DIR", side="L"),
            elm.IcPin(name="STEP", side="L"),
            # Right (bottom → top): bipolar coil outputs
            elm.IcPin(name="B2", side="R"),
            elm.IcPin(name="B1", side="R"),
            elm.IcPin(name="A2", side="R"),
            elm.IcPin(name="A1", side="R"),
        ],
        size=(4, 6),
    )


def stepper_nema17() -> elm.Ic:
    """4-wire bipolar stepper motor (NEMA17), two coils A and B.

    Coil pins on the left so they face the driver when drawn to its right.
    Pin order mirrors :func:`drv8825`'s right side so the four coil wires run
    straight across without crossing.
    """
    return elm.Ic(
        pins=[
            elm.IcPin(name="B2", side="L"),
            elm.IcPin(name="B1", side="L"),
            elm.IcPin(name="A2", side="L"),
            elm.IcPin(name="A1", side="L"),
        ],
        size=(2.5, 4),
    )
