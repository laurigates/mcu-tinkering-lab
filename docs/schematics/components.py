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

Add a new component by writing another factory.
"""

from __future__ import annotations

import schemdraw.elements as elm


def esp32_s3_zero() -> elm.Ic:
    """Waveshare ESP32-S3-Zero dev board (compact, castellated).

    Right side holds the I2S pins (GPIO5/6/7, top-to-bottom) to align with
    :func:`max98357a`. GPIO2 (status LED) is routed out the top.
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
        ],
        size=(3, 5),
    )


def max98357a() -> elm.Ic:
    """MAX98357A mono I2S Class-D amplifier breakout (Adafruit #3006 pinout).

    I2S pins are placed on the *left* side so they face the MCU when the amp
    is drawn to the right of it — wires don't have to cross the chip body.
    """
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
            # Bottom — speaker outputs. Slightly inset from the corners so
            # their labels don't collide with DIN / GAIN on the adjacent sides.
            elm.IcPin(name="OUT-", side="B", pin="-", pos=0.15),
            elm.IcPin(name="OUT+", side="B", pin="+", pos=0.85),
        ],
        size=(4, 5),
    )


def xiao_esp32s3_sense() -> elm.Ic:
    """Seeed XIAO ESP32-S3 Sense — ESP32-S3 + OV2640 camera + 8MB PSRAM.

    Camera/PSRAM pins are internal to the Sense module and don't conflict with
    the 11 GPIO header pins. Right side lists the GPIOs the robocar uses, in
    the order they connect to peripherals top-to-bottom (SDA/SCL at top to
    align with the I2C mux to the right).
    """
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
            elm.IcPin(name="GPIO6", side="R", pin="D5"),  # SCL
            elm.IcPin(name="GPIO5", side="R", pin="D4"),  # SDA
        ],
        size=(3.5, 7),
    )


def tca9548a() -> elm.Ic:
    """TCA9548A 8-channel I2C multiplexer (Adafruit / generic breakout).

    Only the channels used by robocar-unified are exposed (ch0 = PCA9685,
    ch1 = OLED). Upstream I2C + power on the left, downstream channels on
    the right.
    """
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
            elm.IcPin(name="SD1", side="R"),
            elm.IcPin(name="SC1", side="R"),
            elm.IcPin(name="SD0", side="R"),
            elm.IcPin(name="SC0", side="R"),
        ],
        size=(3.5, 5),
    )


def pca9685() -> elm.Ic:
    """PCA9685 16-channel 12-bit I2C PWM driver.

    16 PWM outputs are grouped on the right by destination — drawing 16
    individual pins makes the schematic unreadable. Pin labels record the
    actual channel ranges from ``pin_config.h``.
    """
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


def tb6612fng() -> elm.Ic:
    """TB6612FNG dual H-bridge motor driver (Sparkfun / generic breakout).

    Control signals on the left face the PCA9685; motor outputs on the right.
    PWMA/AIN1/AIN2 drive channel A (right motor), PWMB/BIN1/BIN2 drive B
    (left motor), STBY is a global enable from the MCU.
    """
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
