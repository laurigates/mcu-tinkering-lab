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
    """MAX98357A mono I2S Class-D amplifier breakout (Adafruit #3006 pinout)."""
    return elm.Ic(
        pins=[
            # Left (bottom → top)
            elm.IcPin(name="GAIN", side="L"),
            elm.IcPin(name="SD", side="L"),
            elm.IcPin(name="GND", side="L"),
            elm.IcPin(name="VIN", side="L"),
            # Right (bottom → top) — matches ESP32-S3 GPIO7/6/5 ordering
            elm.IcPin(name="DIN", side="R"),
            elm.IcPin(name="LRC", side="R"),
            elm.IcPin(name="BCLK", side="R"),
            # Bottom — speaker outputs
            elm.IcPin(name="OUT-", side="B", pin="-"),
            elm.IcPin(name="OUT+", side="B", pin="+"),
        ],
        size=(3, 5),
    )
