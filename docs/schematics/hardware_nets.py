"""A circuit's MCU nets, read from the project's hardware join (#462, ADR-021).

``<project>/hardware.toml`` says which part pin each firmware pin role is
wired to (``I2S_BCLK_PIN -> amp.BCLK``); ``main/pin_config.h`` says which GPIO
the role is; the board reference says which header pad that GPIO is. A
circuit that transcribes the result — ``router.wire(xiao.GPIO7, amp.BCLK)`` —
goes quietly wrong when any of the three moves. One that asks for
``nets.ends("I2S_BCLK_PIN")`` follows them.

Only the endpoints come from here. Which nets are drawn in what order, with
what class colour and through which corridor, stays in the circuit: that is a
drawing someone wires from, and ADR-021 § What stays hand-authored keeps it
human.

Each net is consumed once. ``check_all_drawn()`` then fails the render for any
net the join declares and the drawing never drew, so a ``[[nets]]`` entry
added to hardware.toml cannot ship a schematic that lacks it.

Power tags work the same way (#679): ``rail()`` gives the label a tag on a
part pin carries, from ``[[rails]]``, and ``check_all_tagged()`` fails the
render for any rail endpoint on a drawn part that no tag was asked about.

A role wired to several part pins (balancebot's shared nENABLE) comes back
whole from ``fanout()``, and a driver's ``[[outputs]]`` pin lists as wire
pairs from ``output_ends()`` (#738), each driver pin on the load pin its
``load_pins`` names (#769). An I2C mux channel's ``[[bus_nets]]`` lines
come from ``bus_ends()`` (#768). ``check_all_drawn()`` covers both.
"""

from __future__ import annotations

import sys
from pathlib import Path

import schemdraw.elements as elm

_TOOLS = Path(__file__).resolve().parents[2] / "tools"
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from components import gpio_anchor  # noqa: E402
from hardware import MCU, Endpoint, HardwareError, HardwareModel, Net  # noqa: E402

Point = tuple[float, float]


class JoinedNets:
    """The MCU nets of ``model``, resolved against the drawn elements.

    ``mcu`` is the placed physical symbol of the project's MCU board, whose
    pads are anchored by :func:`components.gpio_anchor`. ``parts`` maps each
    hardware.toml part id to its placed element; a part whose nets are all
    hand-drawn (reached through :meth:`start`) need not be in it.
    """

    def __init__(
        self,
        model: HardwareModel,
        mcu: elm.Element,
        parts: dict[str, elm.Element],
    ) -> None:
        self.model = model
        self.mcu = mcu
        self.parts = parts
        self._drawn: set[str] = set()
        self._drawn_outputs: set[str] = set()  # driver part ids
        self._tagged: set[Endpoint] = set()

    def _nets(self, role: str) -> list[Net]:
        nets = [n for n in self.model.nets if n.role == role]
        if not nets:
            raise HardwareError(f"{role}: no [[nets]] entry in hardware.toml")
        if role in self._drawn:
            raise HardwareError(f"{role}: drawn twice")
        return nets

    def _net(self, role: str) -> Net:
        nets = self._nets(role)
        if len(nets) > 1:
            raise HardwareError(
                f"{role}: fans out to {[f'{n.part}.{n.pin}' for n in nets]}; "
                "draw it with fanout()"
            )
        return nets[0]

    def _pin(self, element_of: str, part: str, pin: str) -> Point:
        """Where drawn part ``part``'s pin ``pin`` is; ``element_of`` prefixes errors."""
        element = self.parts.get(part)
        if element is None:
            raise HardwareError(
                f"{element_of}: hardware.toml wires it to part {part!r}, "
                "which the drawing did not pass in"
            )
        if pin not in element.absanchors:
            raise HardwareError(
                f"{element_of}: hardware.toml wires it to {part}.{pin}, "
                f"and the drawn {self.model.parts[part].name} has no such pin"
            )
        return element.absanchors[pin]

    def pad(self, role: str) -> Point:
        """The MCU header pad ``role``'s GPIO lands on."""
        gpio = self.model.roles.get(role)
        if gpio is None:
            raise HardwareError(
                f"{role}: not a pin role in {self.model.headers[0].name}"
            )
        if self.model.pin_for(role) is None:
            raise HardwareError(
                f"{role}: GPIO{gpio} is not on a header pad of "
                f"{self.model.board.path.name}, so no wire can reach it"
            )
        return self.mcu.absanchors[gpio_anchor(gpio)]

    def ends(self, role: str) -> tuple[Point, Point]:
        """Both ends of ``role``'s net: the MCU pad and the part pin it is wired to."""
        net = self._net(role)
        part = self._pin(role, net.part, net.pin)
        mcu = self.pad(role)
        self._drawn.add(role)
        return mcu, part

    def fanout(self, role: str) -> tuple[Point, dict[str, Point]]:
        """The MCU pad of ``role`` and every part pin it is wired to.

        The part pins are keyed ``"part.pin"``. Which trunk joins them is the
        circuit's drawing; that they are all on the net is the join's.
        """
        ends = {
            f"{n.part}.{n.pin}": self._pin(role, n.part, n.pin)
            for n in self._nets(role)
        }
        mcu = self.pad(role)
        self._drawn.add(role)
        return mcu, ends

    def output_ends(self, part: str | None = None) -> list[tuple[Point, Point]]:
        """One (driver pin, load pin) pair per ``[[outputs]]`` wire.

        A driver pin lands on the load pin ``load_pins`` maps it to, or on the
        pin of the same name (#769). With no ``part``, every driver's at once;
        with one, that driver's alone, and ``check_all_drawn()`` then fails for
        a driver whose wires were never asked for. Either way a pin added to
        an output list cannot be left out of the drawing.
        """
        outs = [o for o in self.model.outputs if o.pins and part in (None, o.part)]
        if part is not None and not outs:
            raise HardwareError(f"{part!r}: no [[outputs]] pins in hardware.toml")
        # One driver may have several [[outputs]] entries; drawing it once
        # covers them all, so the guard is per driver per call, not per entry.
        drivers = dict.fromkeys(o.part for o in outs)
        twice = [d for d in drivers if d in self._drawn_outputs]
        if twice:
            raise HardwareError(f"{', '.join(twice)} outputs: drawn twice")
        self._drawn_outputs.update(drivers)
        pairs = []
        for out in outs:
            pairs += [
                (
                    self._pin(f"{out.part} output {pin}", out.part, pin),
                    self._pin(f"{out.part} output {pin}", load, load_pin),
                )
                for pin, load, load_pin in out.wires
            ]
        return pairs

    def bus_ends(
        self, role: str, anchors: dict[str, str] | None = None
    ) -> list[tuple[Point, Point]]:
        """One (mux pad, load pin) pair per line of ``role``'s ``[[bus_nets]]``.

        SDA first, then SCL, for each load in file order. ``anchors`` maps a
        load pin (``"pwm.SDA"``) to the drawn element's anchor where the board
        repeats the pad (``"SDA.R4"``); the anchor must name the same pin.
        """
        buses = [b for b in self.model.bus_nets if b.role == role]
        if not buses:
            raise HardwareError(f"{role}: no [[bus_nets]] entry in hardware.toml")
        if role in self._drawn:
            raise HardwareError(f"{role}: drawn twice")
        anchors = anchors or {}
        on_bus = {f"{b.part}.{pin}" for b in buses for _, pin in b.wires}
        stray = sorted(set(anchors) - on_bus)
        if stray:
            raise HardwareError(f"{role}: anchors for pins not on this bus: {stray}")
        pairs = []
        for bus in buses:
            for pad, pin in bus.wires:
                anchor = anchors.get(f"{bus.part}.{pin}", pin)
                if anchor.split(".")[0] != pin:
                    raise HardwareError(
                        f"{role}: anchor {anchor!r} for {bus.part}.{pin} is another pin"
                    )
                pairs.append(
                    (
                        self._pin(role, bus.source, pad),
                        self._pin(role, bus.part, anchor),
                    )
                )
        self._drawn.add(role)
        return pairs

    def start(self, role: str) -> Point:
        """The MCU end of a net whose far end the circuit draws by hand."""
        self._net(role)
        mcu = self.pad(role)
        self._drawn.add(role)
        return mcu

    def check_all_drawn(self) -> None:
        """Fail if the join declares a net the drawing never consumed."""
        missing = [
            f"{n.role} -> {n.part}.{n.pin}"
            for n in self.model.nets
            if n.role not in self._drawn
        ] + [
            f"{b.role} -> {b.part}"
            for b in self.model.bus_nets
            if b.role not in self._drawn
        ]
        missing += [
            f"{o.part} outputs -> {', '.join(o.loads)}"
            for o in self.model.outputs
            if o.pins and o.part not in self._drawn_outputs
        ]
        if missing:
            listed = ", ".join(missing)
            raise HardwareError(
                f"hardware.toml declares net(s) the schematic does not draw: {listed}"
            )

    def rail(self, part: str, pin: str) -> str:
        """The label of the rail ``part``.``pin`` is on, e.g. ``"+5V"``.

        ``part`` is a hardware.toml part id (``"mcu"`` for the board) and
        ``pin`` the model's pin name, not the drawn element's anchor name.
        """
        end = Endpoint(part, pin)
        for rail in self.model.rails:
            if end == rail.source or end in rail.loads:
                self._tagged.add(end)
                return "+" + rail.name
        raise HardwareError(f"{part}.{pin}: on no [[rails]] entry in hardware.toml")

    def check_all_tagged(self) -> None:
        """Fail if a rail endpoint on a drawn part was never given a tag."""
        drawn = {MCU, *self.parts}
        missing = [
            f"{e.part}.{e.pin} ({r.name})"
            for r in self.model.rails
            for e in (r.source, *r.loads)
            if e.part in drawn and e not in self._tagged
        ]
        if missing:
            raise HardwareError(
                "hardware.toml puts drawn part pin(s) on a rail the schematic "
                f"does not tag: {', '.join(missing)}"
            )
