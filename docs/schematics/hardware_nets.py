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
"""

from __future__ import annotations

import sys
from pathlib import Path

import schemdraw.elements as elm

_TOOLS = Path(__file__).resolve().parents[2] / "tools"
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from components import gpio_anchor  # noqa: E402
from hardware import HardwareError, HardwareModel, Net  # noqa: E402

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

    def _net(self, role: str) -> Net:
        nets = [n for n in self.model.nets if n.role == role]
        if not nets:
            raise HardwareError(f"{role}: no [[nets]] entry in hardware.toml")
        if len(nets) > 1:
            raise HardwareError(
                f"{role}: fans out to {[f'{n.part}.{n.pin}' for n in nets]}; "
                "JoinedNets draws one wire per role"
            )
        if role in self._drawn:
            raise HardwareError(f"{role}: drawn twice")
        return nets[0]

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
        element = self.parts.get(net.part)
        if element is None:
            raise HardwareError(
                f"{role}: hardware.toml wires it to part {net.part!r}, "
                "which the drawing did not pass in"
            )
        if net.pin not in element.absanchors:
            raise HardwareError(
                f"{role}: hardware.toml wires it to {net.part}.{net.pin}, "
                f"and the drawn {self.model.parts[net.part].name} has no such pin"
            )
        mcu = self.pad(role)
        self._drawn.add(role)
        return mcu, element.absanchors[net.pin]

    def start(self, role: str) -> Point:
        """The MCU end of a net whose far end the circuit draws by hand."""
        self._net(role)
        mcu = self.pad(role)
        self._drawn.add(role)
        return mcu

    def check_all_drawn(self) -> None:
        """Fail if the join declares a net the drawing never consumed."""
        missing = [n for n in self.model.nets if n.role not in self._drawn]
        if missing:
            listed = ", ".join(f"{n.role} -> {n.part}.{n.pin}" for n in missing)
            raise HardwareError(
                f"hardware.toml declares net(s) the schematic does not draw: {listed}"
            )
