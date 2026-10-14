"""Balancebot — self-balancing robot wiring schematic.

Seeed XIAO RP2350 driving two DRV8825 stepper carriers (one per wheel) and a
GY-521 MPU6050 IMU over I2C, with the drivers' shared nENABLE held high by a
10 kΩ pull-up so they stay disabled while the XIAO boots.

Power: a 3S pack feeds both DRV8825 VMOT pins and a 5 V buck into the XIAO's
5V pad. The XIAO's 3V3 feeds the MPU6050, the pull-up, and each carrier's
nSLEEP/nRESET and M0/M1 straps; M2 goes to ground for 1/8 microstep. The
Pololu-layout carrier has no logic-supply pin (#738).

The XIAO is drawn physically from docs/reference/boards/xiao-rp2350.md, and
every MCU wire, pin label, power-tag rail and coil wire is read from the
hardware join (#738, ADR-021) at render time: packages/robotics/balancebot/
hardware.toml says which part pin each pin role is wired to, src/pin_config.h
which GPIO the role is, and the board reference which header pad that GPIO
is. Placement, net order, net colour and the nENABLE trunk's shape stay
hand-authored here.
"""

import sys
from pathlib import Path

import schemdraw
import schemdraw.elements as elm
from components import (
    drv8825,
    mpu6050,
    stepper_nema17,
    xiao_rp2350,
)
from hardware_nets import JoinedNets
from routing import Router, net_color

_REPO = Path(__file__).resolve().parents[3]
if str(_REPO / "tools") not in sys.path:
    sys.path.insert(0, str(_REPO / "tools"))
from hardware import HardwareModel, join  # noqa: E402

PROJECT = _REPO / "packages/robotics/balancebot"

# Carrier pins strapped to a rail, left to right along the symbol's top edge.
# Which rail each is on comes from hardware.toml; M2's ground does not, since
# the harness keeps its common ground in WIRING.md's prose.
STRAPS = ("nSLEEP", "nRESET", "M0", "M1")


def _tag(
    d, pin, direction: str, length: float, kind: str, label: str = "", loc: str = "top"
) -> None:
    """A power or ground tag on ``pin``, leading ``direction`` for ``length``."""
    line = getattr(elm.Line(), direction)(length).at(pin)
    d.add(line.color(net_color("power" if kind == "power" else "ground")))
    if kind == "power":
        d.add(elm.Vdd().label(label, loc=loc).color(net_color("power")))
    else:
        d.add(elm.Ground().color(net_color("ground")))


def _straps(d, nets: JoinedNets, part: str, drv: elm.Element) -> None:
    """Tie each run of same-rail strap pins to one bar and one tag, above."""
    reach = 0.5
    runs: list[tuple[str, list[str]]] = []
    for pin in STRAPS:
        rail = nets.rail(part, pin)
        if runs and runs[-1][0] == rail:
            runs[-1][1].append(pin)
        else:
            runs.append((rail, [pin]))
    color = net_color("power")
    for rail, pins in runs:
        ends = [drv.absanchors[p] for p in pins]
        y = ends[0][1] + reach
        for x, py in ends:
            d.add(elm.Line().at((x, py)).to((x, y)).color(color))
        if len(ends) > 1:
            d.add(elm.Line().at((ends[0][0], y)).to((ends[-1][0], y)).color(color))
        d.add(elm.Vdd().up().at((ends[-1][0], y)).label(rail, loc="right").color(color))


def _fanout_trunk(d, mcu, ends: dict, x: float, color: str) -> None:
    """A vertical trunk at ``x`` joining the MCU pad to every part pin of a net.

    Split at every tap, so each T is three wire ends at one point and the
    router dots it.
    """
    taps = [*ends.values(), mcu]
    ys = sorted({round(y, 6) for _, y in taps})
    for lo, hi in zip(ys, ys[1:]):
        d.add(elm.Line().at((x, lo)).to((x, hi)).color(color))
    for px, py in taps:
        if abs(px - x) > 1e-6:
            d.add(elm.Line().at((px, py)).to((x, py)).color(color))


def draw(model: HardwareModel | None = None) -> schemdraw.Drawing:
    """Draw the schematic from ``model``, by default balancebot's live join.

    Taking the model as an argument lets a test hand in a join with one net
    added, and see the render refuse it.
    """
    if model is None:
        model = join(PROJECT)

    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=11)

    # === Components first, so every net below routes with full obstacle
    # awareness (the auto-router only avoids components already placed). ===
    #
    # The XIAO's I2C and INT pads are on its left edge (D3-D5) and its power,
    # STEP/DIR and nENABLE pads on the right, so the IMU sits to the left and
    # the drivers stack to the right.
    xiao = d.add(
        xiao_rp2350(layout="physical")
        .right()
        .label("XIAO RP2350\n(USB-C at top)", loc="bot", ofst=0.4)
    )
    parts: dict[str, elm.Element] = {}
    nets = JoinedNets(model, xiao, parts)

    parts["imu"] = imu = d.add(
        mpu6050(facing="R")
        .at((xiao.center.x - 9, nets.pad("PIN_IMU_SDA")[1]))
        .anchor("center")
        .label("GY-521 MPU6050\n0x68", loc="bot", ofst=0.4)
    )

    # The left driver on top, beside the XIAO's D9/D8 STEP/DIR pads; the
    # right driver below, so DIR_RIGHT (D6, on the XIAO's left edge) can pass
    # under the module instead of over the I2C wires.
    drv_x = xiao.center.x + 13
    parts["drv_left"] = drv_l = d.add(
        drv8825()
        .at((drv_x, xiao.center.y - 0.6))
        .anchor("center")
        .label("DRV8825 (left)\n1/8 µstep", loc="bot", ofst=1.4)
    )
    parts["drv_right"] = drv_r = d.add(
        drv8825()
        .at((drv_x, xiao.center.y - 11))
        .anchor("center")
        .label("DRV8825 (right)\n1/8 µstep", loc="bot", ofst=1.4)
    )

    # Steppers to the right of each driver, coil pins aligned for straight runs.
    for side, drv in (("left", drv_l), ("right", drv_r)):
        parts[f"stepper_{side}"] = d.add(
            stepper_nema17()
            .right()
            .at((drv.center.x + 7.5, drv.center.y))
            .anchor("center")
            .label(f"{side.capitalize()} NEMA17", loc="bot", ofst=0.4)
        )

    # The shared nENABLE trunk runs down the gap between the XIAO and the
    # drivers. The pull-up stands beside it under the XIAO, clear of every
    # STEP/DIR row; its B end is the net, its A end the rail.
    nen_x = xiao.center.x + 5.0
    pullup = d.add(
        elm.Resistor()
        .up()
        .at((nen_x - 2.0, xiao.center.y - 7.5))
        .label("10 kΩ", loc="bot")
    )
    pullup.absanchors["B"] = pullup.absanchors["start"]
    pullup.absanchors["A"] = pullup.absanchors["end"]
    parts["pullup"] = pullup

    # === Local stubs (power tags, nENABLE trunk) before routing, so their
    # footprint — e.g. a GND tag protruding from a chip's side — is a known
    # obstacle to every net below. Placed after routing, the router can't
    # see them and a wire can end up drawn right through one. ===

    # Shared nENABLE: one role, three part pins. A multi-endpoint bus (a
    # T-junction fan-out), so it stays hand-drawn — the router only handles
    # two-terminal wires — but every end of it comes from the join.
    _fanout_trunk(d, *nets.fanout("PIN_MOTOR_NENABLE"), nen_x, net_color("signal"))
    d.add(
        elm.Vdd()
        .up()
        .at(pullup.end)
        .label(nets.rail("pullup", "A"), loc="right")
        .color(net_color("power"))
    )

    # XIAO 5V / GND / 3V3 sit together at the top of its right edge.
    _tag(d, xiao["5V"], "right", 0.75, "power", nets.rail("mcu", "5V"))
    _tag(d, xiao["GND"], "right", 1.5, "ground")
    _tag(d, xiao["3V3"], "right", 0.75, "power", nets.rail("mcu", "3V3"))

    # MPU6050 power on its outward (left) side.
    _tag(d, imu.VCC, "left", 1.0, "power", nets.rail("imu", "VCC"))
    _tag(d, imu.GND, "left", 1.0, "ground")

    # DRV8825 carriers: VMOT up to the pack rail, the nSLEEP/nRESET and
    # microstep straps beside it, M2 and GND down. The 100 µF bulk capacitor
    # across VMOT/GND is in WIRING.md, not here.
    for part, drv in (("drv_left", drv_l), ("drv_right", drv_r)):
        _tag(d, drv.VMOT, "up", 0.5, "power", nets.rail(part, "VMOT"), loc="left")
        _straps(d, nets, part, drv)
        _tag(d, drv.M2, "down", 0.5, "ground")
        _tag(d, drv.GND, "down", 0.5, "ground")

    # === Nets: auto-routed orthogonal, obstacle-avoiding wires. ===
    router = Router(d)

    # I2C + INT: XIAO left-edge pads → MPU6050.
    router.wire(*nets.ends("PIN_IMU_SCL"), net="i2c")
    router.wire(*nets.ends("PIN_IMU_SDA"), net="i2c")
    router.wire(*nets.ends("PIN_IMU_INT"), net="sensor")

    # STEP / DIR to each driver.
    router.wire(*nets.ends("PIN_STEP_LEFT"), net="signal")
    router.wire(*nets.ends("PIN_DIR_LEFT"), net="signal")
    router.wire(*nets.ends("PIN_STEP_RIGHT"), net="signal")
    router.wire(*nets.ends("PIN_DIR_RIGHT"), net="signal")

    # Coil pairs: each DRV8825 right side → its stepper, from [[outputs]].
    for start, end in nets.output_ends():
        router.wire(start, end, net="load")

    # Every MCU net in hardware.toml has now been drawn, or this fails the
    # render naming the one that was not.
    nets.check_all_drawn()
    # Likewise every drawn part pin hardware.toml puts on a rail has a tag.
    nets.check_all_tagged()

    # Every net is routed; draw them all here. The nENABLE trunk and power
    # stubs are already in the drawing, so finish() hops and dots them too.
    router.finish()

    return d


if __name__ == "__main__":
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("balancebot.svg")
    draw().save(str(out))
    print(f"wrote {out}")
