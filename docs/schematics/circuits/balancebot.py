"""Balancebot — self-balancing robot wiring schematic.

Seeed XIAO RP2350 driving two DRV8825 stepper carriers (one per wheel) and a
GY-521 MPU6050 IMU over I2C:
  - I2C1 (GPIO6/7) + INT (GPIO5) → MPU6050
  - GPIO2/GPIO4 → left DRV8825 STEP/DIR
  - GPIO3/GPIO0 → right DRV8825 STEP/DIR
  - GPIO1 → shared nENABLE (active-low, external 10 kΩ pull-up to 3V3)

Power: 2–3S LiPo → both DRV8825 VMOT (+100 µF each) and a 5 V buck → XIAO 5V.
3V3 feeds MPU6050 VCC, both DRV8825 VDD, and the nENABLE pull-up.

Source of truth: packages/robotics/balancebot/src/pin_config.h and WIRING.md
"""

import schemdraw
import schemdraw.elements as elm

from components import (
    drv8825,
    mpu6050,
    stepper_nema17,
    xiao_rp2350,
)


def draw() -> schemdraw.Drawing:
    d = schemdraw.Drawing(show=False)
    d.config(unit=2.0, fontsize=11)

    # === MCU on the left; peripherals stacked on the right. ===
    xiao = d.add(xiao_rp2350().label("XIAO RP2350", loc="bot", ofst=0.4))

    # MPU6050 up-and-right so its I2C+INT pins face the XIAO's top GPIOs. The
    # peripherals sit well to the right so the middle gap has room for the
    # shared-nENABLE trunk, its pull-up, and the driver power tags.
    mpu = d.add(
        mpu6050()
        .at((xiao.center.x + 13, xiao.center.y + 6))
        .anchor("center")
        .label("GY-521 MPU6050\n0x68", loc="bot", ofst=0.4)
    )

    # Two DRV8825 carriers below the IMU, one per wheel.
    drv_l = d.add(
        drv8825()
        .at((xiao.center.x + 13, xiao.center.y - 1))
        .anchor("center")
        .label("DRV8825 (left)\n1/8 µstep", loc="bot", ofst=0.4)
    )
    drv_r = d.add(
        drv8825()
        .at((xiao.center.x + 13, xiao.center.y - 10))
        .anchor("center")
        .label("DRV8825 (right)\n1/8 µstep", loc="bot", ofst=0.4)
    )

    # Steppers to the right of each driver, coil pins aligned for straight runs.
    step_l = d.add(
        stepper_nema17()
        .right()
        .at((drv_l.center.x + 6, drv_l.center.y))
        .anchor("center")
        .label("Left NEMA17", loc="bot", ofst=0.4)
    )
    step_r = d.add(
        stepper_nema17()
        .right()
        .at((drv_r.center.x + 6, drv_r.center.y))
        .anchor("center")
        .label("Right NEMA17", loc="bot", ofst=0.4)
    )

    # === I2C + INT: XIAO top GPIOs → MPU6050 (parallel, no crossing). ===
    d.add(elm.Wire("-|").at(xiao.GPIO7).to(mpu.SCL).color("steelblue"))
    d.add(elm.Wire("-|").at(xiao.GPIO6).to(mpu.SDA).color("steelblue"))
    d.add(elm.Wire("-|").at(xiao.GPIO5).to(mpu.INT).color("steelblue"))

    # === STEP / DIR to each driver. ===
    d.add(elm.Wire("-|").at(xiao.GPIO2).to(drv_l.STEP).color("steelblue"))
    d.add(elm.Wire("-|").at(xiao.GPIO4).to(drv_l.DIR).color("steelblue"))
    d.add(elm.Wire("-|").at(xiao.GPIO3).to(drv_r.STEP).color("steelblue"))
    d.add(elm.Wire("-|").at(xiao.GPIO0).to(drv_r.DIR).color("steelblue"))

    # === Shared nENABLE trunk: GPIO1 → vertical bus → both nENABLE pins,
    # with the 10 kΩ pull-up to 3V3 tapping the top of the bus. ===
    nen_x = xiao.center.x + 4.0
    top = (nen_x, drv_l.nENABLE.y)
    bot = (nen_x, drv_r.nENABLE.y)
    d.add(elm.Wire("-").at(top).to(bot).color("steelblue"))  # vertical bus
    d.add(elm.Wire("-").at(top).to(drv_l.nENABLE).color("steelblue"))
    d.add(elm.Wire("-").at(bot).to(drv_r.nENABLE).color("steelblue"))
    d.add(elm.Wire("-|").at(xiao.GPIO1).to((nen_x, xiao.GPIO1.y)).color("steelblue"))
    # Pull-up above the bus — lifted clear of the I2C wires before the +3V3 tag.
    d.add(elm.Line().up(1.5).at(top))
    d.add(elm.Resistor().up().label("10 kΩ"))
    d.add(elm.Vdd().label("+3V3"))

    # === Coil pairs: each DRV8825 right side → its stepper (straight runs). ===
    for drv, step in ((drv_l, step_l), (drv_r, step_r)):
        d.add(elm.Wire("-").at(drv.A1).to(step.A1))
        d.add(elm.Wire("-").at(drv.A2).to(step.A2))
        d.add(elm.Wire("-").at(drv.B1).to(step.B1))
        d.add(elm.Wire("-").at(drv.B2).to(step.B2))

    # === Power rails. ===
    # XIAO: 3V3 logic, 5V from the buck converter, common ground — left side.
    d.add(elm.Line().left(0.5).at(xiao["3V3"]))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(0.5).at(xiao["5V"]))
    d.add(elm.Vdd().label("+5V buck"))
    d.add(elm.Line().left(0.5).at(xiao.GND))
    d.add(elm.Ground())

    # MPU6050: 3V3 + ground on its left, below the signal wires.
    d.add(elm.Line().left(1.0).at(mpu.VCC))
    d.add(elm.Vdd().label("+3V3"))
    d.add(elm.Line().left(1.0).at(mpu.GND))
    d.add(elm.Ground())

    # DRV8825 carriers: VDD = 3V3 logic, VMOT = battery motor supply
    # (+100 µF electrolytic close to the pins), GND common.
    for drv in (drv_l, drv_r):
        d.add(elm.Line().left(1.0).at(drv.VDD))
        d.add(elm.Vdd().label("+3V3"))
        d.add(elm.Line().left(1.0).at(drv.VMOT))
        d.add(elm.Vdd().label("+VMOT"))
        d.add(elm.Line().left(1.0).at(drv.GND))
        d.add(elm.Ground())

    return d


if __name__ == "__main__":
    import sys
    from pathlib import Path

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("balancebot.svg")
    draw().save(str(out))
    print(f"wrote {out}")
