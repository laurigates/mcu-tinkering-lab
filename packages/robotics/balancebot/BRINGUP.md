# Balancebot Bring-Up: Before the First `arm`

The three adjustments and checks to make once the harness or carrier is
wired and before the firmware drives a motor: the buck output, the DRV8825
straps, and each driver's current limit. Wiring itself is in
[`WIRING.md`](WIRING.md); flashing, first boot and calibration follow in the
[build guide](docs/build-guide.pdf) §7–8.

Tools: a multimeter, and an insulated or ceramic trim-pot screwdriver.

## 1. Set the buck output before the XIAO sees it

LM2596 modules ship adjustable, and often set well above 5 V. With the XIAO
**unplugged**, power the buck from the pack and turn its pot until the output
reads:

- **5.0 V** on the hand-wired harness, which feeds the XIAO's 5V pad directly.
- **5.0–5.3 V** on the carrier PCB, where D1 drops about 0.3 V on the way to
  the 5V pad ([`hardware/carrier/README.md`](hardware/carrier/README.md)).

Only then connect the 5 V feed to the XIAO.

## 2. Check the straps

Power everything off, connect both motors (see the first trap in §3), then
power on from the pack. The firmware holds the drivers disabled until `arm`.
On each DRV8825, against GND:

| Pin | Expect | Why |
|---|---|---|
| M0, M1 | 3.3 V | 1/8 microstep strap |
| M2 | 0 V | 1/8 microstep strap (tied to GND) |
| nSLEEP, nRESET | 3.3 V | Driver awake |
| nENABLE | 3.3 V while disarmed | Drivers held off by the 10 kΩ pull-up (or by the firmware driving it high) |

A wrong M-pin reading means the wheels turn at a different speed than the
firmware expects for the same step rate.

## 3. Set the current limit (Vref), per driver

With the motors connected, VMOT and logic powered, and the drivers disabled:

1. Measure Vref between the trim pot's wiper (its metal top) and GND.
2. The DRV8825 limits each phase to **2 × Vref**. Start at **Vref ≈ 0.4 V**
   (0.8 A/phase).
3. Raise it in steps of 0.05–0.1 V only if push tests show skipped steps (a
   wheel that clicks or buzzes instead of turning). After each step, balance
   and push for a few minutes and feel the heatsinks: hot but holdable is fine.
4. Never exceed the motor's rated phase current: Vref ≤ rated current / 2
   (0.75 V for a 1.5 A motor). Pololu rates its DRV8825 carrier at about
   1.5 A/phase without a heatsink or airflow and up to 2.2 A with enough
   extra cooling; small stick-on heatsinks without a fan land in between.
5. Set both drivers to the same Vref, so one side does not start skipping
   steps before the other.

### Traps

- **Never connect or disconnect a motor while VMOT is powered.** The
  inductive spike can destroy the DRV8825. Plug the motors in before power-up,
  and power down before unplugging one.
- **A metal screwdriver that slips can short the pot to a neighbouring pin**
  and kill the driver. Use an insulated or ceramic one. Clipping the meter's
  positive lead to the screwdriver shaft lets you read Vref while turning.
- **Check the heatsink first.** On some carriers it covers or crowds the pot,
  or touches the header pins. Make sure the pot is reachable and the heatsink
  touches no pin.
- **Thermal shutdown drops the robot.** An overheated DRV8825 cuts its
  outputs, and the robot falls. A fall after several minutes of good balancing
  points here before it points at the tuning; lower Vref or improve cooling.
- **An enclosure runs hotter than the bench.** Repeat the heat check with the
  case on, and keep vents above the drivers.
