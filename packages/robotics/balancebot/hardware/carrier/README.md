# Balancebot carrier PCB

A 90 × 62 mm, 2-layer, all-through-hole carrier that replaces the hand-wired
harness in [`WIRING.md`](../../WIRING.md). Every module sits in a socket or
plugs into a connector, so any of them can be swapped without desoldering.

KiCad 10 project: [`balancebot-carrier/`](balancebot-carrier/). The schematic is
the source of truth for parts and nets; the board is generated from it by the
scripts in [`scripts/`](scripts/).

## What is on it

| Ref | Part | Connects |
|---|---|---|
| U1 | Seeed XIAO RP2350, in two 1×7 female headers | USB-C toward the top edge |
| A1, A2 | Pololu-format DRV8825 carriers, in 1×8 female headers | Left / right stepper |
| J3 | GY-521 (MPU6050), in a 1×8 female header | Lies flat over the outlined area |
| J1 | AMASS XT30PW-M | 3S pack (through its BMS) |
| J2 | JST-XH 4-pin | LM2596 module: IN+, IN−, OUT+, OUT− |
| J4, J5 | JST-XH 4-pin | Motor coils A1, A2, B1, B2 |
| C1, C2 | 100 µF 25 V electrolytic, 8 mm | VMOT bulk at each driver |
| R1 | 10 kΩ | nENABLE pull-up to 3V3 (motors off until firmware drives it low) |
| D1 | 1N5819 | LM2596 output → XIAO 5V pad, so USB cannot back-feed the buck |
| R2, R3, C3 | 100 kΩ, 27 kΩ, 100 nF | Pack voltage divider into D0 (GPIO26, ADC0) |
| H1–H4 | M3 | Mounting holes on a 82 × 54 mm rectangle |

The DRV8825 straps are on the board: M0 and M1 to 3V3 and M2 to GND (1/8
microstep), nSLEEP and nRESET to 3V3, FAULT left open. The Pololu carrier has
no VDD pin. Pin 2 is FAULT, which settles the hardware half of #738.

The divider reads 12.6 V (full 3S) as 2.68 V and draws about 0.1 mA from the
pack. Firmware does not read it yet.

## Check before ordering

These come from library footprints and from memory, not from your parts. Hold
each part against the board render (`just balancebot::carrier-fab`, then open
the Gerbers, or open the board in KiCad):

1. **XT30 polarity.** KiCad's footprint puts "−" on pad 1 and "+" on pad 2; the
   board's silkscreen follows it. Check that the + and − moulded into your XT30
   body match the silkscreen when the connector sits in its outline.
2. **DRV8825 pin order.** The footprint is the Pololu layout: one row GND, FAULT,
   A2, A1, B1, B2, GND, VMOT; the other EN, M0, M1, M2, RST, SLP, STEP, DIR.
   Unbranded carriers usually copy it, but check yours.
3. **GY-521 pin order.** Assumed VCC, GND, SCL, SDA, XDA, XCL, AD0, INT with
   pin 1 at the left end of J3. This one is from memory; read it off your
   module's silkscreen before ordering.

## Assembly notes

- Set the LM2596 to 5.0–5.3 V on the bench **before** plugging it into J2. D1
  drops about 0.3 V, so the XIAO's 5V pad sees roughly 4.7–5.0 V.
- Mount the GY-521 flat with its X arrow toward the board's FWD mark; the
  firmware computes pitch from X and Z. The carrier must sit at axle height for
  the IMU placement the build guide describes.
- Set each DRV8825's Vref before connecting motors, as in the build guide.

## Regenerate and check

```sh
just balancebot::carrier-lib     # socket XIAO footprint + symbol from Seeed's DIP ones
just balancebot::carrier-board   # place, autoroute (Freerouting), pour GND
just balancebot::carrier-check   # ERC, DRC + schematic parity, pin_config.h cross-check
just balancebot::carrier-fab     # Gerbers + drill files → output/balancebot-carrier-fab.zip
```

`carrier-board` needs KiCad 10 (its bundled Python provides `pcbnew`) and the
Freerouting 2.5.0 jar at `~/.cache/freerouting/` (or `FREEROUTING_JAR`).
Freerouting does not route the same way twice, so a re-run gives a different,
equally checked board: commit the board you checked.

`carrier-check` fails if a role in `src/pin_config.h` lands on a different
carrier net than the one this board gives it. It reads the pad for each role
through the hardware join (ADR-021), so renumbering a pin without rewiring the
board fails here.

### Why the board is built by script

- **Placement** is the table in `scripts/build_board.py`; parts and nets come
  from the schematic's netlist.
- **The XIAO footprint** is derived from Seeed's `XIAO-RP2350-DIP`
  ([OSHW-XIAO-Series @ cc7f3bb](https://github.com/Seeed-Studio/OSHW-XIAO-Series),
  MIT), which is kept unmodified in `lib/`. The DIP version mills a cutout under
  the module and plates holes for its bottom pads; a socketed XIAO needs
  neither.
- **`balancebot-carrier.kicad_dru`** lets a GND thermal relief resolve to one
  spoke. `route_board.py` refuses to pour unless every GND pad also has a routed
  track, so the spoke is never the only connection.

## Ordering

JLCPCB standard 2-layer, 1.6 mm FR-4, 1 oz copper. Upload
`output/balancebot-carrier-fab.zip`. Minimums on this board: 0.3 mm track,
0.6/0.3 mm via, 0.2 mm annular ring, 1.0 mm silkscreen text.
