# SSD1306 — 128x64 OLED/PLED Controller

**Manufacturer:** Solomon Systech
**Used in:** Heltec WiFi LoRa 32 V1 (integrated 128x64 OLED); robocar-unified
(0.91-inch 128x32 module on TCA9548A channel 1 —
[board page](../boards/ssd1306-091in-128x32.md))

Sources, cited below by short name:

- **DS** — SSD1306 datasheet, Rev 1.1, Apr 2008; page numbers are the
  document's own `P n/59`.
- **AN** — SSD1306 Application Note (charge pump), Rev 0.4, Jan 2009; appended
  to the same Adafruit PDF as its pages 60–65, cited by the note's own `P n/6`.
- **UG** — Univision UG-2832HSWEG02 product specification, Doc. SAS1-B026-B
  rev B: a 0.91-inch 128x32 SSD1306 panel; page numbers are the document's own
  (PDF page minus 5). The robocar-unified listing does not name its glass, so
  UG is the reference panel for that size, not a confirmed part.

## Key Specs

| Parameter | Value | Source |
|-----------|-------|--------|
| Resolution | 128 x 64 dot matrix | DS §2, p. 6 |
| Type | Common cathode OLED/PLED | DS §1, p. 6 |
| Interfaces | I2C, SPI, 6800/8080 parallel, pin selectable | DS §1–2, p. 6 |
| I2C address | `0111100` (`0x3C`, SA0 = 0) or `0111101` (`0x3D`, SA0 = 1); the D/C# pin acts as SA0 | DS §8.1.5 a, p. 19 |
| Logic supply (VDD) | 1.65–3.3 V; absolute maximum −0.3 to +4 V | DS Table 12-1, p. 48; Table 11-1, p. 47 |
| Panel supply (VCC), external | 7–15 V | DS Table 12-1, p. 48 |
| Charge pump | Generates 7.5 V VCC from VBAT = 3.3–4.2 V with two external capacitors; disabled at reset | AN §2, p. 3; §2.1, p. 3 |
| Logic input levels | VIH ≥ 0.8 x VDD, VIL ≤ 0.2 x VDD | DS Table 12-1, p. 48 |
| I2C clock | Clock cycle ≥ 2.5 µs (400 kHz); rise and fall ≤ 300 ns | DS Table 13-6, p. 54 |
| Brightness | 256-step contrast (`81h`), reset `7Fh` | DS §2, p. 6; Table 9-1, p. 28 |
| RAM | 128 x 64 bit GDDRAM, eight pages PAGE0–PAGE7 | DS §8.7, p. 25 |
| Operating temperature | −40 to 85 °C | DS §2, p. 6 |

The Heltec WiFi LoRa 32 V1 answers at `0x3C`
(`packages/robocar/main/main/pin_config_idf.h`, `OLED_I2C_ADDR`) — a repo fact,
not a datasheet one.

## Panel variants

The controller always has 64 COM outputs. A panel with fewer rows is driven by
lowering the multiplex ratio and matching the COM pin configuration to how the
panel's rows are bonded to the die, so both values belong to the panel, not the
controller.

| Setting | 128x64 (Heltec) | 0.91-inch 128x32 module | Source |
|---------|-----------------|-------------------------|--------|
| Rows, drive duty | 64 | 32, 1/32 duty | DS §2, p. 6; UG §1.1, p. 1 |
| Multiplex ratio `A8h` | `3Fh` (64MUX, the reset value) | `1Fh` (32MUX) | DS Table 9-1, p. 31; UG §4.4, p. 10 |
| COM pins `DAh` | `12h`, alternative (the reset value; what this repo's 128x64 code sends) | `02h`, sequential, no left/right remap | DS Table 9-1, p. 31; UG §4.4, p. 10 |
| GDDRAM pages shown | 0–7 | 0–3 (8 rows per page, 32 rows) | DS §8.7, p. 25 |
| Contrast `81h` | — | `8Fh` in the panel maker's example | UG §4.4, p. 10 |

The Heltec's panel specification is not on file, so its column states the
controller reset values and what the code sends, not a panel maker's
recommendation. `A8h` values 0–14 are invalid (DS Table 9-1, p. 31).

**What a 128x64 initialisation does to a 128x32 panel.** `DAh 12h` selects the
alternative layout, in which COM0–COM31 carry RAM rows 0, 2, 4 … 62 and
COM32–COM63 the odd rows (DS Table 10-3, condition 5, p. 41). The UG panel maps
its rows to Common 0–31 (UG §1.3, p. 1), so it would show the even rows of all
eight pages: the whole 64-row image squeezed into 32 lines, every other pixel
row missing. `A8h 3Fh` also spends half of each frame scanning 32 COM lines the
panel does not use, against the 1/32 duty it is specified for. This is read
from the datasheet tables; the panel's appearance has not been observed on this
robot.

## Pinout (controller, I2C mode)

| Signal | Die pin | Note | Source |
|--------|---------|------|--------|
| SCL | D0 | SCL and SDA both need pull-up resistors | DS §8.1.5, p. 19 |
| SDA | D1 (SDAIN) and D2 (SDAOUT), tied together | SDAOUT may be left unconnected, in which case the acknowledgement is ignored on the bus | DS §8.1.5 b, p. 19 |
| SA0 | D/C# | Selects `0x3C`/`0x3D` | DS §8.1.5 a, p. 19 |
| RES# | RES# | Initialises the device | DS §8.1.5, p. 19 |

The UG panel's 14-pin flex brings out VDD, VSS, VCC, VBAT, VBREF, IREF, VCOMH,
the two charge-pump capacitor pairs, RES#, SCL and SDA (UG §1.5, p. 3). It has no
D/C# pin, so a module built on it cannot strap the address. Header order of the
robocar-unified module is on its [board page](../boards/ssd1306-091in-128x32.md).

## I2C framing the firmware relies on

- Address byte `0111 10 SA0 R/W#` (DS §8.1.5 a, p. 19; Figure 8-7, p. 20).
- After the address, a control byte: Co bit, D/C# bit, then six zeros. D/C# = 0
  makes the following bytes commands, D/C# = 1 makes them GDDRAM data, and the
  column pointer auto-increments after each data byte (DS §8.1.5.2, p. 20). The
  repo's driver sends `0x00` before command runs and `0x40` before data.
- Each data byte fills one 8-row column of a page, D0 in the top row (DS §8.7,
  p. 25). A 128x32 frame is therefore 4 pages x 128 bytes = 512 bytes; a 128x64
  frame is 1024.
- `22h` sets the page start and end address for horizontal or vertical
  addressing mode, 0–7 (DS Table 9-1, p. 31).

## Application notes for this robot

- **The bus already runs at the controller's limit.** `I2C_MASTER_FREQ_HZ` is
  400 kHz; the SSD1306's minimum clock cycle is 2.5 µs with a 300 ns rise-time
  ceiling (DS Table 13-6, p. 54), so the pull-ups on the mux channel must meet
  that rise time.
- **ACK can be weak.** The ITO track resistance and the SDA pull-up form a
  divider, so the ACK may not reach a valid logic low (DS §8.1.5 b, p. 19).
  `robocar-bringup` probes for an ACK before initialising, so a panel that does
  not pull SDA low enough is reported as "no panel", not as a write error.
- **Charge pump.** Enable with `8Dh 14h` before `AFh` (AN §2.1, p. 3). The UG
  panel's example pairs `8Dh 14h` with pre-charge `D9h F1h` when VCC comes from
  the internal DC/DC, and `8Dh 10h` with `D9h 22h` when VCC is supplied
  externally (UG §4.4, p. 10).
- **VBAT range.** The charge pump needs VBAT of 3.3–4.2 V (AN §2, p. 3; UG §3.2,
  p. 7). A module on the XIAO's 3V3 rail sits at the bottom of that range before
  any drop across an on-board regulator.
- **VCOMH.** The datasheet lists `DBh` levels `00h`, `20h` (reset) and `30h`
  (DS Table 9-1, p. 32); the UG panel's example uses `DBh 40h` (UG §4.4, p. 10),
  which is what the repo's driver sends.
- **Power sequencing.** Hold RES# low at least 3 µs after VDD is stable; SEG/COM
  turn on 100 ms after `AFh`. At power-off send `AEh`, remove VCC, then VDD
  (DS §8.9, p. 27). A 4-pin module has no RES# pin, so the module's own circuit
  handles the reset.

## Datasheets & References

- **Datasheet PDF (Adafruit mirror; Rev 1.1 with the charge-pump App Note Rev 0.4
  appended):** <https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf>
- **Solomon Systech product page:** <https://www.solomon-systech.com/product/ssd1306>
- **Datasheet PDF (BuyDisplay, v1.1):** <https://www.buydisplay.com/download/ic/SSD1306.pdf>
  (returns a bot-check page to scripted downloads; not re-read)
- **128x32 panel specification, Univision UG-2832HSWEG02 (Adafruit mirror):**
  <https://cdn-shop.adafruit.com/datasheets/UG-2832HSWEG02.pdf>
