# MSM261D3526H1CPM — PDM Digital MEMS Microphone

**Manufacturer:** MEMSensing Microsystems (Suzhou)
**Used in:** robocar-unified, as the onboard microphone of the Seeed XIAO
ESP32-S3 Sense camera board
([`../boards/xiao-esp32s3-sense.md`](../boards/xiao-esp32s3-sense.md)). The
firmware reads it through I2S0 in PDM RX mode at 16 kHz / 16-bit / mono
(`packages/robocar/unified/main/mic_pdm.c` lines 97–115, `pin_config.h`
lines 265–282); voice turns and the ambient-audio gate depend on it
([`voice.md`](../../../packages/robocar/unified/docs/firmware/voice.md)).

**Part number:** `MSM261D3526H1CPM`, as printed on the symbol `MIC1` in
Seeed's "XIAO ESP32S3 Exp. Board v1.2" schematic (the Sense camera/mic/SD
board). The repo's firmware and `WIRING.md` call it "MSM261D"; the board page
already gives the full number. MEMSensing sells near-identical parts
(MSM261D3526Z1CM, MSM261S3526Z0CM); their datasheets are not evidence for
this one.

## Key Specs

Datasheet test conditions unless noted: 23 °C, VDD 1.8 V, fCLOCK 2.4 MHz,
L/R grounded (DS p. 3).

| Parameter | Value | Source |
|-----------|-------|--------|
| Type | Omnidirectional, top-port, PDM output, fourth-order Σ-Δ modulator | DS p. 2 |
| Package | OCLGA, 3.50 × 2.65 × 0.94 mm, port Ø0.325 mm | DS p. 2; p. 10 |
| Supply (VDD) | 1.6–3.6 V | DS p. 3, General Microphone Specifications |
| Absolute max supply | −0.3 to 4.0 V | DS p. 3, Absolute Maximum Ratings |
| Max sound pressure (abs. max) | 140 dB SPL | DS p. 3 |
| Data format | ½-cycle PDM; increasing sound → increasing density of 1s | DS p. 3 |
| Output load | 200 pF max | DS p. 3 |
| Clock duty cycle | 40–60 % | DS p. 5, Interface Specifications |
| Logic levels | VIH ≥ 0.7·VDD, VIL ≤ 0.3·VDD; VOH ≥ VDD − 0.45 V, VOL ≤ 0.45 V at 2 mA | DS p. 5 |
| Power-up time | 6 ms typ, 20 ms max (VDD ≥ Vmin) | DS p. 3 |
| Wake-up time (from sleep) | 200 µs max (fCLOCK ≥ 151 kHz) | DS p. 3 |
| Fall-asleep time | 30 µs max (fCLOCK ≤ 50 kHz) | DS p. 3 |
| Mode-change time | 10 ms max | DS p. 3 |
| Storage temperature | −40 to 100 °C | DS p. 3 |
| ESD | ±2 kV HBM, ±200 V MM on I/O; ±8 kV to lid | DS p. 14, Reliability |
| MSL | Class 1 | DS p. 11 |

### Performance modes (selected by the clock frequency)

| Mode | fCLOCK | IDD | Sensitivity (94 dB SPL, 1 kHz) | SNR | AOP (10 % THD) | Source |
|------|--------|-----|-------------------------------|-----|----------------|--------|
| Sleep | 0–50 kHz | 1 µA max | — | — | — | DS p. 3 |
| Low-Power | 150–900 kHz | 290 µA @ 768 kHz | −25 dBFS (−26 to −24) | 62 dB(A), 20 Hz–8 kHz | 120 dB SPL | DS p. 5 |
| Standard Performance | 1.1–4.0 MHz | 670 µA @ 2.4 MHz | −26 dBFS (−27 to −25) | 64 dB(A), 20 kHz BW | 120 dB SPL | DS p. 4 |

THD 0.1 % at 94 dB SPL and PSRR 50 dBV/FS (200 mVpp, 1 kHz) in both active
modes (DS pp. 4–5). The datasheet gives no operating temperature range and no
frequency-response limits in numbers (only a plot, DS p. 7).

## Pins and channel select

| Pin | Function | Source |
|-----|----------|--------|
| 1 | VDD | DS p. 10 |
| 2 | CLK | DS p. 10 |
| 3 | DATA | DS p. 10 |
| 4, 6 | GND | DS p. 10 |
| 5 | L/R | DS p. 10 |

**L/R select** (DS p. 7, timing table): L/R to VDD → the mic asserts DATA on
the CLK rising edge and the host latches on the falling edge; L/R to ground →
asserts on the falling edge, host latches on the rising edge. Timing:
tEDGE ≤ 20 ns, tDZ (to high-Z) ≤ 40 ns, tDD (to data driven) ≤ 50 ns. The
recommended circuit decouples VDD with 100 nF and 10 µF ceramics placed close
to the pin (DS p. 13).

## On the XIAO Sense camera board

From the Exp. Board v1.2 schematic, `MIC1`:

| Mic pin | Net | XIAO / ESP32-S3 |
|---------|-----|-----------------|
| 1 VDD | `VCC_3V3`, 100 nF (C7) to ground | 3.3 V rail |
| 2 CLK | `IO42/PDM_CLK` through solder jumper JP1 | GPIO42 |
| 3 DATA | `IO41/PDM_DATA` through solder jumper JP2 | GPIO41 |
| 4, 6 GND | ground | — |
| 5 L/R | ground | — (left / "Mic(Low)") |

The ESP32-S3 side of each jumper also has an unpopulated 1×2 pad (`J4` on
CLK, `J5` on DATA, both marked DNP). The board-to-board connector JA3 labels
pins 4 and 5 `IO42/PDM_CLK` and `IO41/PDM_DATA`, grouped as `MIC`. Only the
100 nF capacitor is fitted; the datasheet's 10 µF is not on this board.
Seeed's getting-started wiki pin table gives the same assignment:
"Digital microphone_CLK … GPIO42", "Digital microphone_DATA … GPIO41".

**Agreement with the firmware:** `pin_config.h` lines 277–278 define
`MIC_PDM_CLK_PIN GPIO_NUM_42` and `MIC_PDM_DATA_PIN GPIO_NUM_41`; this matches
the schematic and the wiki. The firmware's mono slot mask is
`I2S_PDM_SLOT_LEFT` (ESP-IDF v5.4 `I2S_PDM_RX_SLOT_DEFAULT_CONFIG`), and the
mic's L/R is grounded; that the ESP32-S3's "left" PDM slot is the
rising-edge latch the datasheet asks for is not stated in either source, but
the mic is known to work on the robot.

**Disagreement with the board page:**
[`xiao-esp32s3-sense.md`](../boards/xiao-esp32s3-sense.md) says to cut
"solder jumpers J1/J2" to reclaim D11/D12. In the v1.2 schematic the jumpers
in the CLK and DATA lines are `JP1`/`JP2`; `J4`/`J5` are the DNP pads. The
board page was not edited here.

## PDM clock the firmware generates

ESP-IDF v5.4 computes the PDM RX clock as `sample_rate × 64 × (DSR_16S ? 2 :
1)` (`components/esp_driver_i2s/i2s_pdm.c` line 334;
`I2S_LL_PDM_BCK_FACTOR = 64`, `hal/esp32s3/include/hal/i2s_ll.h` line 33),
and `I2S_PDM_RX_CLK_DEFAULT_CONFIG` selects `I2S_PDM_DSR_8S`
(`driver/i2s_pdm.h` lines 61–67). `mic_pdm.c` uses that default at
`MIC_SAMPLE_RATE_HZ = 16000`, so GPIO42 carries **16 000 × 64 = 1.024 MHz**.

That falls in the gap between the datasheet's Low-Power range (150–900 kHz)
and its Standard Performance range (1.1–4.0 MHz) (DS p. 3; state diagram
p. 6). The datasheet does not say how the part behaves there. It works on
the robot; which mode's sensitivity and SNR apply is unverified. Moving to a
listed range would mean a different sample rate or `I2S_PDM_DSR_16S`
(2.048 MHz at 16 kHz); that is a firmware decision outside this page.

`pin_config.h` line 280 and `WIRING.md` describe 16 kHz as "fixed by the
microphone". The datasheet fixes no sample rate: the output rate is set by
the host's clock and decimation. 16 kHz is the firmware's choice (and what
Gemini expects).

## Open checklist

- [ ] Sensitivity and SNR at the 1.024 MHz clock the firmware drives, which
      is in neither datasheet range. Either confirm with MEMSensing, or
      compare a fixed stimulus at DSR_8S (1.024 MHz) and DSR_16S (2.048 MHz).
- [ ] Operating temperature range: not in the datasheet found.
- [ ] Whether the ESP32-S3's `I2S_PDM_SLOT_LEFT` samples on the CLK edge
      the datasheet gives for L/R = ground (host latches on rising edge).
- [ ] The datasheet copy is a third-party mirror (a Shopify CDN file); no
      copy was found on memsensing.com. Check for a newer revision than
      V1.2 there.
- [ ] Which Exp. Board revision the robot carries; the mic block was read
      from v1.2 only.

## Datasheets & References

- **MSM261D3526H1CPM datasheet V1.2, Dec 2018, DOC NO DS-042** (MEMSensing,
  15 pages, SHA-256
  `bf8f9f07ba6a2748f0235c2bcc4d1c11f36f05f2b845427fd6584786a11b9050`),
  mirror: <https://cdn.shopify.com/s/files/1/0174/1800/files/mic-MSM261D3526H1CPM-ENG.pdf>.
  Page numbers above are PDF pages.
- **Seeed XIAO ESP32S3 Exp. Board v1.2 schematic** (2026-08-10; `MIC1`,
  JP1/JP2, J4/J5, JA3):
  <https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_Exp_Board_v1.2_SCH_PDF_260810.pdf>
- **Seeed XIAO ESP32-S3 getting-started wiki** (GPIO42/GPIO41 pin table):
  <https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/>
- **ESP-IDF v5.4**, PDM RX clock:
  [`components/esp_driver_i2s/i2s_pdm.c`](https://github.com/espressif/esp-idf/blob/v5.4/components/esp_driver_i2s/i2s_pdm.c#L334),
  [`components/hal/esp32s3/include/hal/i2s_ll.h`](https://github.com/espressif/esp-idf/blob/v5.4/components/hal/esp32s3/include/hal/i2s_ll.h#L33),
  [`components/esp_driver_i2s/include/driver/i2s_pdm.h`](https://github.com/espressif/esp-idf/blob/v5.4/components/esp_driver_i2s/include/driver/i2s_pdm.h#L61)
