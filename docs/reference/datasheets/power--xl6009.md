# XL6009 — DC-DC Boost Converter

**Manufacturer:** XLSEMI
**Used in:** nothing currently. This page was cited as the robocar power supply, which was never possible — a *boost* converter cannot step 7.4 V down to 5 V. That project uses an [LM2596 buck](power--lm2596.md); kept here as a general reference for step-up applications.

## Key Specs

| Parameter | Value |
|-----------|-------|
| Topology | Boost / Buck-boost / Inverting |
| Input voltage | 3.6–36V |
| Output voltage | 5–35V (adjustable via potentiometer) |
| Switching current | 4A max |
| Switching frequency | 400 kHz (fixed) |
| Efficiency | Up to 94% |
| Load regulation | 0.5% |
| Protection | Overcurrent, thermal |

## Datasheets & References

- **Datasheet PDF:** <https://www.haoyuelectronics.com/Attachment/XL6009/XL6009-DC-DC-Converter-Datasheet.pdf>
- **Datasheet PDF (XLSEMI):** <https://beriled.biz/data/files/XL6009.pdf>
- **Module datasheet:** <https://www.handsontec.com/dataspecs/power%20supply/XL6009-Buck-Boost.pdf>
