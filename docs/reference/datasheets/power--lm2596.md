# LM2596 — DC-DC Step-Down (Buck) Converter

**Manufacturer:** Texas Instruments (SIMPLE SWITCHER® family)
**Used in:** robocar-unified power system (2×18650 in series, 7.4 V → 5 V for the
XIAO, TB6612FNG, PCA9685/servos and MAX98357A)

## Key Specs

Figures from TI datasheet **SNVS124G** (November 1999, revised March 2023).

| Parameter | Value |
|-----------|-------|
| Topology | Step-down (buck), non-synchronous |
| Output current | 3 A |
| Input voltage | Up to 40 V |
| Output voltage | Fixed 3.3 V / 5 V / 12 V versions, or adjustable 1.2–37 V |
| Output tolerance | ±4% over line and load |
| Switching frequency | 150 kHz fixed, ±15% |
| Dropout (input−output differential) | ~1.25 V at 3 A, ~0.95 V at 1 A (Figure 7-6, 25 °C) |
| Efficiency at 5 V out | ~75–80% at 3 A across the input range (Figure 7-3) |
| Standby current | 80 µA typical (ON/OFF pin) |
| Protection | Current limit (two-stage frequency-reducing), thermal shutdown |

## Two things that bite in practice

**The common breakout is the adjustable variant.** It has a multi-turn trimpot
and arrives set to an arbitrary voltage — it does not produce 5 V until somebody
sets it there. Adjust it unloaded, on a meter, before connecting anything.
Downstream, the MAX98357A's recommended maximum supply is 5.5 V (absolute
maximum 6 V) and the XIAO's 5V pin feeds its onboard regulator, so a trimpot
left high damages parts.

**Dropout sets the usable battery range, not cell chemistry.** At 5 V out the
dropout figures above put the limit near **6.3 V of input under heavy load** —
about 3.15 V per cell on a 2S 18650 pack, which is reached while the cells still
hold usable charge. The rail sags rather than cutting out cleanly, so the
symptoms are weak servos, clipping audio and eventually resets rather than an
obvious power-off.

## Datasheets & References

- **Datasheet PDF (TI, SNVS124G):** <https://www.ti.com/lit/ds/symlink/lm2596.pdf>
- **Product page:** <https://www.ti.com/product/LM2596>

## Related

- [XL6009](power--xl6009.md) — the boost converter this project's documentation
  incorrectly named until 2026-09. A boost cannot step 7.4 V down to 5 V, which
  is the tell that the reference was stale rather than a design choice.
