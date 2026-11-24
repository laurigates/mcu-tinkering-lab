"""The BQ25887 2S USB charger block's values against the datasheet equations.

docs/reference/blocks/bq25887-2s-usb-charger.toml carries its reasoning in
comments; this holds the numbers those comments rest on, so a changed resistor
fails here instead of passing every structural gate. Constants are from TI
SLUSD89B (Nov 2019): KILIM §7.5 and §8.3.7, ICB_LIM §8.3.4.3, the VT thresholds
§7.5 (JEITA comparator), RDSON_QCBX §7.5.

    python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import unittest

from hardware.blocks import load_block

SLUG = "bq25887-2s-usb-charger"

# SLUSD89B §7.5 / §8.3.7: IINMAX = KILIM / RILIM. The table's typical is 1110
# A*ohm; §8.3.7's worked example uses KILIM = 1276 as the maximum.
KILIM_TYP = 1110.0
KILIM_MAX = 1276.0
# USB-C's 1.5 A advertisement (Rp 22 kohm): the most a source promises without
# a CC resistor reading, which this block does not do. The hard limit tolerates
# the datasheet's own +3 % over it (its 820 ohm / 1.55 A example).
INPUT_CAP_A = 1.6
INPUT_FLOOR_A = 1.2  # below this the charge is slower than the 5 V / 2 A case needs
# SLUSD89B §7.5: ICB_MAX 400 mA at VCELL 4.2 V, RDSON_QCBX 1 ohm typical, 1.2 max.
CELL_V = 4.2
RDSON_CB_MAX = 1.2
ICB_MAX_A = 0.4
# 0805 rating 0.125 W; keep the balance resistors at 70 % or less of it.
R0805_W = 0.125
# SLUSD89B §7.5 JEITA comparator, as a fraction of REGN: the TS pin must sit
# between VT3 (45 degC, falling) and VT2 (10 degC, rising) to charge at full
# current and voltage.
VT3 = 0.4475
VT2 = 0.6825


def ohms(value: str) -> float:
    """'820' -> 820.0, '5.23k' -> 5230.0."""
    v = value.strip()
    mult = 1.0
    if v[-1] in "kK":
        mult, v = 1e3, v[:-1]
    elif v[-1] in "mM":
        mult, v = 1e6, v[:-1]
    return float(v) * mult


def parallel(*r: float) -> float:
    return 1.0 / sum(1.0 / x for x in r)


class TestChargerBlock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.block = load_block(SLUG)

    def value(self, key: str) -> float:
        return ohms(self.block.parts[key]["value"])

    def test_input_current_cap_is_the_ilim_resistor(self):
        r = self.value("rilim")
        self.assertLessEqual(KILIM_MAX / r, INPUT_CAP_A)
        self.assertGreaterEqual(KILIM_TYP / r, INPUT_FLOOR_A)

    def test_balance_current_and_resistor_heat(self):
        legs = sorted(k for k in self.block.parts if k.startswith("rcb"))
        self.assertGreaterEqual(len(legs), 1)
        r_total = parallel(*(self.value(k) for k in legs))
        i_cb = CELL_V / (r_total + RDSON_CB_MAX)
        self.assertLessEqual(i_cb, ICB_MAX_A)
        # Each leg carries an equal share of the balance current.
        leg_w = max((i_cb / len(legs)) ** 2 * self.value(k) for k in legs)
        self.assertLessEqual(leg_w, 0.7 * R0805_W)

    def test_ts_divider_reads_normal_without_a_thermistor(self):
        rt1, rt2, stand_in = self.value("rt1"), self.value("rt2"), self.value("rts")
        low = parallel(rt2, stand_in)
        frac = low / (rt1 + low)
        self.assertGreater(frac, VT3)
        self.assertLess(frac, VT2)

    def test_mid_pin_has_the_300_ohm_reverse_plug_resistor(self):
        # SLUSD89B §6, pin 9: 300 ohm in series with MID.
        self.assertEqual(self.value("rmid"), 300.0)

    def test_usb_c_port_is_a_sink_with_two_5k1_pull_downs(self):
        self.assertEqual(self.value("rd1"), 5100.0)
        self.assertEqual(self.value("rd2"), 5100.0)
        self.assertEqual(self.block.ports["CC1"], ["rd1.A"])
        self.assertEqual(self.block.ports["CC2"], ["rd2.A"])

    def test_vbus_reaches_only_the_charger_and_its_capacitor(self):
        # The XIAO's 5V pin is its VBUS (#1252): the charger's port must not be
        # able to feed it, so VBUS is a port to the receptacle and nothing else.
        self.assertEqual(sorted(self.block.ports["VBUS"]), ["cvbus.A", "ic.VBUS"])
        self.assertNotIn("5V", self.block.ports)
        for rail in self.block.rails:
            self.assertNotIn("ic.VBUS", [rail["from"], *rail.get("to", [])])

    def test_psel_is_strapped_low_in_the_block(self):
        self.assertIn("ic.PSEL", self.block.ports["GND"])


if __name__ == "__main__":
    unittest.main()
