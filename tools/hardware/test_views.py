"""Tests for the build-filtered views of the join (#750).

One hardware.toml describes two builds: the hand-wired harness the docs draw,
and the carrier PCB tools/kicad generates. These pin down the `build` filter,
rail continuation, output pins, [net_names] and the carrier part keys.

    python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import subprocess
import tempfile
import tomllib
import unittest
from pathlib import Path

from hardware import CARRIER, HARNESS, VIEWS, HardwareError, join
from hardware.test_hardware import BOARD_MD, HEADER_H, REPO_ROOT, write

# LED_PIN is GPIO1 (pad D0) and BEEP_PIN GPIO7 (pad D2) on the test board.
# BEEP_PIN is wired only on the carrier, and excused only on the harness.
SIDECAR = """\
[source]
convention = "robocar"
header = "main/pin_config.h"
board = "board.md"
ref = "U1"

[parts.led]
name = "LED"
kind = "led"
ref = "D1"

[parts.pack]
name = "Pack"
kind = "battery"
ref = "J1"

[parts.drv]
name = "Driver"
kind = "driver"
ref = "A1"

[parts.motor]
name = "Motor"
kind = "motor"
ref = "M1"

[parts.cap]
name = "Cap"
kind = "capacitor"
build = "carrier"
part = "c0805"
ref = "C1"
value = "100nF"
lcsc = "C49678"

[parts.lamp]
name = "Lamp"
kind = "lamp"
build = "harness"

[[nets]]
role = "LED_PIN"
to = "led.A"

[[nets]]
role = "BEEP_PIN"
to = "cap.A"
build = "carrier"

[[undrawn]]
role = "BEEP_PIN"
why = "only the carrier has the capacitor"
build = "harness"

[[rails]]
name = "5V"
from = "pack.+"
to = ["mcu.5V", "led.VCC"]

[[rails]]
name = "5V"
build = "carrier"
to = ["cap.+", "drv.VM"]

[[rails]]
name = "GND"
build = "carrier"
from = "pack.-"
to = ["led.K", "cap.-"]

[[outputs]]
from = "drv"
to = ["motor"]
pins = ["A1", "A2"]
net = "M_{pin}"

[[outputs]]
from = "led"
to = ["lamp"]
build = "harness"

[net_names]
LED_PIN = "LED"
BEEP_PIN = "BEEP"
5V = "+5V"
"""


class ViewTest(unittest.TestCase):
    """join(view=) against a synthetic project, one failure mode per test."""

    def make(self, sidecar: str = SIDECAR) -> Path:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        write(root / "board.md", BOARD_MD)
        proj = root / "proj"
        write(proj / "main/pin_config.h", HEADER_H)
        (proj / "hardware.toml").write_text(sidecar)
        return proj

    def join(self, sidecar: str = SIDECAR, view: str | None = None):
        proj = self.make(sidecar)
        if view is None:
            return join(proj, repo_root=proj.parent)
        return join(proj, repo_root=proj.parent, view=view)

    def raises(self, pattern: str, sidecar: str, *views: str):
        for view in views or VIEWS:
            with (
                self.subTest(view=view),
                self.assertRaisesRegex(HardwareError, pattern),
            ):
                self.join(sidecar, view)

    def rail(self, model, name):
        (rail,) = [r for r in model.rails if r.name == name]
        return [f"{e.part}.{e.pin}" for e in (rail.source, *rail.loads)]


class MembershipTest(ViewTest):
    def test_the_default_view_is_the_harness(self):
        model = self.join()
        self.assertEqual(model.view, HARNESS)
        self.assertNotIn("cap", model.parts)
        self.assertIn("lamp", model.parts)
        self.assertEqual([n.role for n in model.nets], ["LED_PIN"])
        self.assertEqual([u.role for u in model.undrawn], ["BEEP_PIN"])
        self.assertEqual([r.name for r in model.rails], ["5V"])
        self.assertEqual(self.rail(model, "5V"), ["pack.+", "mcu.5V", "led.VCC"])
        self.assertEqual([o.part for o in model.outputs], ["drv", "led"])

    def test_the_carrier_view_is_the_mirror_image(self):
        model = self.join(view=CARRIER)
        self.assertEqual(model.view, CARRIER)
        self.assertIn("cap", model.parts)
        self.assertNotIn("lamp", model.parts)
        self.assertEqual([n.role for n in model.nets], ["LED_PIN", "BEEP_PIN"])
        self.assertEqual(model.undrawn, ())
        self.assertEqual([r.name for r in model.rails], ["5V", "GND"])
        self.assertEqual([o.part for o in model.outputs], ["drv"])

    def test_an_explicit_harness_view_equals_the_default(self):
        proj = self.make()
        self.assertEqual(
            join(proj, repo_root=proj.parent, view=HARNESS),
            join(proj, repo_root=proj.parent),
        )

    def test_an_unknown_view_fails(self):
        with self.assertRaisesRegex(HardwareError, "unknown view 'bogus'"):
            self.join(view="bogus")

    def test_a_build_that_is_not_a_view_fails_in_every_view(self):
        for value in ('"both"', '"carier"', '["carrier"]'):
            broken = SIDECAR.replace(
                'build = "harness"\n\n[[rails]]', f"build = {value}\n\n[[rails]]"
            )
            self.assertNotEqual(broken, SIDECAR)
            with self.subTest(value=value):
                self.raises(r"\[\[undrawn\]\] #1: build must be one of", broken)

    def test_a_bad_build_on_a_part_fails_in_the_other_view_too(self):
        broken = SIDECAR.replace(
            'kind = "lamp"\nbuild = "harness"', 'kind = "lamp"\nbuild = "both"'
        )
        self.assertNotEqual(broken, SIDECAR)
        self.raises(r"\[parts.lamp\]: build must be one of", broken)

    def test_an_unknown_key_on_an_entry_of_the_other_view_fails(self):
        # A typo in a carrier-only entry must still fail the docs' harness join.
        broken = SIDECAR.replace(
            'to = "cap.A"\nbuild = "carrier"', 'to = "cap.A"\nbiuld = "carrier"'
        )
        self.assertNotEqual(broken, SIDECAR)
        self.raises(r"\[\[nets\]\] #2: unknown key\(s\) \['biuld'\]", broken)

    def test_a_dropped_part_named_by_an_in_view_entry_says_why(self):
        broken = SIDECAR.replace('to = "cap.A"\nbuild = "carrier"', 'to = "cap.A"')
        self.raises(
            r"\[\[nets\]\] #2: part 'cap' is build = 'carrier' and not in the harness view",
            broken,
            HARNESS,
        )

    def test_a_dropped_part_in_a_rail_or_output_says_why(self):
        rail = SIDECAR.replace(
            'to = ["mcu.5V", "led.VCC"]', 'to = ["mcu.5V", "led.VCC", "lamp.V"]'
        )
        self.raises(
            r"\[\[rails\]\] #1: part 'lamp' is build = 'harness' and not in the carrier view",
            rail,
            CARRIER,
        )
        output = SIDECAR.replace(
            'to = ["lamp"]\nbuild = "harness"', 'to = ["lamp", "cap"]'
        )
        self.raises(
            "part 'cap' is build = 'carrier' and not in the harness view",
            output,
            HARNESS,
        )


class CompletenessTest(ViewTest):
    def test_a_role_wired_only_on_the_carrier_must_be_excused_on_the_harness(self):
        broken = SIDECAR.replace(
            '[[undrawn]]\nrole = "BEEP_PIN"\nwhy = "only the carrier has the capacitor"\n'
            'build = "harness"\n',
            "",
        )
        self.assertNotEqual(broken, SIDECAR)
        self.raises(r"on no net and not excused: BEEP_PIN \(GPIO7\)", broken, HARNESS)
        self.assertIn("BEEP_PIN", {n.role for n in self.join(broken, CARRIER).nets})

    def test_an_undrawn_entry_in_both_views_clashes_with_the_carrier_net(self):
        # Drop the harness filter and the carrier view fails.
        broken = SIDECAR.replace(
            'why = "only the carrier has the capacitor"\nbuild = "harness"',
            'why = "only the carrier has the capacitor"',
        )
        self.raises("'BEEP_PIN' is on a net and excused as undrawn", broken, CARRIER)

    def test_a_harness_only_undrawn_is_ignored_on_the_carrier(self):
        self.assertEqual(self.join(view=CARRIER).undrawn, ())


class RailContinuationTest(ViewTest):
    def test_continuations_merge_after_the_base_in_file_order(self):
        model = self.join(view=CARRIER)
        self.assertEqual(
            self.rail(model, "5V"),
            ["pack.+", "mcu.5V", "led.VCC", "cap.+", "drv.VM"],
        )

    def test_a_continuation_before_its_base_keeps_the_first_position(self):
        # WIRING.md's power diagram follows model.rails order.
        text = SIDECAR.replace(
            '[[rails]]\nname = "5V"\nfrom = "pack.+"\nto = ["mcu.5V", "led.VCC"]\n\n',
            "",
        )
        text = text.replace(
            '[[outputs]]\nfrom = "drv"',
            '[[rails]]\nname = "5V"\nfrom = "pack.+"\nto = ["mcu.5V", "led.VCC"]\n\n'
            '[[outputs]]\nfrom = "drv"',
            1,
        )
        model = self.join(text, CARRIER)
        self.assertEqual([r.name for r in model.rails], ["5V", "GND"])
        self.assertEqual(
            self.rail(model, "5V"),
            ["pack.+", "mcu.5V", "led.VCC", "cap.+", "drv.VM"],
        )

    def test_a_continuation_with_no_base_in_the_view_fails(self):
        broken = SIDECAR.replace(
            '[[rails]]\nname = "5V"\nfrom = "pack.+"',
            '[[rails]]\nname = "5V"\nbuild = "harness"\nfrom = "pack.+"',
        )
        self.raises(
            r"rail '5V' has loads but no `from` in the carrier view", broken, CARRIER
        )
        self.assertEqual([r.name for r in self.join(broken).rails], ["5V"])

    def test_two_bases_of_one_rail_fail(self):
        broken = SIDECAR.replace(
            'build = "carrier"\nto = ["cap.+"',
            'build = "carrier"\nfrom = "drv.VCC"\nto = ["cap.+"',
        )
        self.assertNotEqual(broken, SIDECAR)
        self.raises(r"\[\[rails\]\] #2: rail '5V' declared twice", broken, CARRIER)

    def test_a_load_repeated_across_the_merge_is_listed_twice(self):
        broken = SIDECAR.replace(
            'to = ["cap.+", "drv.VM"]', 'to = ["cap.+", "led.VCC"]'
        )
        self.raises(r"\[\[rails\]\] #2: \['led.VCC'\] listed twice", broken, CARRIER)

    def test_a_continuation_load_on_another_rail_is_a_short(self):
        broken = SIDECAR.replace('to = ["cap.+", "drv.VM"]', 'to = ["cap.+", "led.K"]')
        self.raises(
            r"\[\[rails\]\] #3: led.K is on rail '5V' and rail 'GND' — that is a short",
            broken,
            CARRIER,
        )


class OutputPinsTest(ViewTest):
    def test_pins_and_their_template_are_kept(self):
        (out,) = self.join(view=CARRIER).outputs
        self.assertEqual((out.pins, out.net), (("A1", "A2"), "M_{pin}"))
        lamp = self.join().outputs[1]
        self.assertEqual((lamp.pins, lamp.net), ((), ""))

    def test_pins_that_are_not_a_list_of_names_fail(self):
        for value in ('"A1"', "[]", '["A1", 2]', '["A1", ""]'):
            broken = SIDECAR.replace('pins = ["A1", "A2"]', f"pins = {value}")
            with self.subTest(value=value):
                self.raises("'pins' must be a non-empty list of pin names", broken)

    def test_a_pin_listed_twice_fails(self):
        broken = SIDECAR.replace('pins = ["A1", "A2"]', 'pins = ["A1", "A1"]')
        self.raises(r"pins \['A1'\] listed twice", broken)

    def test_pins_without_a_net_template_fail(self):
        broken = SIDECAR.replace('net = "M_{pin}"\n', "")
        self.raises("'pins' needs a 'net' name template", broken)

    def test_a_net_template_without_pins_fails(self):
        broken = SIDECAR.replace('pins = ["A1", "A2"]\n', "")
        self.raises("'net' names the nets of 'pins'", broken)

    def test_a_template_with_no_pin_field_cannot_name_two_pins(self):
        broken = SIDECAR.replace('net = "M_{pin}"', 'net = "M"')
        self.raises(r"net 'M' names 2 pins and has no \{pin\}", broken)
        one = broken.replace('pins = ["A1", "A2"]', 'pins = ["A1"]')
        self.assertEqual(self.join(one, CARRIER).outputs[0].net, "M")

    def test_a_template_field_other_than_pin_is_a_hardware_error(self):
        for value in ("M_{foo}", "M_{pin!r}", "M_{pin:>3}", "M_{"):
            broken = SIDECAR.replace('net = "M_{pin}"', f'net = "{value}"')
            with self.subTest(value=value):
                self.raises("net 'M_", broken)

    def test_an_output_pin_on_a_rail_fails(self):
        broken = SIDECAR.replace('to = ["cap.+", "drv.VM"]', 'to = ["cap.+", "drv.A1"]')
        self.raises(r"\[\[outputs\]\] #1: drv.A1 is also rail 5V", broken, CARRIER)

    def test_a_load_pin_on_a_rail_fails(self):
        broken = SIDECAR.replace(
            'to = ["led.K", "cap.-"]', 'to = ["led.K", "cap.-", "motor.A2"]'
        )
        self.raises(r"\[\[outputs\]\] #1: motor.A2 is also rail GND", broken, CARRIER)

    def test_an_output_pin_on_a_signal_net_fails(self):
        broken = SIDECAR.replace(
            'role = "LED_PIN"\nto = "led.A"', 'role = "LED_PIN"\nto = "drv.A2"'
        )
        self.raises(r"drv.A2 is also the signal net LED_PIN", broken)


# One driver, two loads on separate pins (#769): drv's A pair to motor's 1/2,
# its B pair to motor2's 1/2.
TWO_LOADS = SIDECAR.replace(
    '[[outputs]]\nfrom = "drv"\nto = ["motor"]\npins = ["A1", "A2"]\nnet = "M_{pin}"\n',
    """[[outputs]]
from = "drv"
to = ["motor", "motor2"]
pins = ["A1", "A2", "B1", "B2"]
net = "M_{pin}"
load_pins.motor = { A1 = "1", A2 = "2" }
load_pins.motor2 = { B1 = "1", B2 = "2" }
""",
).replace(
    "[parts.cap]",
    '[parts.motor2]\nname = "Motor 2"\nkind = "motor"\nref = "M2"\n\n[parts.cap]',
)


class LoadPinsTest(ViewTest):
    def test_two_loads_take_separate_driver_pins(self):
        (out,) = self.join(TWO_LOADS, CARRIER).outputs
        self.assertEqual(
            out.wires,
            (
                ("A1", "motor", "1"),
                ("A2", "motor", "2"),
                ("B1", "motor2", "1"),
                ("B2", "motor2", "2"),
            ),
        )

    def test_without_load_pins_each_load_takes_every_pin_by_name(self):
        (out,) = self.join(view=CARRIER).outputs
        self.assertEqual(out.wires, (("A1", "motor", "A1"), ("A2", "motor", "A2")))

    def test_load_pins_without_pins_fail(self):
        broken = TWO_LOADS.replace(
            'pins = ["A1", "A2", "B1", "B2"]\nnet = "M_{pin}"\n', ""
        )
        self.raises("'load_pins' maps the driver pins of 'pins', and has none", broken)

    def test_load_pins_must_name_every_load_and_no_other(self):
        missing = TWO_LOADS.replace('load_pins.motor2 = { B1 = "1", B2 = "2" }\n', "")
        self.raises(r"'load_pins' must name each load in 'to'.*motor2", missing)
        extra = TWO_LOADS + ""
        extra = extra.replace(
            'load_pins.motor2 = { B1 = "1", B2 = "2" }\n',
            'load_pins.motor2 = { B1 = "1", B2 = "2" }\nload_pins.lamp = { A1 = "1" }\n',
        )
        self.raises(r"'load_pins' must name each load in 'to'.*lamp", extra)

    def test_a_mapped_driver_pin_must_be_in_pins(self):
        broken = TWO_LOADS.replace('{ B1 = "1", B2 = "2" }', '{ B1 = "1", B3 = "2" }')
        self.raises(r"load_pins.motor2: B3 is not in 'pins'", broken)

    def test_a_driver_pin_on_no_load_fails(self):
        broken = TWO_LOADS.replace('{ B1 = "1", B2 = "2" }', '{ B1 = "1" }')
        self.raises(r"load_pins: driver pin\(s\) \['B2'\] reach no load", broken)

    def test_two_driver_pins_on_one_load_pin_are_a_short(self):
        broken = TWO_LOADS.replace('{ B1 = "1", B2 = "2" }', '{ B1 = "1", B2 = "1" }')
        self.raises(r"load_pins.motor2: B1 and B2 both go to motor2.1", broken)

    def test_load_pins_of_the_wrong_shape_fail(self):
        for value in (
            '"A1"',
            '{ motor = "1" }',
            "{ motor = { A1 = 1 } }",
            '{ motor = { A1 = "" } }',
        ):
            broken = TWO_LOADS.replace(
                'load_pins.motor = { A1 = "1", A2 = "2" }\nload_pins.motor2 = { B1 = "1", B2 = "2" }\n',
                f"load_pins = {value}\n",
            )
            with self.subTest(value=value):
                self.raises("load_pins", broken)

    def test_a_mapped_load_pin_on_a_rail_fails(self):
        broken = TWO_LOADS.replace(
            'to = ["led.K", "cap.-"]', 'to = ["led.K", "cap.-", "motor2.2"]'
        )
        self.raises(r"\[\[outputs\]\] #1: motor2.2 is also rail GND", broken, CARRIER)
        # The driver pin's own name on the load is not one it is wired to.
        fine = TWO_LOADS.replace(
            'to = ["led.K", "cap.-"]', 'to = ["led.K", "cap.-", "motor2.B2"]'
        )
        self.assertEqual(len(self.join(fine, CARRIER).outputs), 1)

    def test_net_names_cover_every_driver_pin(self):
        clash = TWO_LOADS.replace('LED_PIN = "LED"', 'LED_PIN = "M_B2"')
        self.raises(r"'M_B2' is used by role LED_PIN and output drv.B2", clash, CARRIER)


class PartKeysTest(ViewTest):
    def test_the_carrier_keys_are_kept(self):
        model = self.join(view=CARRIER)
        cap = model.parts["cap"]
        self.assertEqual(
            (cap.part, cap.ref, cap.value, cap.lcsc, cap.build),
            ("c0805", "C1", "100nF", "C49678", CARRIER),
        )
        self.assertEqual(model.parts["led"].build, "")
        self.assertEqual((model.mcu_ref, model.mcu_part), ("U1", ""))
        with_part = SIDECAR.replace('ref = "U1"', 'ref = "U1"\npart = "xiao"')
        self.assertEqual(self.join(with_part).mcu_part, "xiao")

    def test_a_bad_ref_fails_in_every_view(self):
        for ref in ("c1", "C", "1C", "C1a", "C-1"):
            broken = SIDECAR.replace('ref = "C1"', f'ref = "{ref}"')
            with self.subTest(ref=ref):
                self.raises(r"\[parts.cap\]: ref .* does not match", broken)
        self.raises(
            r"\[source\]: ref 'u1' does not match",
            SIDECAR.replace('ref = "U1"', 'ref = "u1"'),
        )

    def test_a_ref_used_twice_in_the_view_fails(self):
        broken = SIDECAR.replace('ref = "M1"', 'ref = "D1"')
        self.raises(r"ref 'D1' is used by \[parts.led\] and \[parts.motor\]", broken)

    def test_a_part_ref_equal_to_the_mcu_ref_fails(self):
        broken = SIDECAR.replace('ref = "M1"', 'ref = "U1"')
        self.raises(r"ref 'U1' is used by \[source\] and \[parts.motor\]", broken)

    def test_one_ref_in_two_views_is_not_a_clash(self):
        both = SIDECAR.replace('kind = "lamp"\n', 'kind = "lamp"\nref = "C1"\n')
        for view in VIEWS:
            self.join(both, view)

    def test_a_bad_lcsc_fails_in_every_view(self):
        for lcsc in ("49678", "c49678", "C", "C4967 8"):
            broken = SIDECAR.replace('lcsc = "C49678"', f'lcsc = "{lcsc}"')
            with self.subTest(lcsc=lcsc):
                self.raises(r"\[parts.cap\]: lcsc .* does not match", broken)

    def test_an_empty_value_or_part_fails(self):
        for key in ("value", "part"):
            broken = SIDECAR.replace(f'{key} = "', f'{key} = "" #', 1)
            with self.subTest(key=key):
                self.raises(f"missing or non-string '{key}'", broken)


class NetNamesTest(ViewTest):
    def test_net_names_are_read_in_both_views(self):
        for view in VIEWS:
            self.assertEqual(
                self.join(view=view).net_names,
                {"LED_PIN": "LED", "BEEP_PIN": "BEEP", "5V": "+5V"},
            )

    def test_an_unknown_key_fails_on_the_carrier_and_is_ignored_on_the_harness(self):
        broken = SIDECAR + 'NOPE = "X"\n'
        self.raises(
            r"\[net_names\] NOPE: neither a role on a net nor a rail", broken, CARRIER
        )
        self.join(broken, HARNESS)

    def test_a_key_whose_net_is_in_the_other_view_only_fails(self):
        # LED_PIN moves to a harness-only net, so the carrier has no LED_PIN net.
        broken = SIDECAR.replace(
            '[[nets]]\nrole = "LED_PIN"',
            '[[nets]]\nbuild = "harness"\nrole = "LED_PIN"',
        )
        broken = broken.replace(
            '[[undrawn]]\nrole = "BEEP_PIN"',
            '[[undrawn]]\nrole = "LED_PIN"\nwhy = "x"\nbuild = "carrier"\n\n'
            '[[undrawn]]\nrole = "BEEP_PIN"',
        )
        self.raises(r"\[net_names\] LED_PIN: neither", broken, CARRIER)

    def test_two_keys_with_one_value_fail(self):
        broken = SIDECAR.replace('BEEP_PIN = "BEEP"', 'BEEP_PIN = "LED"')
        self.raises(
            "net name 'LED' is used by role LED_PIN and role BEEP_PIN", broken, CARRIER
        )
        self.join(broken, HARNESS)

    def test_a_value_equal_to_another_roles_default_name_fails(self):
        broken = SIDECAR.replace('LED_PIN = "LED"\n', "").replace(
            'BEEP_PIN = "BEEP"', 'BEEP_PIN = "LED_PIN"'
        )
        self.raises(
            "net name 'LED_PIN' is used by role LED_PIN and role BEEP_PIN",
            broken,
            CARRIER,
        )

    def test_a_value_equal_to_a_rail_name_fails(self):
        broken = SIDECAR.replace('LED_PIN = "LED"', 'LED_PIN = "GND"')
        self.raises(
            "net name 'GND' is used by role LED_PIN and rail GND", broken, CARRIER
        )

    def test_an_expanded_output_name_equal_to_a_rail_fails(self):
        broken = SIDECAR.replace('5V = "+5V"', '5V = "M_A2"')
        self.raises(
            "net name 'M_A2' is used by rail 5V and output drv.A2", broken, CARRIER
        )

    def test_markup_characters_in_a_name_fail_in_every_view(self):
        for name in ("D[0..1]", "A B", "~{EN}", "X/Y"):
            with self.subTest(name=name):
                self.raises(
                    "is not a usable net name",
                    SIDECAR.replace('LED_PIN = "LED"', f'LED_PIN = "{name}"'),
                )
                self.raises(
                    "is not a usable net name",
                    SIDECAR.replace('name = "GND"', f'name = "{name}"'),
                )
                if "{" not in name:  # a brace is a template field, checked apart
                    self.raises(
                        "is not a usable net name",
                        SIDECAR.replace('net = "M_{pin}"', f'net = "{name}{{pin}}"'),
                    )

    def test_net_names_that_are_not_a_table_of_names_fail(self):
        table = SIDECAR.index("[net_names]")
        self.raises("net_names must be a", "net_names = 5\n" + SIDECAR[:table])
        self.raises(
            "expected a non-empty net name", SIDECAR.replace('5V = "+5V"', "5V = 5")
        )
        self.raises(
            "expected a non-empty net name", SIDECAR.replace('5V = "+5V"', '5V = ""')
        )


class RealDataTest(unittest.TestCase):
    """Invariants over every git-tracked hardware.toml, not snapshots of one."""

    def sidecars(self) -> list[Path]:
        listed = subprocess.run(
            ["git", "ls-files", "-z", "--", ":(glob)**/hardware.toml"],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        paths = [REPO_ROOT / p for p in listed.stdout.split("\0") if p]
        self.assertTrue(paths, "no tracked hardware.toml: the glob or git is broken")
        return paths

    def test_every_tracked_sidecar_joins_in_both_views(self):
        for path in self.sidecars():
            for view in VIEWS:
                with self.subTest(sidecar=str(path.relative_to(REPO_ROOT)), view=view):
                    self.assertEqual(join(path.parent, view=view).view, view)

    def test_each_view_holds_exactly_its_own_entries(self):
        def kept(tables, view):
            return [t for t in tables if t.get("build", view) == view]

        for path in self.sidecars():
            data = tomllib.loads(path.read_text(encoding="utf-8"))
            for view in VIEWS:
                with self.subTest(sidecar=str(path.relative_to(REPO_ROOT)), view=view):
                    model = join(path.parent, view=view)
                    parts = data.get("parts", {})
                    self.assertEqual(
                        set(model.parts),
                        {k for k, t in parts.items() if t.get("build", view) == view},
                    )
                    self.assertTrue(
                        all(p.build in ("", view) for p in model.parts.values())
                    )
                    self.assertEqual(
                        len(model.nets), len(kept(data.get("nets", []), view))
                    )
                    self.assertEqual(
                        len(model.channel_nets),
                        len(kept(data.get("channel_nets", []), view)),
                    )
                    self.assertEqual(
                        len(model.bus_nets),
                        len(kept(data.get("bus_nets", []), view)),
                    )
                    self.assertEqual(
                        [u.role for u in model.undrawn],
                        [t["role"] for t in kept(data.get("undrawn", []), view)],
                    )
                    self.assertEqual(
                        [r.name for r in model.rails],
                        list(
                            dict.fromkeys(
                                t["name"] for t in kept(data.get("rails", []), view)
                            )
                        ),
                    )
                    self.assertEqual(
                        len(model.outputs), len(kept(data.get("outputs", []), view))
                    )


if __name__ == "__main__":
    unittest.main()
