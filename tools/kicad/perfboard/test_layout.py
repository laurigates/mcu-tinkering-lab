"""Tests for perfboard.toml, placement and the wire plan, on the balancebot carrier.

Placement and wire checks run on the frozen balancebot inputs
(`kicad.testfixture`, with the frozen perfboard.toml), each mutated in memory
the way a layout edit could break it. `LiveProjectTests` reads the live
project: the committed perfboard.svg and perfboard.md must be what
`kicad.perfboard` writes (as `--check`); regenerable without KiCad, so it may
run in pre-commit.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import dataclasses
import io
import tempfile
import tomllib
import unittest
from unittest import mock
from pathlib import Path

from kicad import library, netlist, registry, testfixture
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT
from kicad.perfboard import __main__ as cli
from kicad.perfboard import config, place, wires

ROOT, PROJECT = testfixture.shared()
LIVE = REPO_ROOT / testfixture.PROJECT_REL
TOML = config.perfboard_dir(PROJECT) / config.FILENAME


def resolve(fpid: str) -> Path:
    return library.footprint_file(fpid, vendored_only=True)


class Inputs:
    """The frozen netlist and config, and a layout of any variant of the config."""

    nl = netlist.build(PROJECT, ROOT)
    cfg = config.load(TOML)
    specs = {c.registry: registry.load(c.registry, ROOT) for c in nl.comps}

    @classmethod
    def layout(cls, cfg=None):
        cfg = cfg or cls.cfg
        skip = cfg.skip | place.pinless(cls.nl)
        parts, raised, tall = place.build_parts(cls.nl, cfg, cls.specs, resolve, skip)
        return parts, raised, tall, place.nets_of(cls.nl, skip)


def moved(**placement):
    return dataclasses.replace(
        Inputs.cfg, placement={**Inputs.cfg.placement, **placement}
    )


def config_from(text: str) -> config.PerfboardConfig:
    return config.parse(tomllib.loads(text), Path("perfboard.toml"))


BASE = TOML.read_text(encoding="utf-8")


class ConfigTests(unittest.TestCase):
    def test_balancebot_config(self):
        cfg = Inputs.cfg
        self.assertEqual((cfg.cols, cfg.rows), (18, 24))
        self.assertEqual(len(cfg.placement), 15)
        self.assertEqual([s.title for s in cfg.steps][0], "Power")
        self.assertEqual(cfg.placement["U1"], (3, 3, 90))
        self.assertEqual(cfg.skip, frozenset())

    def test_wide_board_is_allowed(self):
        # Column names go on past Z (AA, AB, ...), so 27 columns is a board.
        cfg = config_from(BASE.replace("cols = 18", "cols = 27"))
        self.assertEqual(cfg.cols, 27)

    def test_rejects(self):
        nine = ", ".join(f'"N{i}"' for i in range(9))
        cases = {
            "nine nets in a step": (
                BASE.replace('nets = ["+3V3", "NENABLE"]', f"nets = [{nine}]"),
                r"has 9 nets; a step can colour at most 8",
            ),
            "a net in two steps": (
                BASE.replace('nets = ["+3V3", "NENABLE"]', 'nets = ["+3V3", "GND"]'),
                r"net 'GND' is already in step 'Power'",
            ),
            "unknown top-level key": (
                "colour = 1\n" + BASE,
                r"unknown key\(s\) \['colour'\]",
            ),
            "unknown [board] key": (
                BASE.replace("cols = 18", "cols = 18\nholes = 3"),
                r"\[board\]: unknown key",
            ),
            "zero rows": (
                BASE.replace("rows = 24", "rows = 0"),
                "rows must be a positive",
            ),
            "45 degree rotation": (
                BASE.replace("U1 = [3, 3, 90]", "U1 = [3, 3, 45]"),
                r"\[placement\] U1: expected \[column, row, rotation\]",
            ),
            "misspelt [wires]": (
                BASE.replace("[wires]", "[wirez]"),
                r"unknown key\(s\) \['wirez'\]",
            ),
            "unknown swap key": (
                BASE + '\n[swaps.J1]\ncolour = "red"\n',
                r"\[swaps.J1\]: unknown key",
            ),
            "skip not a list": ('skip = "H1"\n' + BASE, "skip must be a list"),
        }
        for name, (text, msg) in cases.items():
            with self.subTest(case=name), self.assertRaisesRegex(KicadError, msg):
                config_from(text)

    def test_missing_tables(self):
        for table in ("board", "wires", "placement"):
            with self.subTest(table=table):
                data = tomllib.loads(BASE)
                del data[table]
                with self.assertRaisesRegex(KicadError, rf"\[{table}\] is required"):
                    config.parse(data, Path("perfboard.toml"))

    def test_every_net_in_a_step(self):
        *_, nets = Inputs.layout()
        self.assertEqual(config.check_steps(Inputs.cfg, nets), [])
        dropped = dataclasses.replace(
            Inputs.cfg,
            steps=(
                *Inputs.cfg.steps[:-1],
                dataclasses.replace(Inputs.cfg.steps[-1], nets=("STEP_L", "BOGUS")),
            ),
        )
        (error,) = config.check_steps(dropped, nets)
        self.assertIn("missing ['DIR_L', 'DIR_R', 'IMU_INT'", error)
        self.assertIn("unknown ['BOGUS']", error)


class PlacementTests(unittest.TestCase):
    def test_balancebot_parts(self):
        parts, raised, tall, nets = Inputs.layout()
        self.assertEqual(len(parts), 15)
        self.assertNotIn("H1", parts)  # a mounting hole has no pins
        self.assertEqual(raised, {"U1", "A1", "A2", "J3"})
        self.assertEqual(tall, {"C1", "C2", "J2", "J4", "J5"})
        self.assertEqual(len(nets), 22)
        # Registry [perfboard]: a through-hole stand-in and its value.
        self.assertEqual(parts["D1"].value, "1N5819")
        self.assertTrue(parts["D1"].fpid.startswith("Diode_THT:"))
        self.assertEqual(parts["J1"].value, "XT30 pigtail")
        self.assertEqual(Inputs.cfg.placement["J3"][:2], parts["J3"].pads["1"])
        self.assertEqual(
            place.check_placement(parts, Inputs.cfg.board, raised, tall), []
        )

    def test_bodies_are_the_true_courtyard(self):
        # Hole units: the placement hole plus the F.CrtYd box / 2.54, rotated.
        parts, *_ = Inputs.layout()
        cases = {
            # CP_Radial_D8.0mm at (2, 19, 0): a circle of radius 4.25 about (1.25, 0).
            "C1": (2 - 3.0 / 2.54, 19 - 4.25 / 2.54, 2 + 5.5 / 2.54, 19 + 4.25 / 2.54),
            # D_DO-41 vertical at (10, 22, 0): (-1.6, -1.6, 3.89, 1.6) mm.
            "D1": (10 - 1.6 / 2.54, 22 - 1.6 / 2.54, 10 + 3.89 / 2.54, 22 + 1.6 / 2.54),
            # XIAO socket at (3, 3, 90): (x, y) -> (y, -x) on (-10.42, -8.88, 10.54, 8.87).
            "U1": (
                3 - 8.88 / 2.54,
                3 - 10.54 / 2.54,
                3 + 8.87 / 2.54,
                3 + 10.42 / 2.54,
            ),
        }
        for ref, box in cases.items():
            with self.subTest(ref=ref):
                for got, want in zip(parts[ref].body, box, strict=True):
                    self.assertAlmostEqual(got, want, places=9)

    def check(self, cfg):
        parts, raised, tall, _ = Inputs.layout(cfg)
        return place.check_placement(parts, cfg.board, raised, tall)

    def test_r1_onto_r2(self):
        errors = self.check(moved(R1=Inputs.cfg.placement["R2"]))
        self.assertIn("R2.1 and R1.1 share hole I13", errors)
        self.assertIn("R1 and R2 overlap on the board", errors)

    def test_c1_under_a1(self):
        errors = self.check(moved(C1=(3, 12, 0)))
        self.assertEqual(errors, ["tall part C1 sits under raised A1"])

    def test_raised_modules_overlap(self):
        errors = self.check(moved(A2=(3, 9, 0)))
        self.assertIn("raised modules A1 and A2 overlap", errors)

    def test_off_the_board(self):
        errors = self.check(moved(J5=(16, 23, 0)))
        self.assertIn("J5.4 at Q27 is off the board", errors)

    def test_placement_names_a_part_that_is_not_there(self):
        with self.assertRaisesRegex(KicadError, r"no netlist part to place: \['U9'\]"):
            Inputs.layout(moved(U9=(0, 0, 0)))

    def test_part_without_placement(self):
        cfg = dataclasses.replace(
            Inputs.cfg,
            placement={k: v for k, v in Inputs.cfg.placement.items() if k != "R3"},
        )
        with self.assertRaisesRegex(KicadError, r"\[placement\]: R3 has no entry"):
            Inputs.layout(cfg)

    def test_swap_overrides_the_registry(self):
        swap = config.Swap(
            "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical", "XT60"
        )
        cfg = dataclasses.replace(Inputs.cfg, swaps={"J1": config.Swap("", "XT60")})
        parts, *_ = Inputs.layout(cfg)
        self.assertEqual(parts["J1"].value, "XT60")
        self.assertTrue(parts["J1"].fpid.endswith("PinHeader_1x02_P2.54mm_Vertical"))
        parts, *_ = Inputs.layout(dataclasses.replace(Inputs.cfg, swaps={"J1": swap}))
        self.assertEqual(len(parts["J1"].pads), 4)

    def test_skip(self):
        cfg = dataclasses.replace(
            Inputs.cfg,
            skip=frozenset({"R3"}),
            placement={k: v for k, v in Inputs.cfg.placement.items() if k != "R3"},
        )
        parts, *_, nets = Inputs.layout(cfg)
        self.assertNotIn("R3", parts)
        self.assertNotIn("R3", {ref for nodes in nets.values() for ref, _ in nodes})

    def test_unknown_swap_or_skip(self):
        unknown = {
            "swap": dataclasses.replace(Inputs.cfg, swaps={"Q7": config.Swap("", "x")}),
            "skip": dataclasses.replace(Inputs.cfg, skip=frozenset({"Q8"})),
        }
        for kind, cfg in unknown.items():
            ref = "Q7" if kind == "swap" else "Q8"
            with (
                self.subTest(kind=kind),
                self.assertRaisesRegex(KicadError, rf"not in the netlist: \['{ref}'\]"),
            ):
                Inputs.layout(cfg)

    def test_swap_footprint_must_cover_the_netlist_pins(self):
        # J2 (the buck module) has 4 pins on nets; a 2-pin header cannot hold it.
        swap = config.Swap(
            "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical", ""
        )
        cfg = dataclasses.replace(Inputs.cfg, swaps={"J2": swap})
        with self.assertRaisesRegex(
            KicadError,
            r"perfboard\.toml \[swaps\.J2\]: footprint Connector_PinHeader_2\.54mm:"
            r"PinHeader_1x02_P2\.54mm_Vertical has no through-hole pad for J2's "
            r"netlist pin\(s\) \['3', '4'\]",
        ):
            Inputs.layout(cfg)

    def test_netlist_pins_include_no_connects(self):
        pins = place.netlist_pins(Inputs.nl)
        self.assertEqual(pins["J2"], {"1", "2", "3", "4"})
        for ref, pin in Inputs.nl.no_connect:
            with self.subTest(node=f"{ref}.{pin}"):
                self.assertIn(pin, pins[ref])


class WireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parts, _, _, cls.nets = Inputs.layout()
        cls.wires = wires.plan_wires(cls.parts, cls.nets)

    def test_plan_is_a_spanning_tree_per_net(self):
        self.assertEqual(len(self.wires), 54)
        self.assertEqual(len(self.wires), sum(len(n) - 1 for n in self.nets.values()))
        self.assertEqual(wires.check_wires(self.parts, self.nets, self.wires), [])

    def test_dropped_wire(self):
        planned = [w for w in self.wires if w[0] != "+3V3"]
        planned += [w for w in self.wires if w[0] == "+3V3"][1:]
        errors = wires.check_wires(self.parts, self.nets, planned)
        self.assertEqual(errors, ["net +3V3 is split into 2 groups"])

    def test_wire_end_on_another_net(self):
        i = next(i for i, w in enumerate(self.wires) if w[0] == "BUCK_5V")
        name, a, (_, label) = self.wires[i]
        other = self.parts["A1"].pads["10"]  # A1.M0, on +3V3
        planned = [*self.wires[:i], (name, a, (other, label)), *self.wires[i + 1 :]]
        errors = wires.check_wires(self.parts, self.nets, planned)
        self.assertTrue(
            any("are joined" in e and "BUCK_5V" in e for e in errors), errors
        )
        self.assertIn(f"wire end {label} lands on G16 owned by A1.10", errors)

    def test_numbering_follows_the_steps(self):
        numbered = wires.number_wires(self.nets, self.wires, Inputs.cfg.steps)
        self.assertEqual([w[0] for w in numbered], list(range(1, 55)))
        steps = [w[1] for w in numbered]
        self.assertEqual(steps, sorted(steps))
        first = numbered[0]
        self.assertEqual((first[2], first[3]), ("VBAT", "#2a78d6"))


class GenerateTests(unittest.TestCase):
    def test_heavy_class_must_exist(self):
        with tempfile.TemporaryDirectory() as tmp:
            project = testfixture.materialise(Path(tmp))
            toml = config.perfboard_dir(project) / config.FILENAME
            toml.write_text(
                toml.read_text().replace(
                    'heavy_class = "power"', 'heavy_class = "powr"'
                )
            )
            with self.assertRaisesRegex(KicadError, r"heavy_class 'powr' is not a"):
                cli.generate(project, Path(tmp))

    def test_write_then_check(self):
        # The CLI around generate(): it writes the outputs, --check then
        # passes, and an edited file fails it naming the project's recipe.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            # A module name that is not the directory name (#766).
            (root / "justfile").write_text("mod bb 'packages/robotics/balancebot'\n")
            out_dir = config.perfboard_dir(root / "packages/robotics/balancebot")
            out_dir.mkdir(parents=True)
            layout = cli.Layout(
                {
                    out_dir / "perfboard.svg": "<svg/>\n",
                    out_dir / "perfboard.md": "# list\n",
                },
                parts=1,
                wires=2,
                nets=3,
            )

            def run(*argv):
                out = io.StringIO()
                with (
                    mock.patch.object(cli, "generate", return_value=layout),
                    contextlib.redirect_stdout(out),
                ):
                    code = cli.main([*argv, str(out_dir.parent.parent)], repo_root=root)
                return code, out.getvalue()

            code, out = run("--check")
            self.assertEqual(code, 1)
            self.assertIn("stale: ['perfboard.svg', 'perfboard.md']", out)
            self.assertEqual(
                run(),
                (
                    0,
                    f"1 project(s): {out_dir.parent.parent.relative_to(root)}\n"
                    "wrote 1 parts, 2 wires, 3 nets connected\n",
                ),
            )
            self.assertEqual(run("--check")[0], 0)
            (out_dir / "perfboard.md").write_text("# list\nedited\n")
            code, out = run("--check")
            self.assertEqual(code, 1)
            self.assertIn(
                "packages/robotics/balancebot: stale: ['perfboard.md']; "
                "run just bb::perfboard",
                out,
            )


class CliErrorTests(unittest.TestCase):
    """A bad perfboard.toml is a FAIL line naming it repo-relative; exit 1, no traceback."""

    def run_cli(self, extra: str):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            project = testfixture.materialise(root)
            toml = config.perfboard_dir(project) / config.FILENAME
            toml.write_text(toml.read_text() + extra)
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                code = cli.main([str(project)], repo_root=root)
        return code, out.getvalue()

    def test_swap_footprint_short_of_pads(self):
        code, out = self.run_cli(
            "\n[swaps.J2]\nfootprint = "
            '"Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical"\n'
        )
        self.assertEqual(code, 1)
        self.assertIn(
            "FAIL: packages/robotics/balancebot/hardware/perfboard/perfboard.toml "
            "[swaps.J2]: footprint Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_"
            "Vertical has no through-hole pad for J2's netlist pin(s) ['3', '4']\n",
            out,
        )
        self.assertNotIn("Traceback", out)

    def test_config_error_names_the_file_repo_relative(self):
        code, out = self.run_cli("\n[swaps.J2]\ncolour = 1\n")
        self.assertEqual(code, 1)
        self.assertIn(
            "FAIL: packages/robotics/balancebot/hardware/perfboard/perfboard.toml "
            "[swaps.J2]: unknown key(s) ['colour']",
            out,
        )
        self.assertNotIn(tempfile.gettempdir(), out)


class LiveProjectTests(unittest.TestCase):
    """The committed balancebot perfboard files are the generator's output."""

    def test_committed_files_are_current(self):
        layout = cli.generate(LIVE)
        self.assertEqual((layout.parts, layout.wires, layout.nets), (19, 60, 25))
        for path, text in layout.outputs.items():
            with self.subTest(path=path.name):
                self.assertEqual(
                    path.read_text(encoding="utf-8"),
                    text,
                    "run `just balancebot::perfboard`",
                )


if __name__ == "__main__":
    unittest.main()
