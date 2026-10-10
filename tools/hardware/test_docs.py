"""Tests for tools/hardware/docs.py — Markdown emitted from the join (#461).

python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import io
import os
import shutil
import subprocess
import tempfile
import textwrap
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

from hardware import HardwareError, join
from hardware.docs import (
    BEGIN,
    END,
    NOTICE,
    check_project,
    inject,
    main,
    pin_table,
    power_diagram,
    render_block,
    signals_table,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
UNIFIED = REPO_ROOT / "packages/robocar/unified"


def git_init(path: Path) -> None:
    """The scan reads `git ls-files`; untracked files count, so no commit is needed."""
    subprocess.run(["git", "init", "-q", str(path)], check=True)


def write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text))
    return path


BOARD_MD = """\
    | Board Pin | GPIO | Side | Pos | Default Function | Alternate Functions |
    |-----------|------|------|-----|------------------|---------------------|
    | D0 | GPIO1 | L | 1 | Analog input | ADC1_CH0 |
    | D10 | GPIO9 | L | 2 | SPI MOSI | — |
    | D2 | GPIO3 | L | 3 | UART TX | — |
    | 5V | — | R | 1 | 5 V rail | — |
    | D1 | GPIO7 | R | 2 | SPI SCK | — |
    | D3 | GPIO8 | R | 3 | SPI MISO | — |
    """

HEADER_H = """\
    #define SDA_PIN GPIO_NUM_1
    #define BEEP_PIN GPIO_NUM_7
    #define TX_PIN GPIO_NUM_3
    #define MIC_PIN GPIO_NUM_41
    #define FAN_PIN GPIO_NUM_9
    """

SIDECAR = """\
    [source]
    convention = "robocar"
    header = "main/pin_config.h"
    board = "board.md"

    [parts.mux]
    name = "TCA9548A"
    kind = "i2c-mux"

    [parts.oled]
    name = "SSD1306"
    kind = "display"

    [parts.buzzer]
    name = "Piezo"
    kind = "piezo"

    [parts.fan]
    name = "Fan | 5 V"
    kind = "fan"

    [[nets]]
    role = "SDA_PIN"
    to = "mux.SDA"
    note = "bus data"

    [[nets]]
    role = "SDA_PIN"
    to = "oled.SDA"
    note = "same bus"

    [[nets]]
    role = "BEEP_PIN"
    to = "buzzer.+"

    [[nets]]
    role = "MIC_PIN"
    to = "fan.SENSE"
    note = "off the header"

    [[undrawn]]
    role = "TX_PIN"
    why = "reserved for UART0"

    [[undrawn]]
    role = "FAN_PIN"
    why = "spare"
    """


class Fixture(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        write(self.root / "board.md", BOARD_MD)
        self.proj = self.root / "proj"
        write(self.proj / "main/pin_config.h", HEADER_H)
        write(self.proj / "hardware.toml", SIDECAR)

    def model(self):
        return join(self.proj, repo_root=self.root)


class PinTableTest(Fixture):
    def rows(self) -> list[str]:
        return [
            ln for ln in pin_table(self.model()).splitlines() if ln.startswith("| D")
        ]

    def test_rows_are_in_silkscreen_order_not_board_order(self):
        # D10 sorts after D3 numerically; the board table lists it second.
        self.assertEqual(
            [r.split("|")[1].strip() for r in self.rows()],
            ["D0", "D1", "D2", "D3", "D10"],
        )

    def test_a_fan_out_lists_every_endpoint_on_one_row(self):
        self.assertEqual(
            self.rows()[0],
            "| D0 | GPIO1 | `SDA_PIN` | TCA9548A SDA, SSD1306 SDA | bus data; same bus |",
        )

    def test_a_net_without_a_note_leaves_the_notes_cell_empty(self):
        self.assertEqual(self.rows()[1], "| D1 | GPIO7 | `BEEP_PIN` | Piezo + |  |")

    def test_an_undrawn_header_pin_carries_its_reason(self):
        self.assertEqual(
            self.rows()[2], "| D2 | GPIO3 | `TX_PIN` | — | reserved for UART0 |"
        )

    def test_a_header_gpio_with_no_role_is_shown_as_unassigned(self):
        # Silently dropping it would read as "this board has no such pad".
        self.assertEqual(self.rows()[3], "| D3 | GPIO8 | — | *unassigned* |  |")

    def test_an_undrawn_role_on_a_header_pad_carries_its_reason(self):
        self.assertEqual(self.rows()[4], "| D10 | GPIO9 | `FAN_PIN` | — | spare |")

    def test_roles_off_the_header_and_power_pads_are_left_out(self):
        table = pin_table(self.model())
        self.assertNotIn("MIC_PIN", table)
        self.assertNotIn("5V", table)

    def test_header_row_and_separator(self):
        lines = pin_table(self.model()).splitlines()
        self.assertEqual(lines[0], "| Pin | GPIO | Macro | Wired to | Notes |")
        self.assertEqual(lines[1], "|-----|------|-------|----------|-------|")


class SignalsTableTest(Fixture):
    def test_rows_follow_the_sidecar_net_order_for_that_part(self):
        self.assertEqual(
            signals_table(self.model(), "mux").splitlines(),
            [
                "| Signal | Pin | Function |",
                "|--------|-----|----------|",
                "| SDA | GPIO1 (D0) | bus data |",
            ],
        )

    def test_a_pin_off_the_header_shows_the_gpio_alone(self):
        self.assertIn(
            "| SENSE | GPIO41 | off the header |", signals_table(self.model(), "fan")
        )

    def test_a_pipe_in_a_cell_is_escaped(self):
        # Move the fan's net onto a header pin so its part name reaches the table.
        write(
            self.proj / "hardware.toml",
            SIDECAR.replace('role = "MIC_PIN"', 'role = "TMP"')
            .replace('role = "FAN_PIN"', 'role = "MIC_PIN"')
            .replace('role = "TMP"', 'role = "FAN_PIN"'),
        )
        self.assertIn("| Fan \\| 5 V SENSE |", pin_table(self.model()))

    def test_an_undeclared_part_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, "'nosuch' is not declared"):
            signals_table(self.model(), "nosuch")

    def test_a_part_with_no_nets_is_an_error(self):
        write(
            self.proj / "hardware.toml",
            SIDECAR + '\n[parts.unused]\nname = "X"\nkind = "y"\n',
        )
        with self.assertRaisesRegex(HardwareError, "'unused' has no nets"):
            signals_table(self.model(), "unused")


POWER = """
[parts.reg]
name = "Regulator"
kind = "regulator"
note = "set to 5.0 V"

[parts.blade]
name = "Blade"
kind = "load"

[parts.spare]
name = "Spare"
kind = "nothing"

[[rails]]
name = "5V"
from = "reg.OUT"
to = ["mcu.5V", "fan.V+"]

[[outputs]]
from = "fan"
to = ["blade"]
"""


class PowerDiagramTest(Fixture):
    def setUp(self):
        super().setUp()
        write(self.proj / "hardware.toml", SIDECAR + POWER)

    def diagram(self) -> str:
        return power_diagram(self.model())

    def lines(self) -> list[str]:
        return [ln.strip() for ln in self.diagram().splitlines()]

    def test_is_a_fenced_top_down_mermaid_graph(self):
        text = self.diagram()
        self.assertTrue(text.startswith("```mermaid\ngraph TD\n"), text)
        self.assertTrue(text.endswith("```\n"), text)

    def test_each_rail_load_is_a_solid_edge_naming_the_rail_and_pin(self):
        self.assertIn('reg -->|"5V → 5V"| mcu', self.lines())
        self.assertIn('reg -->|"5V → V+"| fan', self.lines())

    def test_each_net_is_a_dotted_edge_with_its_gpio_and_pad(self):
        lines = self.lines()
        self.assertIn('mcu -.->|"GPIO1 (D0) → SDA"| mux', lines)
        self.assertIn('mcu -.->|"GPIO1 (D0) → SDA"| oled', lines)
        self.assertIn('mcu -.->|"GPIO7 (D1) → +"| buzzer', lines)

    def test_a_net_off_the_header_shows_the_gpio_alone(self):
        self.assertIn('mcu -.->|"GPIO41 → SENSE"| fan', self.lines())

    def test_an_output_is_an_unlabelled_edge(self):
        self.assertIn("fan --> blade", self.lines())

    def test_every_node_is_declared_once_with_its_name_and_note(self):
        lines = self.lines()
        self.assertIn('reg["Regulator<br/>set to 5.0 V"]', lines)
        self.assertIn('mcu["MCU"]', lines)
        declared = [ln.split("[", 1)[0] for ln in lines if ln.endswith('"]')]
        self.assertEqual(len(declared), len(set(declared)), declared)
        self.assertEqual(
            set(declared), {"reg", "mcu", "fan", "mux", "oled", "buzzer", "blade"}
        )

    def test_a_part_on_no_rail_net_or_output_is_left_out(self):
        self.assertNotIn("spare", self.diagram())

    def test_a_pipe_or_quote_in_a_name_is_escaped(self):
        self.assertIn('fan["Fan #124; 5 V"]', self.lines())
        write(
            self.proj / "hardware.toml",
            (SIDECAR + POWER).replace(
                'board = "board.md"', 'board = "board.md"\nmcu = \'Dev "B"\''
            ),
        )
        self.assertIn('mcu["Dev #quot;B#quot;"]', self.lines())

    def test_a_part_id_mermaid_reserves_is_an_error(self):
        # `end` closes a Mermaid block; as a node id it fails the whole chart,
        # and the string-only --check would stay green.
        write(
            self.proj / "hardware.toml",
            (SIDECAR + POWER)
            .replace("[parts.blade]", "[parts.end]")
            .replace('to = ["blade"]', 'to = ["end"]'),
        )
        with self.assertRaisesRegex(HardwareError, "'end'.*Mermaid"):
            self.diagram()

    def test_the_mcu_node_takes_the_source_name(self):
        write(
            self.proj / "hardware.toml",
            (SIDECAR + POWER).replace(
                'board = "board.md"', 'board = "board.md"\nmcu = "Dev board"'
            ),
        )
        self.assertIn('mcu["Dev board"]', self.lines())


class RenderBlockTest(Fixture):
    def test_known_blocks(self):
        model = self.model()
        self.assertIn("| Pin | GPIO |", render_block(model, "pin-table"))
        self.assertIn("| SDA | GPIO1 (D0) |", render_block(model, "signals:mux"))
        self.assertIn("```mermaid", render_block(model, "power-diagram"))

    def test_an_unknown_block_is_an_error(self):
        with self.assertRaisesRegex(
            HardwareError, "unknown generated block 'pin-tabel'"
        ):
            render_block(self.model(), "pin-tabel")


def doc(*body: str) -> str:
    return "\n".join(body) + "\n"


class InjectTest(unittest.TestCase):
    def render(self, name: str) -> str:
        return f"| {name} |\n"

    def test_replaces_only_the_block_body(self):
        text = doc(
            "# Title",
            "",
            "prose stays",
            f"{BEGIN('t')}",
            "| stale |",
            END,
            "",
            "tail prose",
        )
        out = inject(text, self.render, "x.md")
        self.assertEqual(
            out,
            doc(
                "# Title",
                "",
                "prose stays",
                BEGIN("t"),
                NOTICE,
                "",
                "| t |",
                "",
                END,
                "",
                "tail prose",
            ),
        )

    def test_is_idempotent(self):
        text = doc(BEGIN("a"), END, "between", BEGIN("b"), "junk", END)
        once = inject(text, self.render, "x.md")
        self.assertEqual(inject(once, self.render, "x.md"), once)

    def test_text_without_markers_is_unchanged(self):
        text = doc("just prose", "<!-- an ordinary comment -->")
        self.assertEqual(inject(text, self.render, "x.md"), text)

    def test_a_begin_without_an_end_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, r"x.md:1: .*'t' is never closed"):
            inject(doc(BEGIN("t"), "| row |"), self.render, "x.md")

    def test_a_stray_end_is_an_error(self):
        with self.assertRaisesRegex(
            HardwareError, r"x.md:2: END GENERATED with no open block"
        ):
            inject(doc("prose", END), self.render, "x.md")

    def test_a_nested_begin_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, r"x.md:2: .*inside block 'a'"):
            inject(doc(BEGIN("a"), BEGIN("b"), END, END), self.render, "x.md")

    def test_a_malformed_marker_is_an_error_not_prose(self):
        # A typo would otherwise leave a hand-edited table that no check reads.
        with self.assertRaisesRegex(HardwareError, r"x.md:1: malformed marker"):
            inject(doc("<!-- BEGIN GENERATED pin-table -->", END), self.render, "x.md")


class DocsDiscoveryTest(Fixture):
    """#653: a block in a nested Markdown file is checked, not silently skipped."""

    STALE = doc("# Nested", "", BEGIN("signals:buzzer"), "stale body", END)

    def setUp(self):
        super().setUp()
        git_init(self.root)

    def drifted(self) -> list[str]:
        return [
            p.relative_to(self.proj).as_posix()
            for p, _ in check_project(self.proj, repo_root=self.root)
        ]

    def test_a_block_in_a_nested_file_is_checked(self):
        write(self.proj / "docs/deep/notes.md", self.STALE)
        self.assertEqual(self.drifted(), ["docs/deep/notes.md"])

    def test_a_top_level_file_is_still_checked(self):
        write(self.proj / "WIRING.md", self.STALE)
        self.assertEqual(self.drifted(), ["WIRING.md"])

    def test_a_staged_file_is_checked(self):
        # Unmodified tracked files are what CI sees, and --others does not list
        # them: without --cached the real WIRING.md would go unscanned.
        write(self.proj / "WIRING.md", self.STALE)
        write(self.proj / "docs/deep/notes.md", self.STALE)
        subprocess.run(["git", "add", "-A"], cwd=self.root, check=True)
        self.assertEqual(self.drifted(), ["WIRING.md", "docs/deep/notes.md"])

    def test_a_tracked_file_deleted_from_the_work_tree_is_skipped(self):
        notes = write(self.proj / "docs/notes.md", self.STALE)
        subprocess.run(["git", "add", "-A"], cwd=self.root, check=True)
        notes.unlink()
        self.assertEqual(self.drifted(), [])

    def test_a_gitignored_file_is_not_scanned(self):
        # build/ and managed_components/ hold vendored Markdown nobody edits.
        write(self.proj / ".gitignore", "build/\n")
        write(self.proj / "build/stale.md", self.STALE)
        self.assertEqual(self.drifted(), [])

    def test_a_nested_project_with_its_own_sidecar_is_left_to_itself(self):
        # Its blocks render against its own hardware.toml, not this one's.
        write(self.proj / "child/hardware.toml", SIDECAR)
        write(self.proj / "child/docs/notes.md", self.STALE)
        self.assertEqual(self.drifted(), [])

    def test_regenerating_fixes_the_nested_file(self):
        notes = write(self.proj / "docs/notes.md", self.STALE)
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main([str(self.proj)], repo_root=self.root), 0)
        self.assertEqual(self.drifted(), [])
        self.assertIn("| + | GPIO7 (D1) |", notes.read_text(encoding="utf-8"))

    def test_a_project_outside_a_git_work_tree_is_an_error(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        bare = Path(tmp.name) / "proj"
        shutil.copytree(self.proj, bare)
        write(Path(tmp.name) / "board.md", BOARD_MD)
        # Stop git's upward search at the temp dir, so a TMPDIR that happens to
        # sit inside some work tree cannot make the scan succeed.
        ceiling = mock.patch.dict(os.environ, {"GIT_CEILING_DIRECTORIES": tmp.name})
        ceiling.start()
        self.addCleanup(ceiling.stop)
        with self.assertRaisesRegex(HardwareError, "not inside a git work tree"):
            check_project(bare, repo_root=Path(tmp.name))


class RobocarUnifiedDocsTest(unittest.TestCase):
    """The acceptance in #461: green on the committed tree, red after a hand edit."""

    def copy_project(self) -> Path:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        proj = Path(tmp.name) / "unified"
        (proj / "main").mkdir(parents=True)
        shutil.copy(UNIFIED / "hardware.toml", proj)
        for name in ("pin_config.h", "planner_task.h", "plan_activity.h"):
            shutil.copy(UNIFIED / "main" / name, proj / "main")
        for name in ("WIRING.md", "README.md"):
            shutil.copy(UNIFIED / name, proj)
        git_init(proj)
        return proj

    def test_the_committed_docs_are_up_to_date(self):
        self.assertEqual(check_project(UNIFIED), [])

    def test_wiring_carries_every_expected_block(self):
        text = (UNIFIED / "WIRING.md").read_text(encoding="utf-8")
        for name in ("pin-table", "signals:amp", "signals:ranger", "power-diagram"):
            self.assertIn(BEGIN(name), text)

    def test_the_power_diagram_carries_no_hand_typed_gpio(self):
        # #646: the diagram's GPIO labels come from the join, grouped per part.
        text = (UNIFIED / "WIRING.md").read_text(encoding="utf-8")
        self.assertIn(
            'mcu -.->|"GPIO7 (D8) → BCLK<br/>GPIO8 (D9) → LRC<br/>GPIO9 (D10) → DIN"| amp',
            text,
        )
        self.assertIn('mcu -.->|"GPIO1 (D0) → STBY"| motor_driver', text)
        for stale in ("|GPIO2|", "|GPIO1|", "|GPIO7/8/9 I2S|"):
            self.assertNotIn(stale, text)

    def test_a_hand_edit_inside_a_block_is_drift(self):
        proj = self.copy_project()
        wiring = proj / "WIRING.md"
        text = wiring.read_text(encoding="utf-8")
        self.assertIn("| D4 | GPIO5 |", text)
        wiring.write_text(
            text.replace("| D4 | GPIO5 |", "| D4 | GPIO15 |", 1), encoding="utf-8"
        )
        drift = check_project(proj, repo_root=REPO_ROOT)
        self.assertEqual([p.name for p, _ in drift], ["WIRING.md"])
        self.assertIn("GPIO15", drift[0][1])

    def test_a_hand_edit_outside_every_block_is_not_drift(self):
        proj = self.copy_project()
        wiring = proj / "WIRING.md"
        wiring.write_text(
            wiring.read_text(encoding="utf-8") + "\nhand prose\n", encoding="utf-8"
        )
        self.assertEqual(check_project(proj, repo_root=REPO_ROOT), [])

    def test_a_pin_change_in_the_header_is_drift_until_regenerated(self):
        proj = self.copy_project()
        header = proj / "main/pin_config.h"
        text = header.read_text(encoding="utf-8")
        # Swap the two ultrasonic pins: both stay on header pads, so the join
        # still resolves and only the documents disagree.
        text = text.replace(
            "ULTRASONIC_TRIG_PIN GPIO_NUM_3", "ULTRASONIC_TRIG_PIN GPIO_NUM_X"
        )
        text = text.replace(
            "ULTRASONIC_ECHO_PIN GPIO_NUM_4", "ULTRASONIC_ECHO_PIN GPIO_NUM_3"
        )
        text = text.replace(
            "ULTRASONIC_TRIG_PIN GPIO_NUM_X", "ULTRASONIC_TRIG_PIN GPIO_NUM_4"
        )
        header.write_text(text, encoding="utf-8")
        self.assertEqual(
            [p.name for p, _ in check_project(proj, repo_root=REPO_ROOT)], ["WIRING.md"]
        )
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main([str(proj)], repo_root=REPO_ROOT), 0)
        self.assertEqual(check_project(proj, repo_root=REPO_ROOT), [])
        self.assertIn(
            "| TRIG | GPIO4 (D3) |", (proj / "WIRING.md").read_text(encoding="utf-8")
        )

    def test_a_pin_change_reaches_the_power_diagram(self):
        # The hand-typed `XIAO -->|GPIO1| MD` edge this replaced (#646) would
        # have kept saying GPIO1 after this swap.
        proj = self.copy_project()
        header = proj / "main/pin_config.h"
        text = header.read_text(encoding="utf-8")
        for old, new in (
            ("MOTOR_STBY_PIN GPIO_NUM_1", "MOTOR_STBY_PIN GPIO_NUM_X"),
            ("PIEZO_PIN GPIO_NUM_2", "PIEZO_PIN GPIO_NUM_1"),
            ("MOTOR_STBY_PIN GPIO_NUM_X", "MOTOR_STBY_PIN GPIO_NUM_2"),
        ):
            self.assertIn(old, text)
            text = text.replace(old, new)
        header.write_text(text, encoding="utf-8")
        (drift,) = check_project(proj, repo_root=REPO_ROOT)
        self.assertIn('+    mcu -.->|"GPIO2 (D1) → STBY"| motor_driver', drift[1])

    def test_check_mode_exit_status(self):
        proj = self.copy_project()
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--check", str(proj)], repo_root=REPO_ROOT), 0)
        wiring = proj / "WIRING.md"
        wiring.write_text(
            wiring.read_text(encoding="utf-8").replace(
                "| D4 | GPIO5 |", "| D4 | GPIO15 |", 1
            ),
            encoding="utf-8",
        )
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(main(["--check", str(proj)], repo_root=REPO_ROOT), 1)
        self.assertIn("WIRING.md", out.getvalue())


if __name__ == "__main__":
    unittest.main()
