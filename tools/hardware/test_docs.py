"""Tests for tools/hardware/docs.py — Markdown emitted from the join (#461).

python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import io
import shutil
import tempfile
import textwrap
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from hardware import HardwareError, join
from hardware.docs import (
    BEGIN,
    END,
    NOTICE,
    check_project,
    inject,
    main,
    pin_table,
    render_block,
    signals_table,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
UNIFIED = REPO_ROOT / "packages/robocar/unified"


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

    def test_a_role_on_a_header_pad_but_on_no_net_or_undrawn_is_still_listed(self):
        self.assertEqual(self.rows()[4], "| D10 | GPIO9 | `FAN_PIN` | — |  |")

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
            SIDECAR.replace('role = "MIC_PIN"', 'role = "FAN_PIN"'),
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


class RenderBlockTest(Fixture):
    def test_known_blocks(self):
        model = self.model()
        self.assertIn("| Pin | GPIO |", render_block(model, "pin-table"))
        self.assertIn("| SDA | GPIO1 (D0) |", render_block(model, "signals:mux"))

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
        return proj

    def test_the_committed_docs_are_up_to_date(self):
        self.assertEqual(check_project(UNIFIED), [])

    def test_wiring_carries_every_expected_block(self):
        text = (UNIFIED / "WIRING.md").read_text(encoding="utf-8")
        for name in ("pin-table", "signals:amp", "signals:ranger"):
            self.assertIn(BEGIN(name), text)

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
