"""Tests for tools/hardware/pinout.py — build-guide pinout images (#629).

Stdlib `unittest`, like the rest of tools/hardware:

    python3 -m unittest discover -s tools/hardware -t tools

Positions are asserted on the `<circle>` each pad is drawn as, read back
through the SVG's own `data-anchor`/`data-side`/`data-pos` attributes — the
same document Typst embeds, not a second description of it.
"""

from __future__ import annotations

import dataclasses
import io
import shutil
import tempfile
import textwrap
import unittest
import xml.etree.ElementTree as ET
from contextlib import redirect_stdout
from pathlib import Path

from hardware import HardwareError, join, parse_layout
from hardware.pinout import (
    OUT_DIR,
    Pinout,
    check_project,
    main,
    mcu_labels,
    part_labels,
    pinouts,
    regenerate,
    render_svg,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
UNIFIED = REPO_ROOT / "packages/robocar/unified"
SVG = "{http://www.w3.org/2000/svg}"

BREAKOUT_MD = """\
    # Test breakout <&>

    | Pin | Side | Pos | Header | Notes |
    |-----|------|-----|--------|-------|
    | VCC | L | 1 | JP1.1 | |
    | GND | L | 2 | JP1.2 | |
    | SDA | L | 3 | JP1.3 | |
    | SCL | R | 1 | JP2.1 | |
    | GND | R | 2 | JP2.2 | |
    | V+ | T | 1 | J1.1 | |
    | A<B | T | 2 | J1.2 | |
    | 0 | B | 1 | JP3.1 | |
    | 1 | B | 2 | JP3.2 | |
    | 2 | B | 3 | JP3.3 | |
    """


def write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(text), encoding="utf-8")
    return path


def circles(svg: str) -> dict[str, tuple[str, int, float, float]]:
    """anchor -> (side, pos, cx, cy) for every pad in the document."""
    root = ET.fromstring(svg)
    return {
        c.get("data-anchor"): (
            c.get("data-side"),
            int(c.get("data-pos")),
            float(c.get("cx")),
            float(c.get("cy")),
        )
        for c in root.iter(f"{SVG}circle")
    }


def texts(svg: str) -> list[str]:
    return [t.text or "" for t in ET.fromstring(svg).iter(f"{SVG}text")]


def board_rect(svg: str) -> tuple[float, float, float, float]:
    rects = [r for r in ET.fromstring(svg).iter(f"{SVG}rect") if r.get("rx")]
    (r,) = rects
    return tuple(float(r.get(k)) for k in ("x", "y", "width", "height"))  # type: ignore[return-value]


class RenderSvgTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.layout = parse_layout(write(Path(self.tmp.name) / "b.md", BREAKOUT_MD))

    def render(self, labels=None) -> str:
        return render_svg(Pinout("b", "Test <&>", self.layout, labels or {}), ("note",))

    def test_every_pad_is_drawn_once_with_its_side_and_position(self):
        drawn = circles(self.render())
        self.assertEqual(
            {a: (s, p) for a, (s, p, _, _) in drawn.items()},
            {p.anchor: (p.side, p.pos) for p in self.layout.pads},
        )

    def test_pads_sit_on_their_own_edge_of_the_outline(self):
        svg = self.render()
        x, y, w, h = board_rect(svg)
        for anchor, (side, _, cx, cy) in circles(svg).items():
            with self.subTest(anchor=anchor):
                self.assertTrue(x < cx < x + w and y < cy < y + h)
                nearest = min(
                    ("L", cx - x),
                    ("R", x + w - cx),
                    ("T", cy - y),
                    ("B", y + h - cy),
                    key=lambda e: e[1],
                )[0]
                self.assertEqual(nearest, side)

    def test_position_runs_top_to_bottom_and_left_to_right(self):
        # The convention in layout.py: Pos 1 at the top of L/R, the left of T/B.
        drawn = circles(self.render())
        for side, axis in (("L", 3), ("R", 3), ("T", 2), ("B", 2)):
            with self.subTest(side=side):
                coords = [
                    v[axis]
                    for v in sorted(
                        (v for v in drawn.values() if v[0] == side), key=lambda v: v[1]
                    )
                ]
                self.assertEqual(coords, sorted(coords))
                self.assertEqual(len(set(coords)), len(coords))

    def test_names_and_labels_are_printed_and_escaped(self):
        svg = self.render({"SDA": "MCU D4 · I2C_SDA"})
        printed = texts(svg)
        for pad in self.layout.pads:
            self.assertIn(pad.name, printed)
        self.assertIn("A<B", printed)  # parsed back, so it was escaped
        self.assertIn("Test <&>", printed)
        self.assertIn("MCU D4 · I2C_SDA", printed)

    def test_a_labelled_pad_is_coloured_apart_from_the_rest(self):
        root = ET.fromstring(self.render({"SDA": "x"}))
        fills = {c.get("data-anchor"): c.get("fill") for c in root.iter(f"{SVG}circle")}
        self.assertNotEqual(fills["SDA"], fills["SCL"])
        self.assertEqual(fills["SCL"], fills["VCC"])

    def test_a_label_for_a_pad_the_board_lacks_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, "not pads of this board"):
            self.render({"MISO": "x"})

    def test_output_is_deterministic(self):
        self.assertEqual(self.render({"SDA": "x"}), self.render({"SDA": "x"}))

    def test_printed_size_is_proportional_to_the_drawing(self):
        root = ET.fromstring(self.render())
        _, _, vw, vh = (float(v) for v in root.get("viewBox").split())
        self.assertAlmostEqual(
            float(root.get("width").removesuffix("mm")), vw * 0.25, 1
        )
        self.assertAlmostEqual(
            float(root.get("height").removesuffix("mm")), vh * 0.25, 1
        )


class LabelsFromJoinTest(unittest.TestCase):
    """Labels for robocar-unified, against its committed hardware.toml."""

    @classmethod
    def setUpClass(cls):
        cls.model = join(UNIFIED)

    def test_mcu_pads_carry_gpio_role_and_destination(self):
        layout = parse_layout(self.model.board.path)
        labels = mcu_labels(self.model, layout)
        sda = self.model.pin_for("I2C_SDA_PIN")
        gpio = self.model.roles["I2C_SDA_PIN"]
        self.assertEqual(labels[sda.name], f"GPIO{gpio} · I2C_SDA → TCA9548A SDA")
        tx = self.model.pin_for("UART0_TX_PIN")
        self.assertIn("UART0_TX (not wired)", labels[tx.name])
        # A power pad has no GPIO and so no label.
        self.assertNotIn("5V", labels)

    def test_a_breakout_pad_names_the_mcu_pad_or_channel_driving_it(self):
        # #666: the control row comes from the PCA9685, STBY from the XIAO.
        path = REPO_ROOT / self.model.parts["motor_driver"].board
        labels = part_labels(self.model, "motor_driver", parse_layout(path))
        stby = self.model.pin_for("MOTOR_STBY_PIN")
        ch = self.model.channels

        def driven(role: str) -> str:
            return f"PCA9685 ch{ch[role + '_CHANNEL']} · {role}"

        self.assertEqual(
            labels,
            {
                "STBY": f"MCU {stby.name} · MOTOR_STBY",
                "PWMA": driven("MOTOR_RIGHT_PWM"),
                "AIN2": driven("MOTOR_RIGHT_IN2"),
                "AIN1": driven("MOTOR_RIGHT_IN1"),
                "BIN1": driven("MOTOR_LEFT_IN1"),
                "BIN2": driven("MOTOR_LEFT_IN2"),
                "PWMB": driven("MOTOR_LEFT_PWM"),
            },
        )

    def test_every_pca9685_channel_the_firmware_drives_is_labelled(self):
        path = REPO_ROOT / self.model.parts["pwm"].board
        labels = part_labels(self.model, "pwm", parse_layout(path))
        ch = self.model.channels
        self.assertEqual(set(labels), {str(n) for n in ch.values()})
        self.assertEqual(
            labels[str(ch["MOTOR_RIGHT_PWM_CHANNEL"])],
            "MOTOR_RIGHT_PWM → TB6612FNG PWMA",
        )
        self.assertEqual(
            labels[str(ch["SERVO_PAN_CHANNEL"])], "SERVO_PAN → SG90 servos PAN"
        )
        self.assertEqual(
            labels[str(ch["LED_LEFT_R_CHANNEL"])], "LED_LEFT_R → Left RGB LED R"
        )

    def test_a_fanned_out_channel_names_every_pin_it_reaches(self):
        path = REPO_ROOT / self.model.parts["pwm"].board
        model = self.model
        (red,) = [n for n in model.channel_nets if n.role == "LED_LEFT_R_CHANNEL"]
        both = dataclasses.replace(red, part="led_right", pin="R")
        fanned = dataclasses.replace(model, channel_nets=(*model.channel_nets, both))
        labels = part_labels(fanned, "pwm", parse_layout(path))
        self.assertEqual(
            labels[str(red.channel)], "LED_LEFT_R → Left RGB LED R, Right RGB LED R"
        )

    def test_a_channel_net_to_a_pin_the_board_lacks_is_an_error(self):
        path = REPO_ROOT / self.model.parts["motor_driver"].board
        layout = parse_layout(path)
        model = self.model
        bad = dataclasses.replace(
            model.channel_nets[0], part="motor_driver", pin="PWMC"
        )
        broken = dataclasses.replace(model, channel_nets=(*model.channel_nets, bad))
        with self.assertRaisesRegex(HardwareError, "no pad named 'PWMC'"):
            part_labels(broken, "motor_driver", layout)

    def test_a_net_to_a_pin_the_board_lacks_is_an_error(self):
        path = REPO_ROOT / self.model.parts["amp"].board
        layout = parse_layout(path)
        model = self.model
        bad = type(model.nets[0])(role="I2S_DIN_PIN", part="amp", pin="DATA", note="")
        broken = type(model)(**{**model.__dict__, "nets": (*model.nets, bad)})
        with self.assertRaisesRegex(HardwareError, "no pad named 'DATA'"):
            part_labels(broken, "amp", layout)

    def test_a_net_to_a_repeated_pad_name_is_an_error(self):
        path = REPO_ROOT / self.model.parts["motor_driver"].board
        layout = parse_layout(path)
        model = self.model
        bad = type(model.nets[0])(
            role="PIEZO_PIN", part="motor_driver", pin="GND", note=""
        )
        broken = type(model)(**{**model.__dict__, "nets": (*model.nets, bad)})
        with self.assertRaisesRegex(HardwareError, "3 pads named 'GND'"):
            part_labels(broken, "motor_driver", layout)

    def test_the_mcu_and_every_part_with_a_board_are_drawn(self):
        slugs = [p.slug for p in pinouts(self.model)]
        expected = [self.model.board.path.stem] + [
            Path(p.board).stem for p in self.model.parts.values() if p.board
        ]
        self.assertEqual(slugs, expected)
        self.assertIn("xiao-esp32s3", slugs)


class CommittedImagesTest(unittest.TestCase):
    def copy_project(self) -> Path:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        dest = Path(tmp.name) / "unified"
        shutil.copytree(
            UNIFIED, dest, ignore=shutil.ignore_patterns("build*", "managed_components")
        )
        return dest

    def test_committed_images_are_up_to_date(self):
        # The build guide embeds these; a board reference or hardware.toml
        # change that skipped `just hardware::gen` fails here, not in print.
        self.assertEqual(check_project(UNIFIED), [])

    def test_a_board_reference_change_makes_the_image_stale(self):
        proj = self.copy_project()
        with tempfile.TemporaryDirectory() as tmp:
            boards = Path(tmp) / "docs/reference/boards"
            shutil.copytree(REPO_ROOT / "docs/reference/boards", boards)
            page = boards / "sparkfun-tb6612fng.md"
            page.write_text(
                page.read_text(encoding="utf-8")
                .replace("| AIN2 | R | 2 |", "| AIN2 | R | 3 |")
                .replace("| AIN1 | R | 3 |", "| AIN1 | R | 2 |"),
                encoding="utf-8",
            )
            problems = check_project(proj, repo_root=Path(tmp))
        self.assertEqual(
            problems, [f"STALE: {proj / OUT_DIR / 'sparkfun-tb6612fng.svg'}"]
        )

    def test_write_removes_an_orphan_and_check_reports_it(self):
        proj = self.copy_project()
        orphan = proj / OUT_DIR / "retired-board.svg"
        orphan.write_text("<svg/>", encoding="utf-8")
        self.assertIn(
            f"ORPHANED: {orphan} (no board in hardware.toml produces it)",
            check_project(proj),
        )
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main([str(proj)]), 0)
        self.assertFalse(orphan.exists())
        self.assertEqual(check_project(proj), [])

    def test_check_mode_exit_status(self):
        proj = self.copy_project()
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(["--check", str(proj)]), 0)
        (proj / OUT_DIR / "xiao-esp32s3.svg").unlink()
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(main(["--check", str(proj)]), 1)
        self.assertIn("MISSING:", out.getvalue())

    def test_regenerate_writes_nothing(self):
        proj = self.copy_project()
        shutil.rmtree(proj / OUT_DIR)
        regenerate(proj)
        self.assertFalse((proj / OUT_DIR).exists())


if __name__ == "__main__":
    unittest.main()
