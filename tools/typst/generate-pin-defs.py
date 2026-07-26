#!/usr/bin/env python3
"""Generate Typst `#let` bindings from a C pin_config.h and version.txt.

Usage:
    python3 tools/typst/generate-pin-defs.py \
        packages/robocar/unified/main/pin_config.h \
        packages/robocar/unified/version.txt \
        packages/robocar/unified/docs/auto/pin_defs.typ

The output is a Typst file with #let bindings for every relevant constant.
Run it before `typst compile` to keep docs in sync with the C source of truth.
"""

import re
import sys
from pathlib import Path

# The output is committed and diffed by the build-guide drift guard, so it must
# be byte-identical no matter where the generator is invoked from — the justfile
# recipe runs it from the project directory, CI runs it from the repo root.
# Source paths in the header are therefore anchored to the repo root rather than
# to the current working directory.
REPO_ROOT = Path(__file__).resolve().parents[2]


# ── Extract list: (category_heading, c_macro_name, typst_name, transform) ──
# category_heading: section header in the output (None = no header)
# c_macro_name: the `#define` name in the C header
# typst_name: the Typst `#let` binding name
# transform: None (auto-detect), "hex_str" (force hex literal → string),
#            "int" (force decimal)
#
# Formatting is fenced off below because this is a lookup table: the column
# alignment and the blank line between hardware groups are what make a missing
# or misfiled entry visible at a glance. `ruff format` collapses both.
# fmt: off
EXTRACT = [
    (None, None, None, None),  # placeholder for version (handled separately)

    # ── I2C Bus ──
    ("I2C Bus", "I2C_SDA_PIN",        "I2C_SDA_PIN",        None),
    ("I2C Bus", "I2C_SCL_PIN",        "I2C_SCL_PIN",        None),
    ("I2C Bus", "I2C_MASTER_FREQ_HZ", "I2C_FREQ_HZ",        None),

    # ── TCA9548A Multiplexer ──
    ("TCA9548A Multiplexer", "TCA9548A_ADDR",             "TCA9548A_ADDR",  "hex_str"),
    ("TCA9548A Multiplexer", "I2C_BUS_CHANNEL_PCA9685",   "TCA_CH_PCA9685", None),
    ("TCA9548A Multiplexer", "I2C_BUS_CHANNEL_OLED",      "TCA_CH_OLED",    None),
    ("TCA9548A Multiplexer", "I2C_BUS_CHANNEL_MCP23017",  "TCA_CH_MCP23017", None),

    # ── PCA9685 PWM Driver ──
    ("PCA9685 PWM Driver", "PCA9685_ADDR",    "PCA9685_ADDR",    "hex_str"),
    ("PCA9685 PWM Driver", "PCA9685_FREQ_HZ", "PCA9685_FREQ_HZ", None),

    # ── PCA9685 — LED Channels ──
    ("PCA9685 — LED Channels", "LED_LEFT_R_CHANNEL",  "PCA_CH_LED_LEFT_R",  None),
    ("PCA9685 — LED Channels", "LED_LEFT_G_CHANNEL",  "PCA_CH_LED_LEFT_G",  None),
    ("PCA9685 — LED Channels", "LED_LEFT_B_CHANNEL",  "PCA_CH_LED_LEFT_B",  None),
    ("PCA9685 — LED Channels", "LED_RIGHT_R_CHANNEL", "PCA_CH_LED_RIGHT_R", None),
    ("PCA9685 — LED Channels", "LED_RIGHT_G_CHANNEL", "PCA_CH_LED_RIGHT_G", None),
    ("PCA9685 — LED Channels", "LED_RIGHT_B_CHANNEL", "PCA_CH_LED_RIGHT_B", None),

    # ── PCA9685 — Servo Channels ──
    ("PCA9685 — Servo Channels", "SERVO_PAN_CHANNEL",  "PCA_CH_SERVO_PAN",  None),
    ("PCA9685 — Servo Channels", "SERVO_TILT_CHANNEL", "PCA_CH_SERVO_TILT", None),

    # ── PCA9685 — Motor Channels ──
    ("PCA9685 — Motor Channels", "MOTOR_RIGHT_IN1_CHANNEL", "PCA_CH_MOTOR_R_IN1", None),
    ("PCA9685 — Motor Channels", "MOTOR_RIGHT_IN2_CHANNEL", "PCA_CH_MOTOR_R_IN2", None),
    ("PCA9685 — Motor Channels", "MOTOR_RIGHT_PWM_CHANNEL", "PCA_CH_MOTOR_R_PWM", None),
    ("PCA9685 — Motor Channels", "MOTOR_LEFT_IN1_CHANNEL",  "PCA_CH_MOTOR_L_IN1", None),
    ("PCA9685 — Motor Channels", "MOTOR_LEFT_IN2_CHANNEL",  "PCA_CH_MOTOR_L_IN2", None),
    ("PCA9685 — Motor Channels", "MOTOR_LEFT_PWM_CHANNEL",  "PCA_CH_MOTOR_L_PWM", None),

    # ── Motor Control ──
    ("Motor Control", "MOTOR_STBY_PIN", "MOTOR_STBY_PIN", None),

    # ── MCP23017 GPIO Expander ──
    ("MCP23017 GPIO Expander", "MCP23017_ADDR", "MCP23017_ADDR", "hex_str"),

    # ── SSD1306 OLED ──
    ("SSD1306 OLED", "OLED_I2C_ADDR", "OLED_ADDR",   "hex_str"),
    ("SSD1306 OLED", "OLED_WIDTH",    "OLED_WIDTH",  None),
    ("SSD1306 OLED", "OLED_HEIGHT",   "OLED_HEIGHT", None),

    # ── Ultrasonic Rangefinder ──
    ("Ultrasonic Rangefinder", "ULTRASONIC_TRIG_PIN", "ULTRIG_PIN", None),
    ("Ultrasonic Rangefinder", "ULTRASONIC_ECHO_PIN", "ULECHO_PIN", None),

    # ── Piezo Buzzer ──
    ("Piezo Buzzer", "PIEZO_PIN", "PIEZO_PIN", None),

    # ── MAX98357A I2S Audio ──
    ("MAX98357A I2S Audio", "I2S_BCLK_PIN",         "I2S_BCLK_PIN", None),
    ("MAX98357A I2S Audio", "I2S_LRCLK_PIN",        "I2S_LRCLK_PIN", None),
    ("MAX98357A I2S Audio", "I2S_DIN_PIN",          "I2S_DIN_PIN", None),
    ("MAX98357A I2S Audio", "AUDIO_SAMPLE_RATE_HZ", "AUDIO_SAMPLE_RATE_HZ", None),
]
# fmt: on


def parse_pin_config_h(path: Path) -> dict[str, str]:
    """Return a dict of C #define name → raw value string."""
    text = path.read_text()
    pattern = re.compile(r"^\s*#define\s+(\w+)\s+(.+?)(?:\s*//.*)?$", re.MULTILINE)
    return {m.group(1): m.group(2).strip() for m in pattern.finditer(text)}


def transform_value(raw: str, transform: str | None) -> str:
    """Convert a C #define value into a Typst literal string."""
    raw = raw.strip()
    # Strip GPIO_NUM_() / GPIO_NUM_XXX wrapper
    m = re.match(r"GPIO_NUM_\(?(\d+)\)?", raw)
    if m:
        raw = m.group(1)

    if transform == "hex_str":
        return f'"{raw}"'
    elif transform == "int":
        return str(int(raw, 0))
    else:
        # Auto-detect
        if raw.startswith("0x"):
            return f'"{raw}"'
        try:
            int(raw, 10)
            return raw
        except ValueError:
            return f'"{raw}"'


def read_version(path: Path) -> str:
    """Read the version string from version.txt."""
    return path.read_text().strip()


def repo_relative(path: Path) -> str:
    """Render `path` relative to the repo root, independent of the cwd."""
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        # Outside the repo (unusual, but don't emit a cwd-dependent path).
        return resolved.as_posix()


def generate(pin_h_path: Path, version_path: Path, out_path: Path) -> None:
    """Parse, transform, and write the Typst definitions file."""
    defines = parse_pin_config_h(pin_h_path)
    version = read_version(version_path)

    lines = [
        "// Auto-generated by tools/typst/generate-pin-defs.py",
        f"// Source: {repo_relative(pin_h_path)}",
        f"//         {repo_relative(version_path)}",
        "// Do not edit manually — regenerate with `just robocar-unified::gen-pin-defs`.",
        "",
        f'#let VERSION = "{version}"',
        "",
    ]

    # Group by category_heading, preserving order
    current_cat = None
    for cat, c_name, t_name, transform in EXTRACT:
        if cat is None:
            continue
        if cat != current_cat:
            lines.append(f"// ── {cat} ──")
            current_cat = cat

        raw = defines.get(c_name)
        if raw is None:
            continue  # define not found in header; skip silently
        typ_val = transform_value(raw, transform)
        lines.append(f"#let {t_name} = {typ_val}")

    # Exactly one trailing newline — join + "\n" already supplies it. Appending
    # an empty element here would end the file with a blank line, which the
    # repo's end-of-file-fixer pre-commit hook strips right back out, so the
    # generator's own output would never survive a commit unchanged.
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines) + "\n")
    print(f"Wrote {out_path} ({len(lines)} lines)")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <pin_config.h> <version.txt> <output.typ>")
        sys.exit(1)

    generate(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
