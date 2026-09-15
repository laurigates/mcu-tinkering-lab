#!/usr/bin/env python3
"""Generate Typst `#let` bindings from C headers (pin_config.h plus others).

Usage:
    python3 tools/typst/generate-pin-defs.py \
        packages/robocar/unified/main/pin_config.h \
        [packages/robocar/unified/main/planner_task.h ...] \
        packages/robocar/unified/docs/auto/pin_defs.typ

The first argument is the project's pin_config.h, the last is the output, and
anything between is an additional header to read `#define`s from. The original
two-argument form is unchanged.

The output is a Typst file with #let bindings for every relevant constant.
Run it before `typst compile` to keep docs in sync with the C source of truth.

What may feed this file is decided by WHO WRITES THE SOURCE, not by whether it
is hardware. A firmware constant that only a human edits (a pin in
pin_config.h, the planner's base period in planner_task.h) is safe: changing it
is a source commit, which is on the drift guard's trigger paths, so the guard
runs and demands a regenerated PDF in that same commit. A file that release
automation rewrites is not safe. The firmware version used to be read from
`version.txt` and emitted as `VERSION`: a release-please bump changed it, which
is on none of the guard's trigger paths, so the guard never ran and the
committed PDF silently kept printing the previous version. Regenerating to fix
that is itself a `docs:` commit, which release-please turns into the next
release — a loop that never converges (see issue #439). Never add an input that
a release pipeline writes, and add every new input header to the guard's
trigger paths in `.github/workflows/build-guide-check.yml`.
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

    # ── Planner cadence (planner_task.h, plan_activity.h) ──
    ("Planner cadence", "PLANNER_LOOP_PERIOD_MS",     "PLANNER_LOOP_PERIOD_MS",     "int"),
    ("Planner cadence", "PLAN_LADDER_STEPS",          "PLAN_LADDER_STEPS",          "int"),
    ("Planner cadence", "PLAN_LADDER_TOP_MULTIPLIER", "PLAN_LADDER_TOP_MULTIPLIER", "int"),
]
# fmt: on


def parse_pin_config_h(path: Path) -> dict[str, str]:
    """Return a dict of C #define name → raw value string."""
    text = path.read_text()
    pattern = re.compile(r"^\s*#define\s+(\w+)\s+(.+?)(?:\s*//.*)?$", re.MULTILINE)
    return {m.group(1): m.group(2).strip() for m in pattern.finditer(text)}


def parse_headers(paths: list[Path]) -> dict[str, str]:
    """Merge the `#define`s of several headers; fail on a conflicting redefinition.

    A name defined in two inputs with different values would make the output
    depend on argument order, so that is an error rather than a silent pick.
    """
    merged: dict[str, str] = {}
    for path in paths:
        for name, value in parse_pin_config_h(path).items():
            if name in merged and merged[name] != value:
                sys.exit(
                    f"error: {name} is defined as both {merged[name]!r} and {value!r} across {[str(p) for p in paths]}"
                )
            merged[name] = value
    return merged


# C integer-literal suffixes (`15000U`, `0x40UL`). Stripped before transforming,
# or the int parse fails and the value is emitted as the string "15000U".
_INT_SUFFIX = re.compile(r"^(0[xX][0-9A-Fa-f]+|\d+)[uUlL]+$")


def transform_value(raw: str, transform: str | None) -> str:
    """Convert a C #define value into a Typst literal string."""
    raw = raw.strip()
    # Strip GPIO_NUM_() / GPIO_NUM_XXX wrapper
    m = re.match(r"GPIO_NUM_\(?(\d+)\)?", raw)
    if m:
        raw = m.group(1)
    m = _INT_SUFFIX.match(raw)
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


def repo_relative(path: Path) -> str:
    """Render `path` relative to the repo root, independent of the cwd."""
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO_ROOT).as_posix()
    except ValueError:
        # Outside the repo (unusual, but don't emit a cwd-dependent path).
        return resolved.as_posix()


def generate(header_paths: list[Path], out_path: Path) -> None:
    """Parse, transform, and write the Typst definitions file."""
    defines = parse_headers(header_paths)

    lines = [
        "// Auto-generated by tools/typst/generate-pin-defs.py",
        *(f"// Source: {repo_relative(p)}" for p in header_paths),
        "// Do not edit manually — regenerate with `just robocar-unified::gen-pin-defs`.",
        "",
    ]

    # Group by category_heading, preserving order
    current_cat = None
    for cat, c_name, t_name, transform in EXTRACT:
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
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <pin_config.h> [<extra.h> ...] <output.typ>")
        sys.exit(1)

    generate([Path(a) for a in sys.argv[1:-1]], Path(sys.argv[-1]))
