# A Code Path Gated on Peripheral Presence Is Unrun Until the Peripheral Is Fitted

`init_hardware()` in robocar-unified returns early when the I2C bus does not
answer, so on a bare board **every line after that gate has never executed**:
motor, LED, servo and buzzer init, and the vendored drivers under them. The
firmware had booted cleanly for months. The first boot with the TCA9548A and
PCA9685 fitted (2026-09-05) hit two latent bugs on two consecutive boots, one
line apart, both from April:

| Boot | Symptom | Cause |
|---|---|---|
| 1 | `Double exception`, corrupted backtrace, `A0 = 0x10000000` | `pca9685_set_pwm_values()` indexed a `channels*4` VLA by the *absolute* channel number; `motor_stop()`'s (8, 6) call wrote 32 bytes past it |
| 2 | `Failed to center servos` → `ESP_ERROR_CHECK` reboot | `servo_controller_init()` centred through a setter that refuses until `initialized` is set, and set it afterwards |

Same shape as `freertos-task-gotchas.md` §1 (a non-fatal camera init exposed
the watchdog) and `esp-idf-rmt-rx.md` (a GPIO fallback hid three RMT bugs): a
gate that keeps the board booting also keeps the code behind it untested.
**When a gate opens for the first time, expect a chain, and clear it one bug at
a time.**

## Read the code between the last log line and the panic before any electrical theory

Boot 1 was first diagnosed as a supply sag: the panic landed the instant STBY
was driven high, the brownout detector is disabled for motor inrush, and a
double exception with scrambled registers is what a sag produces. Every
sentence was reasonable and the theory was wrong — the motor driver was not
wired to anything but power. The statement *after* the GPIO write was a
PCA9685 transaction, and reading the driver found the overflow in a minute.

Two panic-dump facts that settle it without a scope:

- **A recognisable data byte in the return register is a stack smash.** `A0 =
  0x10000000` is `LED_FULL_ON_OFF` (`1 << 4`), which the buggy loop stores as
  the high byte of every fourth word. A sag scrambles registers; an overflow
  writes *your own constants* into them. Look up the top byte of a corrupted
  `A0`/`PC` in the driver you just called.
- **The FreeRTOS canary does not catch upward overflows.** `CHECK_STACKOVERFLOW_CANARY`
  watches the low end of the stack; a VLA overflow runs *upward* into caller
  frames and never touches it. With `COMPILER_STACK_CHECK_MODE_NONE` (this
  build) there is no protector either, so the first symptom is the return into
  garbage. `EXCCAUSE 2` (instruction fetch) with `EXCVADDR` in DRAM near `A1`
  is that return.

`analog-fault-isolation.md` says two electrical theories fitting at once means
the fault is upstream of both. The firmware version: an electrical theory that
fits *before the code path has been read* is a theory about the wrong layer.

## Pin the fix where the bench cannot

Both fixes are one line and both are pinned in `test/`:

- `test_pca9685_multi.c` compiles the vendored driver **unmodified** against a
  recording `i2cdev` shim under `-fsanitize=address`, so the overflow itself
  fails the run rather than whatever garbage it produced. Content assertions
  alone can pass on lucky bytes; ASan cannot. Mutation-checked: with the fix
  reverted, ASan aborts at the first out-of-bounds read.
- `test_servo_controller.c` compiles the module unmodified over a recording
  `i2c_bus` stub. The load-bearing case is a failed bus write leaving the
  module *uninitialised* — the fix raises the flag before centring, so it must
  also lower it on failure.

The pattern for any vendored driver: a shim header directory added `BEFORE`
the component on that target's include path (`test/include/pca9685_host/`), so
the real `pca9685.h` resolves its `<i2cdev.h>` to the shim and the `.c` under
test is the shipped file, not a retyped copy.

## Related

- `freertos-task-gotchas.md` §1, `esp-idf-rmt-rx.md` — the two earlier
  instances of a gate masking the path behind it
- `analog-fault-isolation.md` — the hardware sibling; that file's canonical
  break (a disconnected amp ground) is why the first theory here was electrical
- `~/.claude/rules/diagnose-at-the-failure-point.md` — evidence at the failure
  point beats the framing you walked in with; here the framing was "the bench
  is being rewired"
- `.claude/skills/esp32-debugging/SKILL.md` § Runtime Panic Analysis — the
  double-exception row
