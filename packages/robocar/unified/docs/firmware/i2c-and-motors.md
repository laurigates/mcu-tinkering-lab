# robocar-unified: I2C bus, PCA9685 and motors

Moved word for word from the project `CLAUDE.md`, which keeps a
one-line form of each rule below and links here for the reasoning.

## I2C topology

All I2C devices hang off a **TCA9548A multiplexer** on GPIO5/6. Don't talk to devices directly on the primary bus — always select the channel first via `i2c_bus.c`. Current channel map in `pin_config.h`:

- ch0: PCA9685 (motors, servos, LEDs)
- ch1: SSD1306 OLED
- ch2: MCP23017 GPIO expander (optional — 16 generic GPIOs via `gpio_expander.c`, no roles assigned yet, GPA7/GPB7 output-only; firmware boots fine without the board fitted)
- ch3: RCWL-1601 rangefinder, front (0x57, I2C mode)
- ch4: RCWL-1601 rangefinder, rear (0x57, I2C mode)
- ch5-7: reserved

The mux's active-low RESET is on XIAO D2 (`TCA9548A_RST_PIN`).
`i2c_bus_select_channel()` pulses it after `I2C_BUS_MUX_RESET_AFTER_FAILURES`
consecutive failed selects, which deselects every channel and frees the
upstream bus from a segment held low; `trace` counts the resets.

The MCP23017 (CJMCU-2317 module or the integrated carrier's chip; `MCP23017_ADDR` 0x20, with init falling back to the first of 0x20–0x27 that answers and logging a warning, #781) is exercised from the serial console with `gpio`, `gpio mode <pin> in|up|out`, `gpio set <pin> 0|1`, `gpio get <pin>`.
Pins 7 (GPA7) and 15 (GPB7) are **output-only** on the Rev D part (DS20001952D p. 11, § 3.5.1 note): init sets them to outputs driven low, and `gpio mode 7|15 in|up` is refused with a message saying so (`ESP_ERR_NOT_SUPPORTED` from `gpio_expander_set_mode()`). The check is `gpio_expander_core.c`, pinned by `test_gpio_expander_core.c`.

**Every peripheral above is non-fatal, and that did not always hold.** `init_hardware()` degraded gracefully when the I2C bus was absent — a bare board booted to the console — but once the bus answered, a failure in the expander, motors, LEDs, servos or buzzer aborted the boot. That is how PR #498's servo bug presented: a reboot loop with the console unreachable, i.e. the outcome every other non-fatal phase exists to prevent, and the one state in which nothing can be diagnosed. Since issue #500 the phase returns `void` and logs each failure, and `self_report` reads all four peripherals back through their live accessors so the facts line names which one is missing:

```
i2c_peripherals=ok                          # bus up, everything behind it up
i2c_peripherals=degraded(motors,servos)     # bus up, these did not initialise
i2c_peripherals=not-responding              # no bus at all (bare board, or a bus fault)
buzzer=not-responding                       # separate key: it is a GPIO, not I2C
```

The spoken fault line stays coarse — one persona phrase covers "no bus" and "motors did not initialise", because from outside the robot the symptom is identical and the persona has no wording for LEDs, servos or the buzzer. The per-peripheral breakdown lives in the logged and MQTT-published facts line, which is where somebody diagnosing a board is actually looking.

## PCA9685 channel layout

All motor direction, motor PWM, servo, and LED outputs go through the PCA9685 — the ESP32-S3 only drives STBY (GPIO1), the buzzer (GPIO2), and I2C. GPIO budget on XIAO headers is tight (11 pins). See the channel table in `main/pin_config.h` before reassigning.

Motor direction uses PCA9685 "full-on" (4096) / "full-off" (0) values on IN1/IN2 channels.

**Channels 8-13 follow the TB6612FNG's control header, not a per-motor order.**
Read that header top to bottom and it is PWMA, AIN2, AIN1, STBY, BIN1, BIN2,
PWMB — symmetric about STBY rather than repeated — so assigning 8-13 in that
order makes the six jumpers run straight across with no crossings, at the cost
of the A side reading PWM-then-direction and the B side direction-then-PWM. The
apparent asymmetry in the `#define` list is what buys the symmetry on the bench.
STBY is not in the block; it is GPIO1, direct from the MCU.

**Stopping and braking are different electrical states.** `motor_stop()` writes
IN1 = IN2 = low, which the TB6612FNG truth table calls *Stop* — the outputs go
high-impedance and the robot rolls on inertia. `motor_brake()` writes
IN1 = IN2 = **high**, shorting the windings. Only the obstacle reflex brakes;
every other stop in the 30 Hz loop coasts, because the executor stops on every
iteration it is not driving and a parked robot should not sit with its windings
shorted for as long as it is powered. `test_motor_controller.c` pins the two
encodings apart and `test_reactive_controller.c` pins which path uses which —
both were mutation-checked, since a PWM-only assertion passes against either.

`set_motors()` writes all six in one transaction and therefore places each value
by **channel** (`MOTOR_CH_SLOT`), never by argument position, with a
`_Static_assert` that the six are a gap-free block starting at
`MOTOR_FIRST_CHANNEL`. A positional array would keep compiling after a renumber
and simply drive the wrong pins — on a motor driver, a wheel that spins when it
was told to stop. `test_each_value_lands_on_its_own_channel` in
`test_motor_controller.c` pins the placement; it was mutation-checked by
reverting to the positional literal, which fails it.

## Bus clock and PWM phase

Two properties of every PCA9685 write, both set in `i2c_bus.c` and neither
visible at the call site.

**One bus clock for the whole port.** Each esp-idf-lib driver picks its own rate
in `init_desc()` — TCA9548A 100 kHz, PCA9685 and MCP23017 1 MHz — so without an
override the bus runs at whichever speed the last-addressed device wanted.
`pin_bus_clock()` puts all three on `I2C_MASTER_FREQ_HZ` (400 kHz, the fastest
every part here is rated for; the TCA9548A is the Fast-mode limit). That define
had sat unread in `pin_config.h` since the first commit.

Under **i2cdev 1.x** this also removed a real cost: `cfg_equal()` compared
`clk_speed`, so consecutive transactions at different rates made
`i2c_setup_port()` delete and reinstall the whole I2C driver first — twice per
motor update, ~30 times a second while driving, because a mux channel-select
always precedes a device write. **i2cdev 2.0.0 moved to ESP-IDF's bus/device
API**, which caches a per-device handle carrying its own `scl_speed_hz` on a
shared bus, so that reinstall is gone by construction. The override stays for
the first reason only.

**Outputs are phase-staggered.** The vendored driver hardcodes every channel's
ON count to 0, so all sixteen outputs rose on the same tick of the ~197 Hz
period — two motor PWMs, two servos and six LED channels switching together.
`pca9685_phase.c` gives channel *n* an ON offset of *n*/16 of the period and
composes the LEDn registers itself, because upstream esp-idf-lib has no API for
it and the vendored copy is a **symlink into `robocar/main`** — patching it
would change that firmware too and be reverted by any library refresh.

Staggering moves a pulse within the period without changing its width, so
nothing downstream can tell: a servo decodes pulse width, not position in the
frame, and the TB6612FNG sees the same duty cycle. Full-on and full-off are
flag bits rather than counts and deliberately carry **no** phase — on this board
those are the motor direction pins, and a phase offset there would turn a
digital level into a ~197 Hz square wave. `test_pca9685_phase.c` pins all of it,
including the 12-bit wrap at channel 15 where OFF legitimately lands below ON.

This is headroom, not a fix: the audio distortion that prompted the
investigation was resolved by adding bulk capacitance.

## Where the I2C drivers come from

`pca9685`, `tca9548`, `mcp23x17` and `i2cdev` are **managed components**
(`main/idf_component.yml`), not vendored source. They used to be a ~95-component
snapshot of the `UncleRus/esp-idf-lib` monorepo living under `robocar/main` and
reached from here by symlink; that monorepo is archived, and the library now
ships as per-component repos under the `esp-idf-lib` org.

**`pca9685` is floored at `'^1.0.8'`, and the floor is load-bearing.** Releases
up to 1.0.7 index the stack buffer in `pca9685_set_pwm_values()` by the
*absolute* channel number while sizing it for `channels` entries, so any
`first_ch > 0` writes past the end — `motor_stop()`'s (8, 6) wrote 32 bytes past
a 24-byte VLA and double-faulted this board on the first boot with the PCA9685
fitted. The fix landed upstream as `esp-idf-lib/pca9685#2` and shipped in 1.0.8;
until then the manifest pointed at a fork branch (issue #526).

Do **not** relax it to `'^1.0.0'` — the resolver could then pick a release with
the board-crashing bug. `test_pca9685_multi` compiles the resolved driver under
AddressSanitizer specifically to catch that, and was control-tested against the
1.0.0 registry source: it aborts with `stack-buffer-overflow ... in
pca9685_set_pwm_values`.

That test reads the driver from `managed_components/`, which is gitignored, so
it is **skipped with a `STATUS` message in a fresh checkout** and appears once
`just robocar-unified::build` has fetched the dependency. No other suite needs
it. Two declaration-only shims in `test/include/pca9685_decl/` keep the
remaining targets configuring without a prior firmware build; they are stubs,
and the firmware build — which uses the real headers — is what catches any
disagreement.


## Don't

- Don't call `motor_controller.c` directly from anywhere except `reactive_controller.c` — the executor owns motor output. Console/manual movement goes through `reactive_controller_manual()`, which takes a short lease the executor applies *after* the obstacle reflex; a second task writing the PCA9685 directly both fought the 30 Hz executor and bypassed the reflex
- Don't drop the idle re-write suppression in `set_motors()`, and don't make it permanent by removing the refresh. `reactive_controller`'s 30 Hz loop calls `motor_stop()` on every iteration it is not driving, so a parked robot re-stated six identical PCA9685 registers 30 times a second — the firmware's only continuous I2C traffic, and what makes the bus go from silent to permanently busy the moment the PCA9685 is fitted. The suppression expires after `MOTOR_REFRESH_INTERVAL_MS` because the cache describes a chip that cannot be read back: a PCA9685 that browned out or was re-seated no longer matches it, and with no expiry nothing would re-assert the true state. Pinned by `test_motor_controller.c`, including the case a bench cannot stage — the uint32 millisecond wrap at day 49, where a signed elapsed comparison either refreshes on every call or never refreshes again
- Don't put `ESP_ERROR_CHECK` or `ESP_RETURN_ON_ERROR` back into the hardware phase. `init_hardware()` returns `void` on purpose (issue #500): a peripheral that would not initialise used to abort the boot, taking the console, WiFi, provisioning and the planner down with it — and a reboot loop removes the very console you would read the fault on. Every driver behind that phase already fails safe (each guards its entry points on `initialized` and returns `ESP_ERR_INVALID_STATE`; the buzzer's tone routines no-op), so a failure costs exactly the function that failed
- Don't drop a peripheral's `*_is_initialized()` accessor, or let `self_report_collect()` read a cached boot result instead. Those accessors are the only thing standing between a half-populated board and a silent loss of function: the facts line renders `i2c_peripherals=degraded(servos)` from them, and `test_self_report.c` pins the case a bench cannot stage — a live bus with exactly one dead peripheral
- Don't make `motor_stop()` a short brake, or "unify" it with `motor_brake()`. Low/low is coast and high/high is brake; the reflex wants the second and the 30 Hz idle path wants the first. A test that only checks PWM = 0 passes against either, which is why the stub records which entry point was called
- Don't let a driver's `init_desc()` pick the bus clock. Each esp-idf-lib driver hardcodes a different one, and i2cdev reinstalls the whole I2C driver whenever consecutive transactions disagree — call `pin_bus_clock()` after every `init_desc()`, including for any device added later
- Don't change the PCA9685 frequency without re-sending the servo counts — call `servo_set_pwm_frequency()`, never `i2c_bus_pca9685_set_frequency()` directly. The prescaler write leaves every channel's count in place, and a count is a fraction of the period: the 50 Hz centre count (307) replayed at 200 Hz is a ~0.37 ms pulse, which drove both servos into their end stops on the 2026-09-15 bench. `servo_set_pwm_frequency()` releases the enabled outputs, changes the prescaler and re-sends each angle; `test_a_frequency_change_keeps_every_pulse_width` pins it
- Don't give a full-on or full-off channel a phase offset in `pca9685_phase.c`. Those are flag bits, and on this board they are the motor direction pins: a phase offset converts a steady logic level into a ~197 Hz square wave on IN1/IN2
- Don't patch the PCA9685 driver in `managed_components/` to add phase support — it is a fetched dependency, so the edit is wiped by the next `idf.py reconfigure`. Phase lives in `pca9685_phase.c`, which is ours
- Don't lower the `esp-idf-lib/pca9685` floor below `'^1.0.8'`. Releases up to 1.0.7 carry the out-of-bounds write that double-faulted this board. `test_pca9685_multi` catches it under ASan, but only when `managed_components/` is populated
- Don't "tidy" the motor channels back into a per-motor IN1/IN2/PWM order. They are in the motor driver's pin order on purpose, and that ordering is a claim about **soldered wires** — reordering the `#define`s is a rewiring instruction, not a refactor, and the firmware will follow the new numbers perfectly onto the wrong pins
- Don't add direct GPIO motor control — everything goes through PCA9685 via `motor_controller.c`
- Don't bypass the TCA9548A — devices on different channels can share addresses (e.g. PCA9685 and OLED would conflict without it)
