---
name: peripheral-offload
description: Decide whether firmware work belongs in software, an on-chip peripheral (DMA, RMT, PCNT, MCPWM, PIO, ADC monitor, glitch filter, ULP) or external silicon, and verify the chip has it before designing around it. Use when about to write an ISR, a polling loop, a bit-bang or a busy-wait, when a control loop shows jitter or overruns, or when weighing an offload proposal (including one from another model).
user-invocable: true
argument-hint: "[task, e.g. 'wheel encoders on robocar' or a proposal to evaluate]"
allowed-tools: Bash, Read, Grep, Glob, WebFetch
---

# Peripheral Offload: Software, On-Chip Hardware, or External Silicon

Moving work from a CPU loop into a peripheral removes interrupt jitter and frees
the core, but each offload adds a driver, a register-width constraint and a
failure mode that can be silent. This skill is the decision procedure and the
checks that keep an offload proposal honest.

## 1. Measure the cost before offloading anything

An offload is justified by a number, not by the category of task. Get one of:

- **Rate × per-event cost.** 25 000 context switches per second is a problem;
  one CRC over 100 bytes at parameter save is not.
- **Jitter or overrun on the loop that matters.** The robocar executor runs at
  30 Hz and guards the overrun case (`reactive_controller.c`, the
  `xTaskDelayUntil() == pdFALSE` branch); `freertos-task-gotchas.md` §1 is what
  an unguarded overrun does.
- **A latency bound a test pins.** `REACTIVE_REFLEX_WORST_CASE_MS` in
  `test_reflex_latency.c` is the obstacle-to-brake budget; an offload that does
  not move it is not a latency fix.

No number means no offload yet. Leave the software path, add the measurement.

## 2. Pick the tier

| Task | Software | On-chip | External |
|---|---|---|---|
| Buffer movement (I2S, SPI, UART, ADC) | per-byte copy | DMA, descriptor ring | — |
| Step / pulse generation | timer ISR toggling | PIO, RMT TX, MCPWM, timer + DMA | motion controller (TMC5160) |
| Quadrature / pulse count | GPIO edge ISR | PCNT, timer encoder mode, PIO | counter IC (LS7366R) |
| Pulse-width capture | edge ISR + timestamp | RMT RX, MCPWM capture, input capture | — |
| Non-standard serial timing | bit-bang | PIO, RMT, LCD_CAM, FlexIO | — |
| Debounce / glitch reject | poll + timer | GPIO glitch filter | RC + Schmitt (74HC14) |
| Threshold watch | ADC read + compare | ADC monitor / analog watchdog, comparator | window comparator |
| Event → action without ISR | ISR reads A, writes B | ETM, PPI, EVSYS, DMAMUX | discrete logic |
| Periodic sensing while asleep | wake main core | ULP FSM / RISC-V, LP core | sensor with interrupt pin |
| CRC / hash / crypto | table or bitwise loop | CRC unit, DMA sniffer, AES/SHA | — |
| Many PWM channels | LEDC/MCPWM per pin | — | PCA9685 over I2C |

Prefer the lowest tier that meets the measured need. External silicon costs
board area, a BOM line (`part-sourcing.md`) and a bus transaction per update.

## 3. Verify the chip has the peripheral, at the IDF version in use

Peripheral sets differ inside one family, and recalled capability lists are
wrong often enough to matter. ESP32-S3 has **no ETM** (that is C6, H2 and P4);
it has a fixed pin glitch filter but not the configurable one; it has no CRC
peripheral (use `esp_rom_crc32_le()`). Read the capability header for the exact
target and the IDF release the container pins (`docker-compose.yml`):

```sh
curl -sSfL https://raw.githubusercontent.com/espressif/esp-idf/release/v5.4/components/soc/esp32s3/include/soc/soc_caps.h -o /tmp/soc_caps.h
grep -nE "SOC_(ETM|PCNT|MCPWM|RMT|GDMA|ULP|RISCV_COPROC|ANA_CMPR|GPIO_.*GLITCH|ADC_MONITOR|SIMD)" /tmp/soc_caps.h
```

A macro that is absent means the peripheral is absent. For RP2040/RP2350,
STM32 and nRF, read the datasheet's peripheral chapter, not a summary.

Then read the driver's register-width limits in the IDF source before choosing
values: `esp-idf-rmt-rx.md` records three RMT RX limits (even `mem_block_symbols`
≥ 48, an 8-bit glitch filter on the source clock, a 15-bit idle threshold) that
each fail a different call.

## 4. Check the offload against the actual board

An on-chip peripheral only helps if the signal reaches a pin it can use.

- **Is the signal on an MCU pin at all?** robocar drives every motor
  and servo line through the PCA9685 (I2C), so MCPWM fault inputs, timer DMA and
  ETM cannot touch them; only `MOTOR_STBY_PIN` (GPIO1) is direct.
- **Are pins free?** Read the generated pin table in the project's `WIRING.md`
  (`hardware.toml` is the source, ADR-021). robocar has D3 (GPIO4) and
  D7 (GPIO44) spare; everything else goes through the MCP23017.
- **Does the bus already belong to someone?** A ULP polling an I2C bus the
  main core drives at 30 Hz needs arbitration the firmware does not have.
- **Does the device ever sleep?** ULP offload pays only if the main cores power
  down. A robot holding WiFi, a camera and TLS sessions never does.

## 5. Make a failed offload loud

A driver that falls back to a software path on peripheral init failure keeps
working, degraded, and hides the error (`esp-idf-rmt-rx.md`). For each offload:

- Log the init result at WARN or ERROR with the `esp_err_to_name()` text.
- Expose it in the self-report or a status command, as the robocar's
  `*_is_initialized()` accessors do.
- Re-arm on timeout where the peripheral can wedge (RMT RX does).
- Keep a host test on the logic around it (shared-core pattern, `testing.md`).

## 6. Evaluating an offload proposal

Proposals written from an indexed or summarised view of the repo describe a
design that may be gone. Check every premise against `main` before weighing the
idea:

1. **Does the named file, function or bus exist?** `rg -n "<symbol>" packages/<proj>`.
2. **Is the "current bottleneck" the current code?** Comments often describe
   the bug a constant fixed (`TTS_PCM_BATCH_BYTES` in `gemini_tts.c` records
   the 25 000 yields/s it removed).
3. **Is the hardware it assumes fitted?** `hardware.toml` and `WIRING.md` list
   what is wired; a sensor or encoder that is not there makes the proposal moot.
4. **Did the repo already reject it, with a reason?** Search the subsystem doc
   and the module header (`voice_fx_core.h` rejects a band-limit).
5. **Does the chip have it?** §3.

Report each point as confirmed, moot (premise false) or open (needs a
measurement), with the file and line that decided it.

## Worked examples in this repo

| Offload | Where | Note |
|---|---|---|
| PIO step generation | `packages/robotics/balancebot/src/stepper.pio` | `pull noblock` holds the last rate; the CPU only pushes a period per control tick |
| External PWM | robocar PCA9685 (`motor_controller.c`) | frees MCU pins and timers; costs an I2C write per change |
| I2S TX DMA with silence preload | `audio_player.c` `preload_silence()` | `auto_clear` zeroes on underrun, not at enable |
| Producer batching into a DMA consumer | `gemini_tts.c` `TTS_PCM_BATCH_BYTES` | software fix for a scheduling cost; no new peripheral |
| I2C rangefinders instead of echo timing | robocar `ultrasonic.c` (#782, #790) | moved pulse capture into the sensor module |

Counter-example: balancebot's bitwise `crc32_calc()` runs once per parameter
save over one small parameter record (`PARAM_V3_SIZE`). The RP2350 DMA CRC sniffer would save nothing
measurable, and the bitwise loop is host-testable as-is.
