# robocar-unified: Serial console and MQTT commands

Moved word for word from the project `CLAUDE.md`, which keeps a
one-line form of each rule below and links here for the reasoning.

**Opening the USB console resets the board.** On the XIAO's USB-Serial-JTAG,
opening `/dev/cu.usbmodem*` restarts the ESP32-S3 (`rst:0x15
USB_UART_CHIP_RESET`), even from a client that holds DTR and RTS low (observed
2026-11-04 with pyserial on macOS). So a check that opens the port to see
whether the robot is idle wakes it up. To stop the robot without unplugging it
(no WiFi, no API calls), park it in the ROM bootloader with
`uvx esptool@latest --port /dev/cu.usbmodem101 --after no-reset chip-id`, and
confirm over MQTT rather than the port: a running robot publishes
`robocar/status` every 3 s, and a parked one leaves only the broker's retained
copy (`mosquitto_sub -F '%r %t'` prints `1` for it). Any reset or a port open
starts the app again.

**Bench control without the port** (2026-11-05). `just robocar-unified::cmd
"<line>"` sends one console line over MQTT, so nothing resets the board. These
lines keep a bench session readable:

- **Wheels still, planner on hold:** `plan requests 1` trips the spend fuse
  (any ceiling below the count already spent does). The planner forces `stop`
  within one tick and makes no request until `plan resume` or a reboot. `S`
  alone stops for a moment only: a fresh `track` or `drive` goal restarts the
  wheels on the next tick, and `plan sleep` lasts only until the next piece of
  evidence.
- **Judge the head only under a lease:** `servo pan 0` / `servo tilt 0` hold
  for 30 s and outrank both `track` and the idle gaze. A head that looks off
  centre without a lease is usually tracking something or gazing, not at
  neutral. `idle gaze off` keeps it centred while nothing directs it.
- **A flash or reboot clears all of the above** (the fuse, `idle`, `voice
  volume`, leases), and the robot plans on boot as soon as it has evidence. Send
  the settings again once it is back on MQTT. A script that waits for the
  robot must start after the flash finishes: until the reset, the old firmware
  keeps publishing `robocar/status`, and a non-retained subscribe returns on
  that status rather than on the new boot.

## Serial console commands

| Command | Effect |
| --------- | -------- |
| `F` `B` `L` `R` `C` `W` `S` | Manual drive / rotate / stop — a ~1 s lease via `reactive_controller_manual()`, still subject to the obstacle reflex |
| `gpio …` | MCP23017 expander (see above) |
| `voice …` | Persona / TTS voice switching and auditioning; `voice vary` shows a drawn variation directive, `voice said` the recall ring, `voice quiet`/`budget`/`repeat`/`scene` the gates on speaking, `voice vad on\|off` toggles VAD auto-triggering (on at boot), `voice trigger <band%> <db> <ms>` what starts a VAD turn, `voice endpoint <ms> <db>` when it stops recording, `voice ration <n> <s>`/`turns <n>`/`resume` what voice turns may spend, `voice volume <pct>` the output gain |
| `snap [n]` | Dump the next `n` planner frames over the console as base64 JPEG — see "Seeing what the camera sends" |
| `listen [ms]` / `listen clear` | Record and answer a conversational turn (or clear conversational history) |
| `cam …` | Read live sensor gain/exposure; `cam gainceiling 0-6`, `cam ae -2..2`, `cam brightness -2..2` tune exposure without a reflash |
| `sound beep\|melody\|alert` | Buzzer |
| `servo …` | `servo` alone reports readiness, on/off, both angles with the PCA9685 counts, the live travel limits and the PWM frequency; `servo pan\|tilt <deg>` holds one for 30 s through a head lease the executor applies (refused outside its limits), then the executor takes the head back; `servo on\|off` drives or releases both outputs, and runs directly on the console task so `off` works mid-exercise; `servo limit pan\|tilt <min> <max>` sets the live travel limits (boot default pan ±60°, tilt ±30°; not persisted); `servo exercise` shakes then nods between the limits, logging every write; `servo freq <24-1526>` changes the chip-wide prescaler and re-sends both servo pulses at the new period |
| `led <r> <g> <b>` | Both RGB LEDs |
| `mic` / `mic dump <n>` | Microphone state and the VAD speech trigger's scores (best since the last `mic`); dump PCM frames — tells a dead mic from a quiet room |
| `idle …` | The executor's idle behaviour (ADR-029). `idle` alone prints whether the gaze is on, the head mode and whether the head has settled (whether a frame captured now gets a dwell pose); `idle gaze on\|off` switches the idle gaze (on at boot, not persisted). See [head-aiming.md](head-aiming.md#idle-gaze-looking-around-when-nothing-directs-the-head) |
| `ota` / `ota check` / `ota postpone` | `ota` alone prints the running version and build, and where an update is (idle, waiting for a quiet moment, announcing, postpone window open, parking, installing); `ota check` checks the manifest now, as the MQTT nudge does; `ota postpone` defers an announced update to the next check, accepted only in the minute after the robot says it is going. See [wifi-and-ota.md](wifi-and-ota.md#before-the-install-announce-postpone-park) |
| `plan …` | The gate on whether the planner makes a request at all — `plan` alone reports cadence, wake scores and spend; `plan on\|off\|wake\|sleep\|resume`, `plan scene\|range\|requests\|tokens <n>` |
| `trace …` | Camera + endpoint activity counters since boot; `trace led on\|off`, `trace reset` |
| `mqtt …` | Serial only. `mqtt` alone prints the MQTT command mode (`full`/`read-only`) and where the broker credentials came from; `mqtt auth <user> <pass>` stores them in NVS, `mqtt auth clear` removes them — both applied at the next boot |
| `gemini …` | Serial only. `gemini` alone prints whether a key is set, where it came from (`nvs`/`credentials.h`/`env`/`none`), its length and what the last response said (`untested`/`ok`/`rejected`/`exhausted`); `gemini auth <key>` stores a key in NVS, `gemini auth clear` removes it — both applied from the next request. The key is never printed or logged (issue #845) |

### MQTT remote commands

Every command above except `mqtt` is also reachable over MQTT — all of them
only once broker credentials are configured, see below — for the case the console table
above already exists for but cannot reach: a board with no serial link
attached. Publish a plain-text line — the exact same grammar as the console,
not JSON — to `MQTT_COMMAND_TOPIC` (`robocar/commands`, QoS 1):

```bash
mosquitto_pub -h <broker> -t robocar/commands -m "plan resume"
mosquitto_pub -h <broker> -t robocar/commands -m "F"
```

`mqtt_command.c` (pure C, host-tested in `test/test_mqtt_command.c`) validates
and classifies the payload before anything acts on it:

- **Movement** — the console's single letters (`F`/`B`/`L`/`R`/`C`/`W`/`S`,
  case-insensitive) or their word form (`forward`/`backward`/`left`/`right`/
  `rotate_cw`/`rotate_ccw`/`stop`) — routes through `dispatch_movement()`,
  the same queue + `motor_control_task` the console uses, so it reaches
  `reactive_controller_manual()` and stays behind the obstacle reflex.
  `REACTIVE_MANUAL_TTL_MS` (1 s) still applies: a single MQTT message buys
  one second of motion, so continuous remote driving means republishing at
  ~1 Hz, and a dropped connection stops the robot within a second.
- **Everything else** in the console table above (`plan`, `voice`, `trace`,
  `mic`, `cam`, `servo`, `led`, `sound`, `gpio`, `snap`, `listen`, `idle`) — forwarded
  verbatim to `execute_console_line()`, the exact function the serial console
  calls, so the two entry points can never disagree about what a command
  does.
- **Anything else, and any malformed or oversized payload, is rejected and
  logged rather than dispatched.** `event->data` is not NUL-terminated, and
  esp-mqtt can split one message across several `MQTT_EVENT_DATA` callbacks
  once it exceeds the internal buffer (`event->total_data_len` /
  `event->current_data_offset`) — a fragment is the topic ("target") already
  resolved once, with more of the payload still arriving, and dispatching on
  it would act on a truncated command. `mqtt_command_extract_line()` rejects
  anything that is not a single, complete, in-bounds (≤63 bytes), printable-
  ASCII line before `mqtt_command_dispatch()` ever sees it.

**Without broker credentials the topic is read-only** (issue #626). A board
with no `MQTT_USERNAME`/`MQTT_PASSWORD` connects anonymously, and on an
anonymous broker anything on the LAN can publish to `robocar/commands` — so in
that mode only the bare status forms reach a handler: `plan`, `trace`, `mic`,
`cam`, `servo`, `gpio`, `voice`, `voice said`, `idle`. Movement, every setting, and
`snap`/`mic dump`/`listen` (camera frames and raw audio are not status) are
refused and logged as `MQTT command refused: read-only …`. With both a
username and a password configured, the full command set above is available.

- **The allow-list is exact whole lines in `mqtt_command.c`**, not prefixes,
  because the handlers parse loosely — `trace foo` falls through to the report
  and `voice <slug>` switches the persona and persists it. A console command
  added later is therefore locked out of read-only mode until somebody adds it
  to the list on purpose. `test_mqtt_command.c` pins both sides of the list.
- **Credentials come from NVS first, then `credentials.h`.** `mqtt auth <user>
  <pass>` on the serial console stores a pair in NVS, `mqtt auth clear` removes
  it, and `mqtt` alone prints the mode and where the credentials came from.
  Both apply at the next boot, because the MQTT client is created once. `mqtt`
  is serial-only by construction — handled in `command_task()` before
  `execute_console_line()` and absent from `mqtt_command.c`'s prefixes — so an
  anonymous publisher cannot set the credentials that would lift its own
  lockout. The console line grew from 32 to 128 bytes for it, and the line is
  logged as `mqtt ... (redacted)`. The CI and web-flasher stub defines both as
  empty, so no shipped image carries a credential.
- **Credentials only help if the broker enforces them.** The firmware sees
  whether it was given credentials, not whether the broker refuses anonymous
  clients — give the broker `allow_anonymous false` and an ACL on
  `robocar/commands`, or a configured robot is as open as an unconfigured one.
- **The mode is stated twice**: once in the boot log right after NVS init (on
  every board, WiFi or not), and as `mqtt_commands=full|read-only` at the end
  of the self-report facts line that `robocar/status` publishes — which is
  where somebody whose MQTT drive command did nothing will look.
- **Only a command that will run wakes the planner, and only with full
  access.** A refused command has no effect at all, and an anonymous client
  polling `plan` must not hold the planner awake and spending. The wake still
  happens before dispatch, so `plan sleep` over MQTT is not undone by it.
- Command output is `printf` to the serial console, as on the console itself,
  so a status command sent over MQTT does not answer on MQTT; `robocar/status`
  is the MQTT-side read-back.

**`servo exercise` exists because "the servos are not moving" has five causes
that look identical from across the room**: no VCC on the PCA9685 (its 3.3 V
logic supply — a loose VCC lead was the actual cause on the 2026-09 bench), no V+
(servo power), a failed init, a pulse train outside the servos' frame rate, and a
dead servo. The exercise travels between the live `servo limit` values, so it
cannot drive past an end stop somebody has measured. Every step logs the angle, the PCA9685 count written and the bus
result, so a servo that does not move *while the writes succeed* is a different
diagnosis from one whose writes are failing — and `servo freq 50` settles the
frame-rate question without a reflash. That question is now answered for this
build: `robocar-bringup`'s rate ladder (2026-09-18) showed the SG90s fitted here
track cleanly at 50, 100 and 125 Hz and **buzz at 200**, so `PCA9685_FREQ_HZ` is
100 — one rung below the highest rate that worked, because the bench servos were
unloaded and a loaded one has less timing margin. Note the prescaler is
chip-wide: changing it moves motors and LEDs too.

**The wheels get the same treatment at boot, from `run_wheel_exercise()` in
`main.c`.** It drives forward then backward for `MOTOR_EXERCISE_HOLD_MS` (1 s) at
`MOTOR_EXERCISE_SPEED`, right after the executor comes up, so a drivetrain that
does not turn over is distinguishable from a silent everything-else before the
console is reachable. It goes through `reactive_controller_manual()` rather than
`motor_controller.h` — the executor stays the only writer of motor output, and
the obstacle reflex still governs, so a bench with a wall inside
`STOP_THRESHOLD_CM` holds the wheels and logs that it did. **The robot moves on
every boot**, which is a real hazard on a bench edge or with the wheels on the
floor; block it or put it on its back if that matters. There is no console
command for it yet, so re-running one means a reset.

The `sound`/`servo`/`led` commands are the only producers for `peripheral_task`'s queue — without them the task and every `PERIPH_CMD_*` case are unreachable.


## Don't

- Don't give `mqtt_command.c` a second, parallel notion of what a command does. It classifies a line and calls exactly two injected callbacks (`movement`, wired to `dispatch_movement()`; `console_line`, wired to `execute_console_line()`) — both already shared with the serial console. Adding logic that calls `motor_controller.c`, a peripheral driver, or a handler directly from `mqtt_command.c` or its callbacks would create a second entry point that can silently diverge from the console's, and for movement specifically would bypass the obstacle reflex the same way a direct `motor_controller.c` call does
- Don't widen the MQTT read-only allow-list to prefix matches, or add a command to it without reading its handler in `main.c` end to end. Read-only mode is the only thing between an anonymous LAN publisher and the robot (issue #626), and the handlers parse loosely enough that `voice <anything>` switches and persists the persona. Exact whole lines, each pinned in `test_read_only_allows_status_commands`
- Don't move the `mqtt` console command into `execute_console_line()`, or add `"mqtt"` to `mqtt_command.c`'s forwarded prefixes. It sets the broker credentials, so reaching it over MQTT would let an anonymous publisher unlock itself; `test_credential_command_is_never_reachable` pins it
- Don't skip `mqtt_command_extract_line()`'s fragmentation check (`current_data_offset != 0 || total_data_len != data_len`) when touching the MQTT command path. `event->data` is a fragment, not the whole payload, once a message exceeds esp-mqtt's buffer — the topic ("target") is resolved once on the first `MQTT_EVENT_DATA` callback, and dispatching on a later fragment acts on a truncated command before the rest of it has arrived
