# Cliff sensors (front and rear floor sensors)

Two downward-facing reflective sensors, one at the front and one at the rear,
stop the robot driving off a table edge. They are an interim fit: two
TCRT5000-style sensors salvaged from soap dispensers, until the VL53L4CD
time-of-flight pair planned in issue #1010 arrives. The reflex logic does not
depend on the sensor, so the ToF pair would feed the same decisions.

| File | Role |
|---|---|
| `main/cliff_core.{c,h}` | Debounce, reflex state machine, which motions are allowed. No ESP-IDF; host-tested in `test/test_cliff_core.c` |
| `main/cliff_sensor.{c,h}` | Reads both sensors from the MCP23017 in one I2C transaction |
| `main/reactive_controller.c` | Runs one cliff step per executor tick and arbitrates the wheels; integration tests in `test/test_reactive_controller.c` |

## Wiring

The sensor is a 4-pin through-hole part: an IR LED and a phototransistor side
by side, facing the same way. The salvaged parts are unmarked, so identify the
pins with a multimeter's diode test before wiring: the LED conducts one way
with a forward drop around 1.1-1.4 V; the phototransistor shows no diode
between its two pins. On TCRT5000-style parts the LED usually sits under the
blue (or clear) lens and the phototransistor under the black one.

| Sensor pin | Connect to | Notes |
|---|---|---|
| LED anode | 3.3 V through 100 Ω | ~20 mA with a ~1.2 V forward drop. From 5 V use about 180 Ω. Check the current against the part's datasheet if it can be identified |
| LED cathode | GND | |
| Phototransistor collector | MCP23017 `GPA0` (front) / `GPA1` (rear) | `CLIFF_FRONT_EXPANDER_PIN` / `CLIFF_REAR_EXPANDER_PIN` in `main/pin_config.h` |
| Phototransistor emitter | GND | |

The firmware enables the MCP23017's internal pull-up (about 100 kΩ) on both
pins, so no external resistor is needed on the collector. With floor under the
sensor the phototransistor conducts and pulls the pin low; with no floor it is
off and the pull-up holds the pin high. If the sensors read floor where there
is none (strong daylight, or a pull-up that is too weak against leakage), fit
an external 10-47 kΩ pull-up from the pin to 3.3 V: a stiffer pull-up needs
more reflected light to read as floor.

**An unplugged sensor reads as no floor.** That is intended: a broken wire stops
the robot moving toward that sensor rather than letting it drive blind.

### Mounting

- Close to the floor, pointing straight down. These sensors are built for
  millimetre-scale distances; start at about 5 mm and find the highest height
  that still reads floor on the darkest surface the robot drives on.
- At the front edge of the chassis, ahead of the wheels, and the same at the
  rear. The reflex can only stop the robot if the sensor reaches the edge before
  the wheels do.
- Shield the sensor from the side if daylight reaches it.

## Behaviour

Every executor tick (~33 ms) the firmware reads both pins.

- **Debounce.** A sensor is declared over a cliff after `CLIFF_TRIP_SAMPLES` (2)
  no-floor reads in a row, about 66 ms, and back on floor after
  `CLIFF_CLEAR_SAMPLES` (3).
- **Fail closed.** A failed I2C read counts as no floor under both sensors. The
  state also starts as no floor, so after boot or `cliff on` the wheels hold for
  about 100 ms until both sensors confirm the floor. Starting there means arming
  is never taken for an edge.
- **Reflex.** When a sensor loses the floor, the robot brakes for
  `CLIFF_BRAKE_MS` (150 ms), then backs away from the edge at
  `CLIFF_BACKOFF_SPEED` for `CLIFF_BACKOFF_MS` (400 ms): reverse for a front
  edge, forward for a rear edge. The back-off is skipped, or stopped, if the
  other sensor sees no floor, and a forward pull-away is skipped while the
  obstacle reflex holds the wheels. The horn sounds as for the obstacle reflex,
  with the same cooldown.
- **Refusal.** While a sensor sees no floor, any manual or planner motion toward
  it is replaced by a brake: forward needs front floor, backward needs rear
  floor, rotating in place needs both. Stopping is always allowed. The
  direction comes from the same classification the rangefinders use (#1026),
  so a drive goal past ±90°, which reverses (#1028), needs rear floor.
- **Order** (`apply_wheels()`): the OTA hold, then the cliff reflex, then the
  rangefinders, then the floor sensors' refusal, then the manual lease or the
  planner. A front edge reverses even with an obstacle ahead, because the
  obstacle is the other way.
- **Disarmed** with no MCP23017, or after `cliff off`: nothing is filtered.

The brake, back-off times and speed are estimates. The robot's stopping
distance at the executor's duties has not been measured (issue #1010), so check
them on the bench before trusting the reflex near a real edge.

## Console

| Command | Effect |
|---|---|
| `cliff` | Whether the reflex is on, whether the sensors are present, what each sees, whether the reflex is acting and the trip count. Read-only, so allowed over a read-only MQTT topic |
| `cliff on` / `cliff off` | Switch the reflex. On at every boot, not persisted |

The self-report carries `cliff=present|absent` (the pins were configured on a
detected MCP23017). Telemetry (`reactive_telemetry_t`) carries `cliff_armed`,
`cliff_front`, `cliff_rear`, `cliff_active` and `cliff_trips`.

## Bench checks before relying on it

1. With the robot lifted, `cliff` reads `NO FLOOR` on both; on the floor it
   reads `floor` on both. Repeat on every surface it drives on, the darkest
   and the shiniest included. Black or glossy surfaces that read as no floor
   stop the robot (safe, but it will not drive there).
2. In daylight, check a lifted sensor still reads `NO FLOOR`. Ambient infrared
   can turn the phototransistor on with nothing below it, which is the unsafe
   failure; an external pull-up or a side shield is the fix.
3. Drive slowly toward a table edge with a hand ready to catch the robot, and
   measure how far past the edge the sensor travels before the brake holds.
   Raise `CLIFF_BRAKE_MS`/`CLIFF_BACKOFF_MS` or mount the sensors farther
   forward if the wheels get close.
4. Unplug one sensor's collector wire: that direction should refuse to move.

## Don't

- **Don't start the state on floor.** Fail closed: a sensor that never answered
  must not permit motion, and starting on no floor is what keeps arming from
  looking like an edge. `test_starts_closed` and
  `test_cliff_holds_until_floor_confirmed` pin it.
- **Don't let a failed read count as floor.** `test_read_failure_fails_closed`
  and `test_cliff_read_failure_holds` pin it.
- **Don't filter motion in a second place.** The executor's `apply_wheels()` is
  the one arbiter for every wheel layer; `goal_motion()` mirrors
  `execute_goal()`'s branches and must change with it.
