/**
 * @file console_devices.c
 * @brief Console commands for devices and diagnostics: `gpio`, `cam`, `snap`, `trace`, `cliff` and
 * `ota`.
 *
 * Moved from main.c unchanged; execute_console_line() reaches each handler
 * through the console command table (console_cmd_core.h).
 */

#include <stdio.h>
#include <string.h>

#include "esp_err.h"

#include "activity_trace.h"
#include "camera.h"
#include "cliff_sensor.h"
#include "console_handlers.h"
#include "frame_dump.h"
#include "gpio_expander.h"
#include "i2c_bus.h"
#include "ota_manager.h"
#include "reactive_controller.h"

// ========================================
// GPIO expander console commands (bench testing)
//   gpio                     - 16-pin port dump
//   gpio mode <pin> in|up|out - set direction (up = input with pullup);
//                              pins 7 and 15 (GPA7/GPB7) are output-only
//   gpio set <pin> 0|1       - drive an output pin
//   gpio get <pin>           - read one pin

void handle_gpio_cmd(const char *buf)
{
    if (!gpio_expander_available()) {
        printf("gpio: no MCP23017 detected\n");
        return;
    }

    char op[8] = {0};
    unsigned pin = 0;
    char arg[8] = {0};
    int n = sscanf(buf, "gpio %7s %u %7s", op, &pin, arg);

    esp_err_t ret = ESP_ERR_INVALID_ARG;
    if (n <= 0) {
        uint16_t port = 0;
        ret = gpio_expander_read_port(&port);
        if (ret == ESP_OK)
            printf("gpio: port=0x%04X\n", port);
    } else if (n == 3 && strcmp(op, "mode") == 0) {
        gpio_expander_mode_t mode;
        if (strcmp(arg, "in") == 0)
            mode = GPIO_EXPANDER_INPUT;
        else if (strcmp(arg, "up") == 0)
            mode = GPIO_EXPANDER_INPUT_PULLUP;
        else if (strcmp(arg, "out") == 0)
            mode = GPIO_EXPANDER_OUTPUT;
        else {
            printf("gpio: mode must be in|up|out\n");
            return;
        }
        ret = gpio_expander_set_mode((uint8_t)pin, mode);
        if (ret == ESP_OK) {
            printf("gpio: pin %u mode=%s\n", pin, arg);
        } else if (ret == ESP_ERR_NOT_SUPPORTED) {
            printf("gpio: pin %u (%s) is output-only on the MCP23017 — use out\n", pin,
                   pin == 7 ? "GPA7" : "GPB7");
            return;
        }
    } else if (n == 3 && strcmp(op, "set") == 0) {
        ret = gpio_expander_write((uint8_t)pin, arg[0] != '0');
        if (ret == ESP_OK)
            printf("gpio: pin %u = %c\n", pin, arg[0] != '0' ? '1' : '0');
    } else if (n == 2 && strcmp(op, "get") == 0) {
        bool level = false;
        ret = gpio_expander_read((uint8_t)pin, &level);
        if (ret == ESP_OK)
            printf("gpio: pin %u = %d\n", pin, level);
    } else {
        printf("gpio: usage: gpio | gpio mode <pin> in|up|out | gpio set <pin> 0|1 | gpio get "
               "<pin>\n");
        return;
    }

    if (ret != ESP_OK)
        printf("gpio: error: %s\n", esp_err_to_name(ret));
}

/* Camera inspection and live exposure tuning.
 *
 * Exposure is deliberately a runtime knob rather than a compile-time constant:
 * whether a frame is "too dark" is a judgement only someone looking at the room
 * and the decoded image can make, and a reflash per trial is far too slow a
 * loop for it. `cam` prints the sensor's live registers, `snap` puts the actual
 * JPEG on the wire — tune against those two, then pin the winning value in
 * CAMERA_DEFAULT_GAINCEILING. */
void handle_cam_cmd(const char *buf)
{
    char op[16] = {0};
    int value = 0;
    const int n = sscanf(buf, "cam %15s %d", op, &value);

    if (n <= 0) {
        camera_exposure_t exp = {0};
        char buf_exp[64];
        camera_read_exposure(&exp);
        camera_format_exposure(buf_exp, sizeof(buf_exp), &exp);
        printf("cam: pid=%04x %s (gainceiling range 0-%d)\n", exp.pid, buf_exp,
               camera_gainceiling_max());
        printf("     orientation: flip=%s mirror=%s%s\n", camera_get_vflip() ? "on" : "off",
               camera_get_hmirror() ? "on" : "off",
               (camera_get_vflip() && camera_get_hmirror()) ? " (rotated 180)" : "");
        return;
    }

    esp_err_t ret;
    if (n == 2 && strcmp(op, "gainceiling") == 0) {
        ret = camera_set_gainceiling(value);
        if (ret == ESP_OK)
            printf("cam: gainceiling=%d\n", value);
    } else if (n == 2 && strcmp(op, "ae") == 0) {
        ret = camera_set_ae_level(value);
        if (ret == ESP_OK)
            printf("cam: ae_level=%d\n", value);
    } else if (n == 2 && strcmp(op, "brightness") == 0) {
        ret = camera_set_brightness(value);
        if (ret == ESP_OK)
            printf("cam: brightness=%d\n", value);
    } else if (strcmp(op, "flip") == 0 || strcmp(op, "mirror") == 0) {
        /* Parsed off the raw line rather than the %d above: "on"/"off" reads
         * better than 1/0 for a two-state control, and sscanf already failed to
         * fill `value` for them. */
        const bool on = (strstr(buf, " on") != NULL);
        const bool off = (strstr(buf, " off") != NULL);
        if (on == off) {
            printf("cam: usage: cam %s on|off  (currently %s)\n", op,
                   ((strcmp(op, "flip") == 0) ? camera_get_vflip() : camera_get_hmirror()) ? "on"
                                                                                           : "off");
            return;
        }
        ret = (strcmp(op, "flip") == 0) ? camera_set_vflip(on) : camera_set_hmirror(on);
        if (ret == ESP_OK) {
            printf("cam: %s=%s — the next frame will read as a scene change\n", op,
                   on ? "on" : "off");
        }
    } else {
        printf("cam: usage: cam | cam gainceiling 0-%d | cam ae -2..2 | cam brightness -2..2"
               " | cam flip on|off | cam mirror on|off\n",
               camera_gainceiling_max());
        return;
    }

    if (ret != ESP_OK)
        printf("cam: error: %s\n", esp_err_to_name(ret));
}

void handle_snap_cmd(const char *buf)
{
    int frames = 1;
    if (sscanf(buf, "snap %d", &frames) != 1) {
        frames = 1; /* bare `snap` */
    } else if (frames < 0) {
        frames = 0; /* `snap 0` disarms, per frame_dump_arm's contract */
    }
    frame_dump_arm(frames);
    if (frames == 0) {
        printf("snap: dumping disarmed\n");
    } else {
        printf("snap: will dump the next %d planner frame(s) — decode with "
               "tools/decode-frame-dump.py\n",
               frames);
    }
}

/**
 * @brief `trace` — what the robot has been capturing and calling since boot.
 *
 * The console half of activity_trace.h. The LED half is what you read while the
 * robot is driving; this is what you read afterwards to find out whether the
 * planner is being rate-limited, whether captures are silently failing, or which
 * endpoint is eating the wall clock between two planner lines.
 */
void handle_trace_cmd(const char *buf)
{
    if (strncmp(buf, "trace led on", 12) == 0) {
        activity_trace_set_leds(true);
        printf("trace: LED indicators on (left = camera, right = endpoints)\n");
        return;
    }
    if (strncmp(buf, "trace led off", 13) == 0) {
        activity_trace_set_leds(false);
        printf("trace: LED indicators off — counters still running\n");
        return;
    }
    if (strncmp(buf, "trace led", 9) == 0) {
        printf("trace: usage: trace led on|off  (currently %s)\n",
               activity_trace_leds_enabled() ? "on" : "off");
        return;
    }
    if (strncmp(buf, "trace reset", 11) == 0) {
        activity_trace_reset();
        i2c_bus_stats_reset();
        printf("trace: counters zeroed\n");
        return;
    }

    activity_trace_report();
    /* Printed alongside the camera and endpoint counters because it answers the
     * same kind of question about the third shared resource on this board. An
     * idle robot should read close to zero here; a steady rate with nothing
     * moving means something is writing the bus on a timer. */
    i2c_bus_stats_report();
}

/**
 * @brief `cliff` — the front and rear floor sensors and the cliff reflex.
 *
 * `cliff` alone prints whether the reflex is on and armed, what each sensor
 * sees and the trip count; it only reads, which is why it is on the MQTT
 * read-only list. `cliff on|off` switches the reflex. Not persisted: every
 * boot starts at CLIFF_REFLEX_AT_BOOT (docs/firmware/cliff-sensors.md).
 */
void handle_cliff_cmd(const char *buf)
{
    char state[8] = {0};
    const int n = sscanf(buf, "cliff %7s", state);

    if (n == 1 && (strcmp(state, "on") == 0 || strcmp(state, "off") == 0)) {
        const bool on = (strcmp(state, "on") == 0);
        reactive_controller_set_cliff(on);
        printf("cliff: reflex %s\n",
               on ? "ON — the wheels hold until both sensors confirm the floor"
                  : "OFF — nothing stops the robot at an edge");
        return;
    }
    if (n > 0) {
        printf("cliff: usage: cliff | cliff on|off\n");
        return;
    }

    reactive_telemetry_t tele = {0};
    const bool have_tele = (reactive_controller_get_telemetry(&tele) == ESP_OK);
    printf("cliff: reflex=%s sensors=%s\n", reactive_controller_cliff_enabled() ? "on" : "off",
           cliff_sensor_available() ? "present" : "absent(no MCP23017)");
    if (have_tele && tele.cliff_armed) {
        printf("  front: %s  rear: %s  active: %s  trips: %lu\n",
               tele.cliff_front ? "NO FLOOR" : "floor", tele.cliff_rear ? "NO FLOOR" : "floor",
               tele.cliff_active ? "yes" : "no", (unsigned long)tele.cliff_trips);
    } else {
        printf("  disarmed: nothing is filtered\n");
    }
    printf("  usage: cliff on|off\n");
}

/**
 * @brief `ota` console command: the pre-install sequence's state, a check now,
 *        or a postpone of an announced update.
 *
 * `ota` prints the running version and build, and where the sequence is.
 * `ota check` runs a manifest check now (the same nudge MQTT sends).
 * `ota postpone` defers an announced update to the next check; accepted only
 * in the minute after the robot says it is going (ota_announce.h).
 */
void handle_ota_cmd(const char *buf)
{
    char sub[12] = {0};
    const int n = sscanf(buf, "ota %11s", sub);

    if (n == 1 && strcmp(sub, "postpone") == 0) {
        if (ota_manager_postpone() == ESP_OK) {
            printf("ota: postponed to the next check\n");
        } else {
            printf("ota: nothing to postpone (no update announced in the last minute)\n");
        }
        return;
    }
    if (n == 1 && strcmp(sub, "check") == 0) {
        const esp_err_t err = ota_manager_check_update();
        printf("ota: check %s\n", err == ESP_OK ? "queued" : esp_err_to_name(err));
        return;
    }
    if (n == 1) {
        printf("ota: usage: ota | ota check | ota postpone\n");
        return;
    }

    char pending[32] = {0};
    ota_manager_pending_version(pending, sizeof(pending));
    const ota_phase_t phase = ota_manager_phase();
    printf("ota: running %s (build %s)\n", ota_manager_get_version(), ota_manager_get_build_sha());
    printf("  update: %s%s%s\n", ota_manager_phase_name(phase),
           (phase != OTA_PHASE_IDLE && pending[0]) ? " — " : "",
           (phase != OTA_PHASE_IDLE && pending[0]) ? pending : "");
    printf("  usage: ota check | ota postpone\n");
}
