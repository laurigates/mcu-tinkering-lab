/**
 * @file main.c
 * @brief Robocar firmware for XIAO ESP32-S3 Sense
 *
 * Hierarchical AI controller architecture:
 *   - Core 0: motor control, peripheral I/O, serial commands, reactive executor (30 Hz)
 *   - Core 1: camera capture, Gemini planner (PLANNER_LOOP_PERIOD_MS, 15 s default), WiFi/MQTT/OTA
 *
 * The planner (planner_task, Core 1) captures frames, calls Gemini ER 2,
 * and writes structured goals into goal_state.  The reactive executor
 * (reactive_controller, Core 0) reads goals at 30 Hz and drives motors.
 * Serial / MQTT commands remain available for manual override.
 */

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"

#include "activity_trace.h"
#include "ambient_audio.h"
#include "ambient_listener.h"
#include "audio_clip.h"
#include "audio_player.h"
#include "buzzer.h"
#include "camera.h"
#include "cliff_sensor.h"
#include "config.h"
#include "console_cmd_core.h"
#include "console_handlers.h"
#include "console_line.h"
#include "credentials_loader.h"
#include "credentials_validator.h"
#include "dialogue_style.h"
#include "event_report.h"
#include "frame_dump.h"
#include "gemini_key_state.h"
#include "gemini_tts.h"
#include "goal_state.h"
#include "gpio_expander.h"
#include "i2c_bus.h"
#include "improv_console.h"
#include "improv_wifi.h"
#include "led_controller.h"
#include "log_mirror.h"
#include "mdns.h"
#include "mic_dump.h"
#include "mic_pdm.h"
#include "net_supervisor_core.h"
#include "note_gate.h"
/* motor_controller_init() in the hardware phase and motor_is_initialized() in
 * the boot wheel exercise — both reads. Motor *output* is the reactive
 * executor's job, and nothing here writes the motors directly. */
#include "live_session.h"
#include "motor_controller.h"
#include "mqtt_command.h"
#include "mqtt_logger.h"
#include "oled_status.h"
#include "ota_manager.h"
#include "pin_config.h"
#include "plan_activity.h"
#include "plan_budget.h"
#include "planner_task.h"
#include "power_latch.h"
#include "power_manager.h"
#include "quest_log_core.h"
#include "quest_store.h"
#include "reactive_controller.h"
#include "scene_change.h"
#include "self_report.h"
#include "servo_controller.h"
#include "sound_clips_core.h"
#include "speech_budget.h"
#include "speech_queue.h"
#include "speech_trigger.h"
#include "voice_budget.h"
#include "voice_persona.h"
#include "voice_turn.h"
#include "wall_clock.h"
#include "wifi_manager.h"

static const char *TAG = "robocar";

// ========================================
// Command types for FreeRTOS queues
// ========================================
typedef enum {
    MOTOR_CMD_FORWARD,
    MOTOR_CMD_BACKWARD,
    MOTOR_CMD_LEFT,
    MOTOR_CMD_RIGHT,
    MOTOR_CMD_ROTATE_CW,
    MOTOR_CMD_ROTATE_CCW,
    MOTOR_CMD_STOP,
} motor_cmd_type_t;

typedef struct {
    motor_cmd_type_t type;
    uint8_t speed;
} motor_cmd_t;

typedef enum {
    PERIPH_CMD_LED_COLOR,
    PERIPH_CMD_SERVO_PAN,
    PERIPH_CMD_SERVO_TILT,
    PERIPH_CMD_SERVO_EXERCISE,
    PERIPH_CMD_SOUND_BEEP,
    PERIPH_CMD_SOUND_MELODY,
    PERIPH_CMD_SOUND_ALERT,
} periph_cmd_type_t;

typedef struct {
    periph_cmd_type_t type;
    union {
        struct {
            uint8_t r, g, b;
            led_position_t position;
        } led;
        int16_t angle;
    };
} periph_cmd_t;

// ========================================
// Global state
// ========================================
static QueueHandle_t s_motor_queue;
static QueueHandle_t s_periph_queue;
/* Written by the network supervisor task, read by command_task (issue #846).
 * Bumped on every Improv arm and every disarm, so it is odd while armed
 * (console_line_improv_armed()) and command_task sees each edge, even a
 * disarm and re-arm between two console bytes, and finishes a packet in
 * flight across it (issue #1087). One aligned 32-bit word: a single store and load. */
static volatile uint32_t s_improv_session;

/** Map a queued console command onto the executor's manual-override vocabulary. */
static reactive_manual_cmd_t to_manual_cmd(motor_cmd_type_t type)
{
    switch (type) {
        case MOTOR_CMD_FORWARD:
            return REACTIVE_MANUAL_FORWARD;
        case MOTOR_CMD_BACKWARD:
            return REACTIVE_MANUAL_BACKWARD;
        case MOTOR_CMD_LEFT:
            return REACTIVE_MANUAL_LEFT;
        case MOTOR_CMD_RIGHT:
            return REACTIVE_MANUAL_RIGHT;
        case MOTOR_CMD_ROTATE_CW:
            return REACTIVE_MANUAL_ROTATE_CW;
        case MOTOR_CMD_ROTATE_CCW:
            return REACTIVE_MANUAL_ROTATE_CCW;
        case MOTOR_CMD_STOP:
        default:
            return REACTIVE_MANUAL_STOP;
    }
}

// ========================================
// Motor control task (Core 0, priority 6)
// ========================================
/* Console movement commands are handed to the reactive executor rather than
 * written to the motors here. Two tasks writing the same PCA9685 channels meant
 * the 30 Hz executor overwrote every console command within 33 ms — and it
 * violated the project invariant that the executor owns motor output. Routing
 * through reactive_controller_manual() also puts console driving behind the
 * obstacle reflex, and its lease replaces the COMMAND_TIMEOUT_MS watchdog that
 * used to live in this loop. */
static void motor_control_task(void *pvParameters)
{
    (void)pvParameters;
    motor_cmd_t cmd;

    ESP_LOGI(TAG, "Motor control task started on core %d", xPortGetCoreID());

    while (1) {
        if (xQueueReceive(s_motor_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_err_t ret = reactive_controller_manual(to_manual_cmd(cmd.type), cmd.speed,
                                                       REACTIVE_MANUAL_TTL_MS);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Manual command dropped: %s", esp_err_to_name(ret));
            }
        }
    }
}

// ========================================
// Peripheral task (Core 0, priority 4)
// ========================================
static void peripheral_task(void *pvParameters)
{
    (void)pvParameters;
    periph_cmd_t cmd;

    ESP_LOGI(TAG, "Peripheral task started on core %d", xPortGetCoreID());

    TickType_t last_oled = xTaskGetTickCount();

    while (1) {
#if FEATURE_OLED_ENABLED
        /* The status screen redraws between commands. oled_flush() writes only
         * the rows that changed, one bus acquisition per row. */
        if (xTaskGetTickCount() - last_oled >= pdMS_TO_TICKS(OLED_STATUS_REFRESH_MS)) {
            last_oled = xTaskGetTickCount();
            oled_status_refresh();
        }
#else
        (void)last_oled;
#endif
        if (xQueueReceive(s_periph_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (cmd.type) {
                case PERIPH_CMD_LED_COLOR:
                    led_set_rgb(cmd.led.position, cmd.led.r, cmd.led.g, cmd.led.b);
                    break;
                case PERIPH_CMD_SERVO_PAN:
                    servo_set_pan(cmd.angle);
                    break;
                case PERIPH_CMD_SERVO_TILT:
                    servo_set_tilt(cmd.angle);
                    break;
                case PERIPH_CMD_SERVO_EXERCISE: {
                    /* Runs here rather than on the console task because it
                     * blocks for several seconds. The executor owns the head,
                     * so it is told to keep its hands off for the duration —
                     * otherwise it would re-centre over every step. */
                    const bool leased = (reactive_controller_head_external(0) == ESP_OK);
                    servo_exercise();
                    if (leased) {
                        reactive_controller_head_release();
                    }
                    break;
                }
                case PERIPH_CMD_SOUND_BEEP:
                    buzzer_beep();
                    break;
                case PERIPH_CMD_SOUND_MELODY:
                    buzzer_melody();
                    break;
                case PERIPH_CMD_SOUND_ALERT:
                    buzzer_alert();
                    break;
            }
        }
    }
}

// ========================================
// Dispatch helpers (used by serial command task)
// ========================================
static void dispatch_motor_cmd(motor_cmd_type_t type, uint8_t speed)
{
    if (!s_motor_queue) {
        return;
    }
    motor_cmd_t cmd = {.type = type, .speed = speed};
    xQueueSend(s_motor_queue, &cmd, pdMS_TO_TICKS(100));
}

static void dispatch_periph_cmd(const periph_cmd_t *cmd)
{
    if (!s_periph_queue) {
        return;
    }
    xQueueSend(s_periph_queue, cmd, pdMS_TO_TICKS(100));
}

static void dispatch_movement(const char *movement)
{
    if (!movement)
        return;

    uint8_t speed = (uint8_t)(DEFAULT_SPEED * 255 / PCA9685_PWM_MAX);

    if (strcmp(movement, "forward") == 0)
        dispatch_motor_cmd(MOTOR_CMD_FORWARD, speed);
    else if (strcmp(movement, "backward") == 0)
        dispatch_motor_cmd(MOTOR_CMD_BACKWARD, speed);
    else if (strcmp(movement, "left") == 0)
        dispatch_motor_cmd(MOTOR_CMD_LEFT, speed);
    else if (strcmp(movement, "right") == 0)
        dispatch_motor_cmd(MOTOR_CMD_RIGHT, speed);
    else if (strcmp(movement, "rotate_cw") == 0)
        dispatch_motor_cmd(MOTOR_CMD_ROTATE_CW, speed);
    else if (strcmp(movement, "rotate_ccw") == 0)
        dispatch_motor_cmd(MOTOR_CMD_ROTATE_CCW, speed);
    else if (strcmp(movement, "stop") == 0)
        dispatch_motor_cmd(MOTOR_CMD_STOP, 0);
}

/**
 * @brief Boot diagnostic: drive forward, then backward, then stop.
 *
 * The point is the one thing a log line cannot prove on its own — that the
 * drivetrain turns over — before anything downstream (WiFi, the planner, the
 * spoken self-report) has a chance to claim the robot is ready.
 *
 * It goes through reactive_controller_manual() rather than motor_controller.h
 * for the two reasons that API exists, and both are load-bearing here: the
 * executor stays the only writer of motor output (see the "Rules" list in
 * CLAUDE.md), and the obstacle reflex still governs. A bench with a wall inside
 * STOP_THRESHOLD_CM will hold the wheels stopped — the correct outcome, and one
 * that is logged rather than silently swallowed, because a diagnostic that
 * quietly does nothing is worse than no diagnostic.
 *
 * Blocking, ~2 s. Called once from app_main after the executor is up; this is a
 * boot step, not control logic, so it does not belong on the executor task.
 */
static void run_wheel_exercise(void)
{
    if (!motor_is_initialized()) {
        ESP_LOGW(TAG, "wheel exercise: skipped — motors not initialised");
        return;
    }

    const uint8_t speed = MOTOR_EXERCISE_SPEED;
    const uint32_t hold_ms = MOTOR_EXERCISE_HOLD_MS;
    /* The lease has to outlive the delay it is driving: a TTL of exactly
     * hold_ms lets the executor drop the command a tick or two before the next
     * one is issued, so the wheels coast through the hand-off. */
    const uint32_t ttl_ms = hold_ms + REACTIVE_MANUAL_TTL_MS;

    esp_err_t ret = reactive_controller_manual(REACTIVE_MANUAL_FORWARD, speed, ttl_ms);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "wheel exercise: unavailable (%s)", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "wheel exercise: forward %u ms at %u/255", (unsigned)hold_ms, (unsigned)speed);
    /* Sample halfway through rather than at the end: a reflex that latched
     * during the pulse is what explains wheels that did not turn, and by the
     * time the pulse is over the reading may already have cleared. */
    vTaskDelay(pdMS_TO_TICKS(hold_ms / 2));
    reactive_telemetry_t tele = {0};
    if (reactive_controller_get_telemetry(&tele) == ESP_OK && tele.reflex_active) {
        ESP_LOGW(TAG,
                 "wheel exercise: obstacle reflex active (front mean %u cm) — wheels held stopped",
                 (unsigned)tele.distance_cm);
    }
    vTaskDelay(pdMS_TO_TICKS(hold_ms - hold_ms / 2));

    reactive_controller_manual(REACTIVE_MANUAL_BACKWARD, speed, ttl_ms);
    ESP_LOGI(TAG, "wheel exercise: backward %u ms at %u/255", (unsigned)hold_ms, (unsigned)speed);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));

    /* STOP ends the lease immediately instead of holding the motors stopped for
     * a further TTL, so the planner can resume on the next executor tick. */
    reactive_controller_manual(REACTIVE_MANUAL_STOP, 0, 0);
    ESP_LOGI(TAG, "wheel exercise: done");
}

static void dispatch_sound(const char *sound)
{
    if (!sound)
        return;

    periph_cmd_t cmd;
    if (strcmp(sound, "beep") == 0)
        cmd.type = PERIPH_CMD_SOUND_BEEP;
    else if (strcmp(sound, "melody") == 0)
        cmd.type = PERIPH_CMD_SOUND_MELODY;
    else if (strcmp(sound, "alert") == 0)
        cmd.type = PERIPH_CMD_SOUND_ALERT;
    else
        return;

    dispatch_periph_cmd(&cmd);
}

/* Console access to the peripheral queue.
 *
 *   sound beep|melody|alert
 *   servo pan|tilt <deg>
 *   led <r> <g> <b>
 *
 * These exist because peripheral_task, its queue and every PERIPH_CMD_* case
 * were unreachable — nothing in the firmware ever built a periph_cmd_t. The
 * hardware is real and wired (buzzer on GPIO2, pan/tilt servos and both RGB
 * LEDs on the PCA9685), so the queue is given the bench commands it was
 * evidently built for rather than being deleted along with the drivers.
 */
static void handle_periph_cmd(const char *buf)
{
    char arg[16] = {0};
    int a = 0, b = 0, c = 0;

    if (strncmp(buf, "sound", 5) == 0) {
        static const char k_sound_usage[] =
            "sound: usage: sound beep|melody|alert (buzzer) | sound horn|aoogah|bell|boing "
            "(speaker)\n";
        if (sscanf(buf, "sound %15s", arg) != 1) {
            printf("%s", k_sound_usage);
            return;
        }
        /* A speaker clip goes through the speech queue as a tag-only line,
         * the same path the planner's `sound` tool takes, so it waits for any
         * sentence in progress instead of talking over it. */
        const sound_clip_t clip = sound_clip_from_name(arg);
        if (clip != SOUND_CLIP_NONE) {
            char line[24];
            snprintf(line, sizeof(line), "[%s]", sound_clip_name(clip));
            const esp_err_t err = speech_queue_post(line);
            printf("sound: %s %s\n", arg, err == ESP_OK ? "queued" : "dropped (speech queue full)");
            return;
        }
        if (strcmp(arg, "beep") != 0 && strcmp(arg, "melody") != 0 && strcmp(arg, "alert") != 0) {
            printf("%s", k_sound_usage);
            return;
        }
        dispatch_sound(arg);
        printf("sound: %s\n", arg);
        return;
    }

    if (strncmp(buf, "servo", 5) == 0) {
        static const char k_servo_usage[] =
            "servo: usage: servo | servo pan|tilt <deg> | servo on|off | "
            "servo limit pan|tilt <min> <max> | servo exercise | servo freq <24-1526>\n";
        const int n = sscanf(buf, "servo %15s %d", arg, &a);

        if (n <= 0) {
            /* State first: most of the ways this goes wrong are visible here,
             * and the rest (nothing moves while every write succeeds) is what
             * `servo exercise` is for. */
            servo_position_t pos = {0};
            int16_t pan_lo = 0, pan_hi = 0, tilt_lo = 0, tilt_hi = 0;
            const bool ready = servo_is_initialized();
            servo_get_position(&pos);
            servo_get_limits(SERVO_PAN, &pan_lo, &pan_hi);
            servo_get_limits(SERVO_TILT, &tilt_lo, &tilt_hi);
            printf("servo: %s | pca9685=%u Hz\n", ready ? "ready" : "NOT INITIALISED",
                   (unsigned)i2c_bus_pca9685_frequency());
            printf("       pan=%+d deg (count %u, %s, limits %+d..%+d)\n", pos.pan_angle,
                   (unsigned)servo_angle_to_count(SERVO_PAN, pos.pan_angle),
                   servo_is_enabled(SERVO_PAN) ? "on" : "off", pan_lo, pan_hi);
            printf("       tilt=%+d deg (count %u, %s, limits %+d..%+d)\n", pos.tilt_angle,
                   (unsigned)servo_angle_to_count(SERVO_TILT, pos.tilt_angle),
                   servo_is_enabled(SERVO_TILT) ? "on" : "off", tilt_lo, tilt_hi);
            reactive_telemetry_t tele = {0};
            if (reactive_controller_get_telemetry(&tele) == ESP_OK) {
                printf("       executor head: %s\n", reactive_head_mode_name(tele.head_mode));
            }
            printf("%s", k_servo_usage);
            return;
        }

        /* on/off run here rather than on peripheral_task, so `servo off` still
         * releases the outputs while an exercise is blocking that queue. */
        if (strcmp(arg, "off") == 0) {
            const esp_err_t ret = servo_disable_all();
            if (ret == ESP_OK) {
                printf("servo: off — pan and tilt outputs released (no pulses)\n");
            } else {
                printf("servo: off failed: %s\n", esp_err_to_name(ret));
            }
            return;
        }

        if (strcmp(arg, "on") == 0) {
            const esp_err_t ret = servo_enable_all();
            if (ret == ESP_OK) {
                servo_position_t pos = {0};
                servo_get_position(&pos);
                printf("servo: on — holding pan=%+d tilt=%+d deg\n", pos.pan_angle, pos.tilt_angle);
            } else {
                printf("servo: on failed: %s\n", esp_err_to_name(ret));
            }
            return;
        }

        if (strcmp(arg, "limit") == 0) {
            char which[8] = {0};
            int lo = 0;
            int hi = 0;
            if (sscanf(buf, "servo limit %7s %d %d", which, &lo, &hi) != 3 ||
                (strcmp(which, "pan") != 0 && strcmp(which, "tilt") != 0)) {
                printf("servo: usage: servo limit pan|tilt <min> <max>\n");
                return;
            }
            const servo_id_t id = (strcmp(which, "pan") == 0) ? SERVO_PAN : SERVO_TILT;
            const esp_err_t ret =
                (lo < INT16_MIN || lo > INT16_MAX || hi < INT16_MIN || hi > INT16_MAX)
                    ? ESP_ERR_INVALID_ARG
                    : servo_set_limits(id, (int16_t)lo, (int16_t)hi);
            if (ret == ESP_OK) {
                printf("servo: %s limits %+d..%+d deg — does not persist\n", which, lo, hi);
            } else {
                int16_t hw_lo = 0, hw_hi = 0;
                servo_get_hw_range(&hw_lo, &hw_hi);
                printf("servo: limit rejected (%s) — need %+d <= min <= 0 <= max <= %+d, "
                       "min < max\n",
                       esp_err_to_name(ret), hw_lo, hw_hi);
            }
            return;
        }

        if (strcmp(arg, "exercise") == 0) {
            periph_cmd_t cmd = {.type = PERIPH_CMD_SERVO_EXERCISE};
            dispatch_periph_cmd(&cmd);
            printf("servo: exercise queued — shake then nod within the limits, ~6 s; watch the "
                   "log for each write\n");
            return;
        }

        if (n == 2 && strcmp(arg, "freq") == 0) {
            const esp_err_t ret =
                (a < 0 || a > 0xFFFF) ? ESP_ERR_INVALID_ARG : servo_set_pwm_frequency((uint16_t)a);
            if (ret == ESP_OK) {
                printf("servo: pca9685=%d Hz — servo pulses re-sent; affects motors and LEDs "
                       "too, and does not persist\n",
                       a);
            } else {
                printf("servo: freq failed: %s\n", esp_err_to_name(ret));
            }
            return;
        }

        if (n != 2) {
            printf("%s", k_servo_usage);
            return;
        }

        periph_cmd_t cmd = {0};
        servo_id_t id;
        if (strcmp(arg, "pan") == 0) {
            cmd.type = PERIPH_CMD_SERVO_PAN;
            id = SERVO_PAN;
        } else if (strcmp(arg, "tilt") == 0) {
            cmd.type = PERIPH_CMD_SERVO_TILT;
            id = SERVO_TILT;
        } else {
            printf("%s", k_servo_usage);
            return;
        }

        /* Checked here as well as in servo_set_angle(), because the move itself
         * runs later on peripheral_task and its refusal would be silent. */
        if (!servo_is_initialized()) {
            printf("servo: NOT INITIALISED — nothing sent\n");
            return;
        }
        if (a < INT16_MIN || a > INT16_MAX || !servo_is_angle_valid(id, (int16_t)a)) {
            int16_t lo = 0, hi = 0;
            servo_get_limits(id, &lo, &hi);
            printf("servo: %s=%d rejected — limits are %+d..%+d deg (`servo limit %s <min> "
                   "<max>`)\n",
                   arg, a, lo, hi, arg);
            return;
        }
        if (!servo_is_enabled(id)) {
            printf("servo: %s is off — `servo on` first\n", arg);
            return;
        }

        /* The executor owns the head (issue #511), so a console move is a
         * lease it applies, not a second writer it would overwrite. Only when
         * the executor is not running — nothing else is writing the servos —
         * does the move go straight to peripheral_task as before. */
        const esp_err_t lease_ret = reactive_controller_head_hold(
            (id == SERVO_PAN) ? REACTIVE_HEAD_PAN : REACTIVE_HEAD_TILT, (int16_t)a, 0);
        if (lease_ret == ESP_OK) {
            printf("servo: %s=%d (count %u) — held %u s, then the executor takes the head back\n",
                   arg, a, (unsigned)servo_angle_to_count(id, (int16_t)a),
                   (unsigned)(REACTIVE_HEAD_HOLD_TTL_MS / 1000U));
            return;
        }
        cmd.angle = (int16_t)a;
        dispatch_periph_cmd(&cmd);
        printf("servo: %s=%d (count %u)\n", arg, a, (unsigned)servo_angle_to_count(id, (int16_t)a));
        return;
    }

    if (sscanf(buf, "led %d %d %d", &a, &b, &c) != 3) {
        printf("led: usage: led <r> <g> <b>   (0-255)\n");
        return;
    }
    periph_cmd_t cmd = {.type = PERIPH_CMD_LED_COLOR};
    cmd.led.r = (uint8_t)a;
    cmd.led.g = (uint8_t)b;
    cmd.led.b = (uint8_t)c;
    cmd.led.position = LED_BOTH;
    dispatch_periph_cmd(&cmd);
    printf("led: rgb=%d,%d,%d\n", a, b, c);
}

// ========================================
// Improv WiFi provisioning (serial)
// ========================================
/* Improv Serial rides the console — the USB-Serial-JTAG, not UART0: ESP Web
 * Tools opens that port after flashing and speaks the protocol over it, which
 * is why the byte feed lives inside command_task rather than in a task of its
 * own. Replies go back over the same port through improv_console_write()
 * (issue #644); the component's UART0 default would send them to D6/D7. */
/* Credentials Improv handed over, waiting for the link to prove them. Written
 * by command_task (the Improv callback), consumed by the network supervisor;
 * the flag is published with release/acquire ordering after the buffers. */
static char s_improv_ssid[33];
static char s_improv_pass[65];
static bool s_improv_pending;

/* Non-blocking since issue #846: the callback runs on command_task, inside
 * the console byte loop, and used to block it for the whole connect attempt.
 * It now starts the attempt and returns; net_supervisor_task() saves the
 * credentials on GOT_IP or reports failure once the fast retries are spent. */
static void on_improv_credentials(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Improv: credentials received for SSID '%s'", ssid ? ssid : "");
    improv_wifi_send_state(IMPROV_STATE_PROVISIONING);

    if (!ssid || ssid[0] == '\0' || strlen(ssid) >= sizeof(s_improv_ssid) ||
        (password && strlen(password) >= sizeof(s_improv_pass))) {
        improv_wifi_fail_provisioning(IMPROV_ERROR_INVALID_RPC);
        return;
    }
    if (__atomic_load_n(&s_improv_pending, __ATOMIC_ACQUIRE)) {
        ESP_LOGW(TAG, "Improv: an attempt is already in progress");
        return;
    }

    strlcpy(s_improv_ssid, ssid, sizeof(s_improv_ssid));
    strlcpy(s_improv_pass, password ? password : "", sizeof(s_improv_pass));
    if (wifi_connect(s_improv_ssid, s_improv_pass) != ESP_OK) {
        improv_wifi_fail_provisioning(IMPROV_ERROR_UNABLE_CONNECT);
        return;
    }
    __atomic_store_n(&s_improv_pending, true, __ATOMIC_RELEASE);
}

/**
 * @brief Execute one already-trimmed, multi-word console command line.
 *
 * Shared by the serial console (command_task, below) and the MQTT command
 * topic (mqtt_command_received(), via start_mqtt_service()), so the two
 * entry points can never diverge on what a command line does. Single-letter
 * movement (F/B/L/R/C/W/S) is handled separately by each caller instead —
 * see command_task()'s buf_pos==1 switch and mqtt_command.c's
 * movement_word_for() — because the two reach it through different framing
 * (a bare console keystroke vs. an MQTT payload naming a movement word) and
 * both already route to dispatch_movement().
 *
 * A command that must stay serial-only (`mqtt`, which sets the broker
 * credentials) does not belong here; command_task() handles it before this.
 */
/**
 * @brief `motors` — the TB6612FNG standby line and the wheel exercise.
 *
 * `motors standby on|off` holds the driver in standby or enables it; the
 * executor keeps writing the PCA9685 either way. `motors exercise` repeats the
 * boot wheel exercise. Together they are the bench control for issue #944: the
 * same writes with and without motor current. Not persisted: the driver is
 * enabled at every boot.
 */
static void handle_motors_cmd(const char *buf)
{
    char verb[10] = {0};
    char state[8] = {0};
    const int n = sscanf(buf, "motors %9s %7s", verb, state);

    if (n == 2 && strcmp(verb, "standby") == 0 &&
        (strcmp(state, "on") == 0 || strcmp(state, "off") == 0)) {
        const bool on = (strcmp(state, "on") == 0);
        const esp_err_t ret = motor_set_standby(on);
        if (ret != ESP_OK) {
            printf("motors: standby unavailable (%s)\n", esp_err_to_name(ret));
            return;
        }
        printf("motors: driver %s\n",
               on ? "in STANDBY — the wheels will not turn; PCA9685 writes continue" : "enabled");
        return;
    }
    if (n == 1 && strcmp(verb, "exercise") == 0) {
        if (motor_in_standby()) {
            printf("motors: exercise with the driver in standby — the wheels will not turn\n");
        }
        run_wheel_exercise();
        return;
    }
    if (n > 0) {
        printf("motors: usage: motors | motors standby on|off | motors exercise\n");
        return;
    }

    printf("motors: initialised=%s driver=%s\n", motor_is_initialized() ? "yes" : "no",
           motor_in_standby() ? "standby" : "enabled");
    printf("  usage: motors standby on|off | motors exercise\n");
}

static void execute_console_line(const char *buf)
{
    /* No default: -Wswitch names a table command without a handler here. */
    switch (console_cmd_lookup(buf)) {
        case CONSOLE_CMD_NONE:
        case CONSOLE_CMD_COUNT:
            break;
        case CONSOLE_CMD_GPIO:
            handle_gpio_cmd(buf);
            break;
        case CONSOLE_CMD_VOICE:
            handle_voice_cmd(buf);
            break;
        case CONSOLE_CMD_SNAP:
            handle_snap_cmd(buf);
            break;
        case CONSOLE_CMD_LISTEN:
            handle_listen_cmd(buf);
            break;
        case CONSOLE_CMD_TRACE:
            handle_trace_cmd(buf);
            break;
        case CONSOLE_CMD_MIC:
            handle_mic_cmd(buf);
            break;
        case CONSOLE_CMD_CAM:
            handle_cam_cmd(buf);
            break;
        case CONSOLE_CMD_PLAN:
            handle_plan_cmd(buf);
            break;
        case CONSOLE_CMD_IDLE:
            handle_idle_cmd(buf);
            break;
        case CONSOLE_CMD_CLIFF:
            handle_cliff_cmd(buf);
            break;
        case CONSOLE_CMD_MOTORS:
            handle_motors_cmd(buf);
            break;
        case CONSOLE_CMD_OTA:
            handle_ota_cmd(buf);
            break;
        case CONSOLE_CMD_REPORT:
            handle_report_cmd(buf);
            break;
        case CONSOLE_CMD_QUEST:
            handle_quest_cmd(buf);
            break;
        case CONSOLE_CMD_PERIPH:
            handle_periph_cmd(buf);
            break;
    }
}

/* MQTT command-topic access mode, fixed for the boot (issue #626). Read-only
 * until init_mqtt_access() finds a complete pair of broker credentials. */
static mqtt_command_access_t s_mqtt_access = MQTT_COMMAND_ACCESS_READ_ONLY;

/**
 * @brief Load the broker credentials once and log the MQTT command mode.
 *
 * Called right after NVS init, so the boot log always states the mode — even
 * on a board with no WiFi, where MQTT never starts — and so the credential
 * load happens on one task before anything else can race it.
 */
static void init_mqtt_access(void)
{
    s_mqtt_access = mqtt_command_access_for_credentials(get_mqtt_username(), get_mqtt_password());
    if (s_mqtt_access == MQTT_COMMAND_ACCESS_FULL) {
        ESP_LOGI(TAG, "MQTT commands: full (broker credentials from %s, user %s)",
                 get_mqtt_credentials_source(), get_mqtt_username());
    } else {
        ESP_LOGW(TAG,
                 "MQTT commands: read-only — no broker credentials, so movement and settings "
                 "are refused on %s (set them with `mqtt auth <user> <pass>`)",
                 MQTT_COMMAND_TOPIC);
    }
}

/**
 * @brief mqtt_command_ops_t::movement callback for the MQTT command topic.
 *
 * Reuses dispatch_movement() unchanged — the same function the console's
 * single-letter commands call — so an MQTT-originated drive command reaches
 * reactive_controller_manual() via the existing motor queue + executor lease,
 * never motor_controller.c directly.
 */
static void mqtt_cmd_movement(const char *word, void *ctx)
{
    (void)ctx;
    dispatch_movement(word);
}

/**
 * @brief mqtt_command_ops_t::console_line callback for the MQTT command topic.
 *
 * Reuses execute_console_line() unchanged — the same dispatch the serial
 * console uses for everything past single-letter movement.
 */
static void mqtt_cmd_console_line(const char *line, void *ctx)
{
    (void)ctx;
    execute_console_line(line);
}

/**
 * @brief Registered with mqtt_logger via mqtt_logger_set_command_handler().
 *
 * mqtt_logger.c has already bound-checked and validated the payload into a
 * NUL-terminated line (mqtt_command_extract_line()) before calling here; this
 * only classifies it and routes it through the same handlers the serial
 * console uses. An unrecognised line is logged and dropped rather than
 * silently ignored (mqtt_command_dispatch() dispatches neither callback).
 */
static void mqtt_command_received(const char *line)
{
    const mqtt_command_ops_t ops = {
        .movement = mqtt_cmd_movement,
        .console_line = mqtt_cmd_console_line,
        .ctx = NULL,
        .access = s_mqtt_access,
    };

    /* Classify before anything acts on the line, so a refused command has no
     * effect at all — including the planner wake below, which costs Gemini
     * requests. */
    const mqtt_command_result_t verdict = mqtt_command_check(line, s_mqtt_access);
    if (verdict != MQTT_COMMAND_OK) {
        ESP_LOGW(TAG, "MQTT command %s: %s", mqtt_command_result_reason(verdict), line);
        return;
    }

    /* An inbound remote command is the MQTT equivalent of "somebody is at the
     * console" — the robot is not unattended, so dormancy's whole premise has
     * lapsed. Matches the wake call in command_task() below. Before dispatch,
     * not after, so `plan sleep` is not undone by its own wake.
     *
     * Full access only: in read-only mode the sender is anonymous, and an
     * anonymous client polling `plan` once a second would otherwise hold the
     * planner awake — and spending — until the budget fuse trips. */
    if (s_mqtt_access == MQTT_COMMAND_ACCESS_FULL) {
        plan_activity_wake();
    }

    const mqtt_command_result_t result = mqtt_command_dispatch(line, &ops);
    if (result != MQTT_COMMAND_OK) {
        ESP_LOGW(TAG, "MQTT command %s: %s", mqtt_command_result_reason(result), line);
    }
}

/**
 * @brief `mqtt` — broker-credential and broker-URI state and NVS provisioning
 *        (issues #626, #1312).
 *
 * Serial console ONLY. command_task() calls this before execute_console_line(),
 * and "mqtt" is not in mqtt_command.c's forwarded prefixes, so no MQTT message
 * can reach it — otherwise an anonymous publisher could hand itself the
 * credentials that lift its own lockout. test_mqtt_command.c pins that.
 *
 * Changes apply at the next boot: the MQTT client is created once, with the
 * credentials it was given, and the mode it reports must be the one it runs.
 */
static void handle_mqtt_cmd(const char *buf)
{
    /* Scratch buffers as long as the whole console line, so sscanf can never
     * truncate a token: an over-long credential reaches
     * credentials_nvs_save_mqtt() whole and is refused there, instead of being
     * cut to a different, wrong credential that looks saved. */
    _Static_assert(CONSOLE_LINE_MAX == 128, "keep the %127s widths below in step");
    char op[8] = {0};
    char user[CONSOLE_LINE_MAX] = {0};
    char pass[CONSOLE_LINE_MAX] = {0};
    const int n = sscanf(buf, "mqtt %7s %127s %127s", op, user, pass);

    if (n <= 0) {
        printf("mqtt: commands=%s (credentials from %s, user=%s) broker=%s (from %s)\n",
               mqtt_command_access_name(s_mqtt_access), get_mqtt_credentials_source(),
               get_mqtt_username() ? get_mqtt_username() : "-",
               get_mqtt_broker_uri(MQTT_BROKER_URI), get_mqtt_broker_uri_source());
        printf("  usage: mqtt auth <user> <pass> | mqtt auth clear   (applied at next boot;\n");
        printf("         max 32/64 chars, no spaces; NVS wins over credentials.h)\n");
        printf("         mqtt broker <uri> | mqtt broker clear     (mqtt[s]:// or ws[s]://,\n");
        printf("         applied at next boot; NVS wins over the build default)\n");
        return;
    }
    /* Issue #1312: a release carries only the compiled default broker. */
    if (strcmp(op, "broker") == 0 && n == 2) {
        if (strcmp(user, "clear") == 0) {
            printf("mqtt: %s\n", credentials_nvs_clear_mqtt_broker()
                                     ? "NVS broker cleared — reboot to use " MQTT_BROKER_URI
                                     : "clearing the NVS broker FAILED");
        } else {
            printf("mqtt: %s\n", credentials_nvs_save_mqtt_broker(user)
                                     ? "broker saved to NVS — reboot to apply"
                                     : "saving the broker FAILED (not mqtt[s]:// or ws[s]://, "
                                       "too long, or NVS error)");
        }
        return;
    }
    if (strcmp(op, "auth") == 0 && n == 2 && strcmp(user, "clear") == 0) {
        printf("mqtt: %s\n", credentials_nvs_clear_mqtt()
                                 ? "NVS credentials cleared — reboot to apply"
                                 : "clearing NVS credentials FAILED");
        return;
    }
    /* `mqtt auth clear <x>` falls to the usage line rather than storing a user
     * literally named "clear". */
    if (strcmp(op, "auth") == 0 && n == 3 && strcmp(user, "clear") != 0) {
        printf("mqtt: %s\n", credentials_nvs_save_mqtt(user, pass)
                                 ? "credentials saved to NVS — reboot to apply"
                                 : "saving credentials FAILED (too long, or NVS error)");
        return;
    }
    printf("mqtt: usage: mqtt | mqtt auth <user> <pass> | mqtt auth clear | mqtt broker <uri> "
           "| mqtt broker clear\n");
}

/**
 * @brief `gemini` — the Gemini API key: where it came from, what the last
 *        response said about it, and NVS provisioning (issue #845).
 *
 * Serial console ONLY, for the same reason as `mqtt`: command_task() calls
 * this before execute_console_line(), and "gemini" is not in mqtt_command.c's
 * forwarded prefixes, so no MQTT publisher can read or replace the key. The
 * line is logged redacted, and the key itself is never printed: only its
 * source, length and state.
 *
 * Unlike `mqtt auth`, a key applies at once: every request reads it fresh.
 */
static void handle_gemini_cmd(const char *buf)
{
    _Static_assert(CONSOLE_LINE_MAX == 128, "keep the %127s width below in step");
    char op[8] = {0};
    char key[CONSOLE_LINE_MAX] = {0};
    char extra[2] = {0};
    const int n = sscanf(buf, "gemini %7s %127s %1s", op, key, extra);

    if (n <= 0) {
        const char *k = get_gemini_api_key();
        printf("gemini: key=%s (source %s, %u chars) last response=%s\n", k ? "present" : "absent",
               get_gemini_api_key_source(), k ? (unsigned)strlen(k) : 0u,
               gemini_key_state_name(gemini_key_state_get()));
        printf("  usage: gemini auth <key> | gemini auth clear   (applied now; NVS wins over\n");
        printf("         credentials.h; max %d chars, no spaces)\n", MAX_API_KEY_LENGTH - 1);
        return;
    }
    if (strcmp(op, "auth") == 0 && n == 2 && strcmp(key, "clear") == 0) {
        printf("gemini: %s (now from %s)\n",
               credentials_nvs_clear_gemini() ? "NVS key cleared" : "clearing the NVS key FAILED",
               get_gemini_api_key_source());
        return;
    }
    if (strcmp(op, "auth") == 0 && n == 2) {
        printf("gemini: %s\n", credentials_nvs_save_gemini(key)
                                   ? "key saved to NVS — in use from the next request"
                                   : "saving the key FAILED (too long, a space, or NVS error)");
        return;
    }
    printf("gemini: usage: gemini | gemini auth <key> | gemini auth clear\n");
}

// ========================================
// Serial command task (Core 0, priority 5)
// ========================================
/* A packet split across two USB transfers can wait out one EOF sleep below
 * between two of its bytes; a burst gap no longer than that would reset the
 * parser inside a well-formed packet and drop it (console_line.h). */
_Static_assert(CONSOLE_LINE_BURST_GAP_MS >= 2u * TASK_DELAY_SHORT_MS,
               "CONSOLE_LINE_BURST_GAP_MS must outlast command_task's EOF sleep");

static void command_task(void *pvParameters)
{
    (void)pvParameters;
    console_line_t line;
    console_line_init(&line);
    int64_t last_improv_announce = 0;

    ESP_LOGI(TAG, "Command task started on core %d", xPortGetCoreID());

    /* Improv packets are binary: a 0x0D in a request must reach the parser as
     * 0x0D, not as the LF the console maps it to by default. Both terminators
     * already end a command line below, so typed commands are unaffected. */
    improv_console_raw_input();

    while (1) {
        /* While unprovisioned, announce ourselves ~1 Hz so ESP Web Tools can
         * discover the device on the port it just flashed. The announcement
         * repeats the remembered state, so during a connect attempt it keeps
         * saying Provisioning instead of overwriting it with Ready. */
        if (console_line_improv_armed(s_improv_session)) {
            int64_t now = esp_timer_get_time();
            if (now - last_improv_announce > 1000000LL) {
                improv_wifi_announce();
                last_improv_announce = now;
            }
        }

        int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SHORT_MS));
            continue;
        }

        /* While Improv is armed every byte goes to its parser first; it
         * ignores anything that is not an Improv packet, so ASCII console
         * commands still reach the switch below. No byte of a packet becomes
         * a console line (console_line.h): a Send Wi-Fi settings packet
         * carries the password, and a line is logged below and mirrored to
         * UART0 and the GlitchTip breadcrumb ring. The read time lets it
         * tell a packet still arriving after an arm or disarm from one cut
         * off for good. */
        const char *const buf = console_line_feed(&line, (char)ch, s_improv_session,
                                                  (uint32_t)(esp_timer_get_time() / 1000));
        if (buf != NULL) {
            /* Never echo a broker password into the serial log, which is
             * routinely captured to a file (`monitor | tee`). The redaction
             * test is looser than the routing one: a mistyped ` mqtt auth`
             * or `MQTT auth` is not run, but it still carries a password. */
            const bool is_mqtt_cmd = strncmp(buf, "mqtt", 4) == 0;
            const bool is_gemini_cmd = strncmp(buf, "gemini", 6) == 0;
            const char *lead = buf;
            while (*lead == ' ' || *lead == '\t') {
                ++lead;
            }
            const bool redact = strncasecmp(lead, "mqtt", 4) == 0;
            const bool redact_key = strncasecmp(lead, "gemini", 6) == 0;
            ESP_LOGI(TAG, "Serial cmd: %s",
                     redact       ? "mqtt ... (redacted)"
                     : redact_key ? "gemini ... (redacted)"
                                  : buf);

            /* Somebody is at the console, so the robot is not unattended
             * and the whole premise of dormancy has lapsed. Placed here —
             * one point, on a completed line — rather than on each command:
             * a new command added below then wakes the planner for free
             * instead of being the one that silently does not.
             *
             * Not on every BYTE: Improv provisioning packets are bytes too,
             * and a board being provisioned has no WiFi to plan over yet. */
            plan_activity_wake();

            // Single-letter movement commands (manual override / debug)
            if (buf[0] != '\0' && buf[1] == '\0') {
                switch (buf[0]) {
                    case 'F':
                    case 'f':
                        dispatch_movement("forward");
                        break;
                    case 'B':
                    case 'b':
                        dispatch_movement("backward");
                        break;
                    case 'L':
                    case 'l':
                        dispatch_movement("left");
                        break;
                    case 'R':
                    case 'r':
                        dispatch_movement("right");
                        break;
                    case 'C':
                    case 'c':
                        dispatch_movement("rotate_cw");
                        break;
                    case 'W':
                    case 'w':
                        dispatch_movement("rotate_ccw");
                        break;
                    case 'S':
                    case 's':
                        dispatch_movement("stop");
                        break;
                }
            } else if (is_mqtt_cmd) {
                /* Serial-only by construction: kept out of
                 * execute_console_line(), which the MQTT topic shares. */
                handle_mqtt_cmd(buf);
            } else if (is_gemini_cmd) {
                /* Serial-only, like `mqtt`: the key is a credential. */
                handle_gemini_cmd(buf);
            } else {
                execute_console_line(buf);
            }
        }
    }
}

// ========================================
// Initialization phases
// ========================================
static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/**
 * @brief Log a peripheral's init result and carry on.
 *
 * The hardware phase reports rather than aborts, so this is where a failure is
 * turned into a log line instead of a reboot. Deliberately returns nothing:
 * there is no caller decision left to make, and a return value would invite
 * somebody to re-chain these with ESP_RETURN_ON_ERROR.
 */
static void init_peripheral(const char *name, esp_err_t ret)
{
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "%s init failed (%s) — continuing without it", name, esp_err_to_name(ret));
    }
}

/* Returns nothing on purpose: after the change above there is no failure this
 * phase can report that should stop the boot, and a `void` signature is what
 * stops the next ESP_ERROR_CHECK from being wrapped around it. Health is read
 * afterwards from the live accessors, not from a return value. */
static void init_hardware(void)
{
    ESP_LOGI(TAG, "Phase 1: I2C bus + peripherals");

    // The I2C bus (TCA9548A + PCA9685 + optional MCP23017) is non-fatal: a bare
    // board with no I2C hardware fitted should still boot so the console,
    // camera, WiFi and AI planner remain reachable. The bus accessors return
    // ESP_ERR_INVALID_STATE while uninitialised, so the PCA9685-backed
    // peripherals (motors, servos, LEDs) and the expander fail safely at
    // runtime rather than crashing.
    esp_err_t i2c_ret = i2c_bus_init();
    if (i2c_ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus init failed (%s) — motors/servos/LEDs/expander disabled",
                 esp_err_to_name(i2c_ret));
        // Buzzer is on a dedicated GPIO (not I2C) — keep the startup beep.
        init_peripheral("Buzzer", buzzer_init());
        buzzer_beep();
        return;
    }

    /* Nothing past this point is fatal, and that is the whole point (issue
     * #500). Each of these used to abort the boot, so one peripheral that could
     * not initialise took down the console, WiFi, provisioning and the planner
     * with it — the opposite of what the bus-absent path above, and the camera,
     * network and AI phases below, all do. A reboot loop is also the worst
     * possible way to report a fault: it takes away the console you would use
     * to read the report.
     *
     * Every one of them fails safe on its own. The PCA9685-backed drivers guard
     * every entry point on their `initialized` flag and return
     * ESP_ERR_INVALID_STATE, the expander marks itself unavailable, and the
     * buzzer's tone routines no-op — so a failure here costs exactly the
     * function that failed. What is lost instead is visibility, and
     * self_report_collect() reads all four back through their live accessors so
     * the facts line names the missing peripheral (`i2c_peripherals=degraded(
     * servos)`) rather than leaving a silently immobile pan/tilt head. */
    init_peripheral("GPIO expander", gpio_expander_init());
    /* After the expander: the floor sensors are two of its pins. Without it
     * the cliff reflex stays disarmed and the self-report says so. */
    init_peripheral("Cliff sensors", cliff_sensor_init());
    init_peripheral("Motors", motor_controller_init());
    init_peripheral("LEDs", led_controller_init());
    init_peripheral("Servos", servo_controller_init());
    init_peripheral("Buzzer", buzzer_init());
#if FEATURE_OLED_ENABLED
    init_peripheral("OLED", oled_status_init());
#endif

    // Startup indication — each no-ops if its peripheral did not come up.
    led_set_both(&LED_COLOR_GREEN);
    buzzer_beep();

    /* Activity indicators take over both LEDs from here. Started after the boot
     * indication so the green stays visible until the first camera capture or
     * endpoint call actually happens, and non-fatal like every other optional
     * subsystem — a robot whose indicators failed must still drive and talk. */
    if (activity_trace_init() != ESP_OK) {
        ESP_LOGW(TAG, "activity_trace_init failed — no capture/endpoint indicators");
    }
}

static esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Phase 2: Camera");
    return camera_init();
}

/* Set by init_network(), read by net_supervisor_task(). */
static bool s_have_wifi_credentials;
static bool s_improv_ready;

static esp_err_t init_network(void)
{
    ESP_LOGI(TAG, "Phase 3: WiFi + network");

    /* Warns when credentials.h still holds placeholders, which is exactly the
     * state a web-flasher build boots in — the message tells the user that
     * Improv provisioning (started below) is the expected next step. */
    validate_credentials_at_runtime();

    /* Start joining and move on: the console, the executor and the reflex come
     * up while WiFi is still connecting (issue #846). The network supervisor
     * starts MQTT, OTA and Improv from the link's edges. */
    credentials_t creds = {0};
    load_credentials(&creds);
    s_have_wifi_credentials = creds.credentials_loaded && creds.wifi_ssid[0] != '\0';
    if (wifi_init() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed — continuing offline");
    } else if (s_have_wifi_credentials) {
        wifi_connect(creds.wifi_ssid, creds.wifi_password);
    }

    /* The parser is always installed; the supervisor arms it (feeds bytes and
     * announces) only while there is no link. A firmware built without
     * credentials (CI, the web flasher) has no other way to be given any. */
    improv_wifi_set_writer(improv_console_write);
    const esp_err_t improv_ret = improv_wifi_init(on_improv_credentials);
    if (improv_ret != ESP_OK) {
        ESP_LOGE(TAG, "improv_wifi_init failed (%s) — no provisioning path",
                 esp_err_to_name(improv_ret));
    }
    s_improv_ready = (improv_ret == ESP_OK);

    /* mDNS is a convenience, not a dependency — a failure here must not stop a
     * robot that is otherwise fine from booting. Hostname matches the project
     * directory name, per .claude/rules/mdns-hostname.md. */
    esp_err_t mdns_ret = mdns_init();
    if (mdns_ret != ESP_OK) {
        ESP_LOGW(TAG, "mdns_init failed (%s) — .local discovery unavailable",
                 esp_err_to_name(mdns_ret));
        return ESP_OK;
    }
    if (mdns_hostname_set("robocar") != ESP_OK || mdns_instance_name_set("Robocar") != ESP_OK) {
        ESP_LOGW(TAG, "Could not set mDNS hostname");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "mDNS: robocar.local");

    return ESP_OK;
}

static esp_err_t init_hierarchical_ai(void)
{
    ESP_LOGI(TAG, "Phase 4: Hierarchical AI controller");

    // goal_state must be initialised before reactive_controller and planner_task
    ESP_RETURN_ON_ERROR(goal_state_init(), TAG, "goal_state_init failed");

    // Persona must load before the planner and TTS tasks start, since both read
    // it while building their first request. Needs NVS, which Phase 0 set up.
    voice_persona_init();

    // The three gates on whether the planner is even offered the `speak` tool:
    // how recently it spoke (the ration), whether the view has changed since,
    // and whether the room sounds different since (the two senses). All must be
    // reset before the planner task starts, since it reads them while building
    // its first request.
    speech_budget_init();
    scene_change_init();
    ambient_audio_init();

    // The two gates on whether the planner makes a REQUEST at all — distinct
    // from the three above, which only decide what goes INTO one. Initialised
    // here rather than in planner_task_init() so `plan` reports real state even
    // on a board whose planner never started (no API key, no WiFi): a status
    // command that silently shows compile-time defaults is worse than one that
    // is absent.
    plan_budget_init();
    plan_activity_init(PLANNER_LOOP_PERIOD_MS);
    note_gate_init();

    // The same backstop for voice turns (issue #623): a ration on hands-free
    // triggers and a per-boot ceiling on every voice-turn request. Here, before
    // the listener and the voice-turn task start, for the same reason as above.
    voice_budget_init();

    // Speech path: queue and player must exist before the planner can emit a
    // `speak` call. Both are non-fatal — a robot that cannot talk should still
    // drive, so failures here are logged and stepped over rather than aborting
    // the boot.
    if (speech_queue_init() != ESP_OK) {
        ESP_LOGW(TAG, "speech_queue_init failed — voice disabled");
    } else if (audio_player_init() != ESP_OK) {
        ESP_LOGW(TAG, "audio_player_init failed — voice disabled");
    } else if (gemini_tts_start() != ESP_OK) {
        ESP_LOGW(TAG, "gemini_tts_start failed — voice disabled");
    }

    /* The microphone comes up AFTER audio_player_init(), and the order is load
     * bearing rather than stylistic: both live on I2S0, and bringing the TX side
     * up first means the controller object already exists when the PDM RX channel
     * asks for the free RX slot. See the allocation comment in mic_pdm.c for what
     * goes wrong if the two are ever merged into one i2s_new_channel() call.
     *
     * Non-fatal end to end, like the rest of this phase: a robot with a broken or
     * absent microphone must still boot, drive, talk and be provisioned. It simply
     * never has audio evidence, so the ambient gate never opens and the robot
     * falls back to exactly its pre-microphone behaviour. */
    if (mic_pdm_init() != ESP_OK) {
        ESP_LOGW(TAG, "mic_pdm_init failed — ambient audio gate disabled");
    } else if (ambient_listener_start() != ESP_OK) {
        ESP_LOGW(TAG, "ambient_listener_start failed — ambient audio gate disabled");
    }

    /* Push-to-talk. Independent of the ambient listener above — a board whose
     * gate failed to start can still answer a `listen`, and vice versa — so it
     * gets its own non-fatal branch rather than riding the same else-if chain. */
    if (voice_turn_start() != ESP_OK) {
        ESP_LOGW(TAG, "voice_turn_start failed — `listen` unavailable");
    }

    // reactive_controller spawns the 30 Hz executor on Core 0. This one *is*
    // required: without the executor nothing drives the motors at all.
    ESP_RETURN_ON_ERROR(reactive_controller_init(), TAG, "reactive_controller_init failed");

    /* planner_task spawns the Gemini planner on Core 1. It starts without WiFi
     * or a key and idles until both exist (issues #845, #846). Non-fatal by design: a board flashed
     * from the web flasher has no credentials at all, and panicking here would boot-loop the
     * documented default configuration. Without a planner the executor simply holds STOP, which is
     * safe — and the console, Improv provisioning and self-report all stay reachable so the device
     * can be told what it is missing. */
    esp_err_t planner_ret = planner_task_init();
    if (planner_ret != ESP_OK) {
        ESP_LOGW(TAG, "planner_task_init failed (%s) — AI planner disabled, robot holds STOP",
                 esp_err_to_name(planner_ret));
    }
    self_report_note_init(SELF_REPORT_SUBSYS_PLANNER, planner_ret == ESP_OK);

    return ESP_OK;
}

#if MQTT_LOGGING_ENABLED
static bool s_mqtt_created;
#if OTA_ENABLED
/* Set by the supervisor once ota_manager_init() has succeeded; until then
 * that init does the OTA subscription itself (issue #1300). */
static bool s_ota_initialised;
#endif

/**
 * @brief Create the MQTT client without connecting it (ADR-028 §2).
 *
 * app_main calls this before the network supervisor starts, so OTA and the
 * upgrade desk can register their subscriptions before the first link-up;
 * mqtt_logger_subscribe() refuses until the client exists (issue #1054). The
 * supervisor runs start_mqtt_service(), which retries a failed creation,
 * before ota_manager_init() subscribes (issue #1126). s_mqtt_created is written
 * by app_main before the supervisor task exists and by that task afterwards,
 * never concurrently.
 *
 * @return true once the client exists.
 */
static bool create_mqtt_client(void)
{
    if (s_mqtt_created) {
        return true;
    }
    const mqtt_logger_config_t mqtt_cfg = {
        /* NVS (`mqtt broker`) first, then the compiled default (#1312). */
        .broker_uri = get_mqtt_broker_uri(MQTT_BROKER_URI),
        .client_id = MQTT_CLIENT_ID,
        /* NULL when unconfigured, which connects anonymously and leaves the
         * command topic read-only (init_mqtt_access()). */
        .username = get_mqtt_username(),
        .password = get_mqtt_password(),
        .log_topic = MQTT_LOG_TOPIC_BASE,
        .status_topic = MQTT_STATUS_TOPIC,
        .command_topic = MQTT_COMMAND_TOPIC,
        .buffer_size = MQTT_LOG_BUFFER_SIZE,
        .keepalive_interval = MQTT_KEEPALIVE_INTERVAL,
        .qos_level = MQTT_QOS_LEVEL,
        .min_level = MQTT_MIN_LOG_LEVEL,
        .retain_status = MQTT_RETAIN_STATUS,
    };
    const esp_err_t mqtt_ret = mqtt_logger_init(&mqtt_cfg);
    if (mqtt_ret != ESP_OK) {
        ESP_LOGW(TAG, "mqtt_logger_init failed (%s) — remote logging disabled",
                 esp_err_to_name(mqtt_ret));
        return false;
    }
    s_mqtt_created = true;
    /* Called after create_tasks(), so the queues the command handler
     * dispatches into already exist; no command arrives before the
     * supervisor connects the client anyway. */
    mqtt_logger_set_command_handler(mqtt_command_received);
    return true;
}

/** Every link-up connects the client. A boot-time creation that failed is
 *  retried here, as the first link-up used to do. */
static void start_mqtt_service(void)
{
    const bool created_before = s_mqtt_created;
    if (!create_mqtt_client()) {
        return;
    }
#if OTA_ENABLED
    /* ota_manager_init() subscribes once per boot, on the first link-up. When
     * the client could not be created by then, that subscription failed; the
     * creation that finally succeeds registers it (issue #1300). Before init
     * has run, init subscribes itself: a nudge taken earlier would be spent
     * on a check that cannot start yet. */
    if (!created_before && s_ota_initialised) {
        (void)ota_manager_subscribe_notify();
    }
#endif
    mqtt_logger_resume();
}
#endif

/** Improv credentials waiting on the link: save them on GOT_IP, or report
 *  failure once the fast retries are spent and go back to the stored network. */
static void settle_improv_attempt(bool link_up, net_supervisor_t *sup)
{
    if (!__atomic_load_n(&s_improv_pending, __ATOMIC_ACQUIRE)) {
        return;
    }
    if (link_up) {
        /* Persist only after the credentials are proven to work, so a typo
         * cannot overwrite a known-good stored network. */
        if (!credentials_nvs_save_wifi(s_improv_ssid, s_improv_pass)) {
            improv_wifi_fail_provisioning(IMPROV_ERROR_UNKNOWN);
        } else {
            credentials_reload();
            s_have_wifi_credentials = true;
            net_supervisor_set_credentials(sup, true);
            improv_wifi_send_state(IMPROV_STATE_PROVISIONED);
            improv_wifi_send_provisioned_result(NULL);
            ESP_LOGI(TAG, "Improv: provisioning complete");
        }
        __atomic_store_n(&s_improv_pending, false, __ATOMIC_RELEASE);
    } else if (wifi_connect_failed()) {
        ESP_LOGW(TAG, "Improv: could not join '%s'", s_improv_ssid);
        improv_wifi_fail_provisioning(IMPROV_ERROR_UNABLE_CONNECT);
        __atomic_store_n(&s_improv_pending, false, __ATOMIC_RELEASE);
        credentials_t creds = {0};
        if (load_credentials(&creds) && creds.wifi_ssid[0] != '\0') {
            wifi_connect(creds.wifi_ssid, creds.wifi_password);
        }
    }
}

/**
 * @brief Run the network services from the link's edges (issue #846).
 *
 * MQTT and OTA used to start from one check at boot, so a board that booted
 * offline never got them, and an OTA image whose first boot was offline was
 * never confirmed and rolled back. net_supervisor_core.c decides; this task
 * observes the link every NET_SUPERVISOR_PERIOD_MS and carries out what it
 * returns. Nothing here blocks on WiFi.
 */
static void net_supervisor_task(void *arg)
{
    (void)arg;
    net_supervisor_t sup;
    net_supervisor_init(&sup, s_have_wifi_credentials, (uint32_t)(esp_timer_get_time() / 1000));

    for (;;) {
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        const bool link_up = wifi_is_connected();

        settle_improv_attempt(link_up, &sup);
        if (link_up) {
            wall_clock_start(); /* once per boot; quest timestamps (ADR-039) */
        }
        const net_actions_t act = net_supervisor_step(&sup, link_up, now_ms);

        /* MQTT first: ota_manager_init() subscribes to the OTA check-now topic
         * once per boot, and mqtt_logger_subscribe() refuses until the client
         * exists. start_mqtt_service() retries a boot-time creation that
         * failed, so the subscription no longer depends on it (issue #1126). */
        if (act.start_mqtt) {
#if MQTT_LOGGING_ENABLED
            start_mqtt_service();
#endif
        }
        if (act.start_ota) {
#if OTA_ENABLED
            /* Arms ota_manager_confirm_valid() too. With
             * CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y an image that is never
             * confirmed is rolled back on the next boot, so this must run on
             * the first link-up of every boot, not only on an online boot. */
            const esp_err_t ota_ret = ota_manager_init();
            if (ota_ret != ESP_OK) {
                ESP_LOGW(TAG, "ota_manager_init failed (%s) — updates disabled",
                         esp_err_to_name(ota_ret));
            }
#if MQTT_LOGGING_ENABLED
            s_ota_initialised = (ota_ret == ESP_OK);
#endif
#endif
        }
        if (act.stop_mqtt) {
#if MQTT_LOGGING_ENABLED
            mqtt_logger_pause();
#endif
        }
        const bool improv_armed = console_line_improv_armed(s_improv_session);
        if (act.arm_improv && s_improv_ready && !improv_armed) {
            /* Ready again: the remembered state may still be PROVISIONED from
             * an earlier provisioning, and command_task's announcement repeats
             * whatever is remembered. */
            improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);
            s_improv_session = s_improv_session + 1u; /* odd: armed */
            ESP_LOGW(TAG, "No WiFi link — Improv provisioning armed (ESP Web Tools)");
        }
        if (act.disarm_improv && improv_armed) {
            s_improv_session = s_improv_session + 1u; /* even: disarmed */
            ESP_LOGI(TAG, "WiFi link up — Improv provisioning disarmed");
        }

        vTaskDelay(pdMS_TO_TICKS(NET_SUPERVISOR_PERIOD_MS));
    }
}

static void start_network_supervisor(void)
{
    if (xTaskCreatePinnedToCore(net_supervisor_task, "net_sup", NET_SUPERVISOR_TASK_STACK_SIZE,
                                NULL, NET_SUPERVISOR_TASK_PRIORITY, NULL,
                                NET_SUPERVISOR_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "network supervisor task not created — no MQTT, OTA or Improv");
    }
}

/* Every other creation site in this firmware checks its result; this one used
 * to be the outlier, which turned an allocation failure into an assert-abort
 * inside FreeRTOS rather than a diagnosable message. */
static esp_err_t create_tasks(void)
{
    ESP_LOGI(TAG, "Phase 5: Creating FreeRTOS tasks");

    s_motor_queue = xQueueCreate(MOTOR_CMD_QUEUE_DEPTH, sizeof(motor_cmd_t));
    s_periph_queue = xQueueCreate(PERIPHERAL_CMD_QUEUE_DEPTH, sizeof(periph_cmd_t));
    if (!s_motor_queue || !s_periph_queue) {
        ESP_LOGE(TAG, "Command queue allocation failed");
        return ESP_ERR_NO_MEM;
    }

    // Core 0 tasks: motor control, peripherals, serial commands
    BaseType_t ok = pdPASS;
    ok &= xTaskCreatePinnedToCore(motor_control_task, "motor", MOTOR_TASK_STACK_SIZE, NULL,
                                  MOTOR_TASK_PRIORITY, NULL, MOTOR_TASK_CORE);
    ok &= xTaskCreatePinnedToCore(peripheral_task, "periph", PERIPHERAL_TASK_STACK_SIZE, NULL,
                                  PERIPHERAL_TASK_PRIORITY, NULL, PERIPHERAL_TASK_CORE);
    ok &= xTaskCreatePinnedToCore(command_task, "cmd", COMMAND_TASK_STACK_SIZE, NULL,
                                  COMMAND_TASK_PRIORITY, NULL, COMMAND_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "One or more core tasks could not be created");
        return ESP_ERR_NO_MEM;
    }
    // Note: reactive_controller task is created inside reactive_controller_init() (Core 0)
    // Note: planner task is created inside planner_task_init() (Core 1)
    return ESP_OK;
}

/** Name for esp_reset_reason(), as the boot banner prints it. BROWNOUT is
 *  reported even with CONFIG_ESP_BROWNOUT_DET=n: on 2026-11-07 the chip reset
 *  on supply dips with the ROM printing rst:0xf (BROWNOUT_RST) and this
 *  returning ESP_RST_BROWNOUT, 20 times on battery. A dip deep enough to drop
 *  the supply entirely reads POWERON instead. */
static const char *reset_reason_name(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:
            return "POWERON";
        case ESP_RST_EXT:
            return "EXT";
        case ESP_RST_SW:
            return "SW";
        case ESP_RST_PANIC:
            return "PANIC";
        case ESP_RST_INT_WDT:
            return "INT_WDT";
        case ESP_RST_TASK_WDT:
            return "TASK_WDT";
        case ESP_RST_WDT:
            return "WDT";
        case ESP_RST_DEEPSLEEP:
            return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:
            return "BROWNOUT";
        case ESP_RST_SDIO:
            return "SDIO";
        case ESP_RST_USB:
            return "USB";
        case ESP_RST_JTAG:
            return "JTAG";
        case ESP_RST_EFUSE:
            return "EFUSE";
        case ESP_RST_PWR_GLITCH:
            return "PWR_GLITCH";
        case ESP_RST_CPU_LOCKUP:
            return "CPU_LOCKUP";
        default:
            return "UNKNOWN";
    }
}

/** Two lines identifying exactly what is running and why it booted, first in
 *  the log so a capture from J15 or MQTT is self-describing. Two, not one:
 *  the UART0 mirror cuts lines at LOG_MIRROR_LINE_MAX (192), and a single line
 *  lost the reset reason off its end. The reset reason leads the first line. */
static void log_boot_banner(void)
{
    const esp_app_desc_t *desc = esp_app_get_description();
    char elf_sha[9];
    esp_app_get_elf_sha256(elf_sha, sizeof(elf_sha));
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    const esp_reset_reason_t reason = esp_reset_reason();

    ESP_LOGI(TAG, "Boot: reset %s (%d) | %s v%s build %s", reset_reason_name(reason), (int)reason,
             desc->project_name, desc->version, ota_manager_get_build_sha());
    ESP_LOGI(TAG, "Build: elf %s | compiled %s %s | IDF %s | chip rev v%d.%d, %d cores", elf_sha,
             desc->date, desc->time, desc->idf_ver, chip.revision / 100, chip.revision % 100,
             chip.cores);
}

// ========================================
// Application entry point
// ========================================
void app_main(void)
{
    /* First of all: the button that switched the robot on stays down until HOLD
     * is high (ADR-038). A constructor in power_latch.c has usually done this
     * already; this call is the backstop and keeps that object linked. Makes
     * no logging calls. */
    const esp_err_t latch_ret = power_latch_init();

    /* Before anything logs: the previous boot's last log lines are still in
     * RTC memory, and the first ESP_LOG line from here on overwrites them
     * (ADR-033). Makes no logging calls itself. */
    event_report_boot_capture();

    /* First, so the banner and every boot phase after it reach J15 (UART0 TX)
     * as well as the USB console (issue #798). It needs only PSRAM, which the
     * startup code has already brought up. Non-fatal: without it the log
     * stays on USB alone, as it always has. */
    const esp_err_t mirror_ret = log_mirror_init();

    ESP_LOGI(TAG, "=== Robocar (XIAO ESP32-S3 Sense) ===");
    if (mirror_ret != ESP_OK) {
        ESP_LOGW(TAG, "UART0 log mirror unavailable (%s) — log on USB only",
                 esp_err_to_name(mirror_ret));
    }
    log_boot_banner();
    if (latch_ret == ESP_OK) {
        ESP_LOGI(TAG, "Power latch: HOLD high %s",
                 power_latch_asserted_early() ? "before app_main" : "from app_main");
    } else if (latch_ret != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGE(TAG, "Power latch: HOLD not driven (%s)", esp_err_to_name(latch_ret));
    }

    /* One GlitchTip event for a reset that was not deliberate (ADR-033). It
     * waits in the queue until MQTT connects; nothing here blocks the boot. */
    const esp_err_t report_ret = event_report_start();
    if (report_ret != ESP_OK) {
        ESP_LOGW(TAG, "event reporting unavailable (%s)", esp_err_to_name(report_ret));
    }
    const char *reason = reset_reason_name(esp_reset_reason());
    const esp_err_t sent = event_report_send_reset(reason);
    if (sent == ESP_OK) {
        ESP_LOGI(TAG, "reset event queued for GlitchTip (%s)", reason);
    } else if (sent != ESP_ERR_NOT_SUPPORTED && report_ret == ESP_OK) {
        ESP_LOGW(TAG, "reset event not queued (%s)", esp_err_to_name(sent));
    }

    ESP_ERROR_CHECK(init_nvs());
    init_mqtt_access();
    /* Load the Gemini key once here, on one task, before the planner, TTS and
     * voice tasks can race the first load (issue #845). */
    ESP_LOGI(TAG, "Gemini key: %s (source %s)", get_gemini_api_key() ? "present" : "absent",
             get_gemini_api_key_source());

    /* The hardware phase cannot fail the boot: a missing bus and a peripheral
     * that would not initialise are both logged and carried past, so what came
     * up is read back from the live accessors rather than from a return value.
     * self_report renders the per-peripheral breakdown (issue #500). */
    init_hardware();
    self_report_note_init(SELF_REPORT_SUBSYS_I2C_BUS, i2c_bus_is_ready());

    /* Quest log (ADR-039): mounts and loads on its own task, so a first-boot
     * SPIFFS format does not hold the boot. Non-fatal. */
    if (quest_store_start() != ESP_OK) {
        ESP_LOGW(TAG, "quest store not started — quests unavailable");
    }

    // Camera is non-fatal here so the robot can still boot, connect, and report
    // "camera not responding" instead of panicking on a board without the Sense
    // module fitted.
    esp_err_t cam_ret = init_camera();
    self_report_note_init(SELF_REPORT_SUBSYS_CAMERA, cam_ret == ESP_OK);
    if (cam_ret != ESP_OK) {
        ESP_LOGW(TAG, "Camera init failed (%s) — continuing so status stays reportable",
                 esp_err_to_name(cam_ret));
    }

    init_network();

    /* The AI phase is non-fatal end to end now (see init_hierarchical_ai): the
     * only hard requirement inside it is the executor, and losing that is
     * reported rather than panicked on so the console stays usable. */
    esp_err_t ai_ret = init_hierarchical_ai();
    if (ai_ret != ESP_OK) {
        ESP_LOGE(TAG, "Hierarchical AI init failed (%s) — motion control unavailable",
                 esp_err_to_name(ai_ret));
    }
    self_report_note_init(SELF_REPORT_SUBSYS_AUDIO, audio_player_is_ready());

    esp_err_t tasks_ret = create_tasks();
    if (tasks_ret != ESP_OK) {
        ESP_LOGE(TAG, "Task creation failed (%s) — console and manual control unavailable",
                 esp_err_to_name(tasks_ret));
    }

    /* Pack monitor and power button (ADR-038). After the expander and the
     * executor, which the button read and the shutdown use. Non-fatal. */
    const esp_err_t power_ret = power_manager_start();
    if (power_ret != ESP_OK && power_ret != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "power manager not started (%s)", esp_err_to_name(power_ret));
    }

    /* Boot diagnostic: prove the drivetrain turns over before any later phase
     * claims the robot is ready. Subject to the obstacle reflex, and skipped
     * gracefully when the motors are absent or the executor never started. */
    run_wheel_exercise();

#if MQTT_LOGGING_ENABLED
    /* Before the supervisor, so OTA's subscription on the first link-up finds
     * the client (issue #1054). Connects nothing; non-fatal. */
    create_mqtt_client();
#endif
    start_network_supervisor();

    // Spoken self-introduction + status diagnostic (announces once voice-able,
    // re-announces on health change). Non-fatal: a robot that cannot start the
    // monitor still drives.
    if (self_report_start() != ESP_OK) {
        ESP_LOGW(TAG, "self_report_start failed — no spoken self-report");
    }

    ESP_LOGI(TAG, "=== System ready ===");

    // app_main returns; FreeRTOS scheduler runs tasks
}
