/**
 * @file main.c
 * @brief Unified robocar firmware for XIAO ESP32-S3 Sense
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
#include "motor_controller.h"
#include "mqtt_command.h"
#include "mqtt_logger.h"
#include "oled_status.h"
#include "ota_manager.h"
#include "pin_config.h"
#include "plan_activity.h"
#include "plan_budget.h"
#include "planner_task.h"
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
/* Written by the network supervisor task, read by command_task (issue #846). */
static volatile bool s_improv_active;

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
// GPIO expander console commands (bench testing)
//   gpio                     - 16-pin port dump
//   gpio mode <pin> in|up|out - set direction (up = input with pullup);
//                              pins 7 and 15 (GPA7/GPB7) are output-only
//   gpio set <pin> 0|1       - drive an output pin
//   gpio get <pin>           - read one pin
// ========================================
/* `voice`                      — show current persona/voice/budget and options
 * `voice <slug>`               — switch persona (e.g. `voice fi-1950`)
 * `voice say <text>`           — speak a line now, to audition the persona
 * `voice name <VoiceName>`     — override the prebuilt voice (`voice name -` clears)
 * `voice vary`                 — draw and print a variation directive (see below)
 * `voice said`                 — print what the robot remembers saying
 * `voice quiet <seconds>`      — minimum gap between utterances (0 = none)
 * `voice budget <n> <seconds>` — at most n utterances per window (n=0 mutes)
 * `voice repeat <pct>`         — similarity at which a line counts as a repeat
 * `voice scene <n>`            — how much the view must change (0 = gate off)
 * `voice loud <db>`            — excursion above the noise floor (0 = sub-gate off)
 * `voice sound <db>`           — how much the room's spectrum must move (0 = off)
 * `voice endpoint <ms> <db>`   — quiet time / speech margin that ends a VAD turn
 * `voice trigger <%> <db> <ms>` — voice-band share / floor margin / duration that starts one
 * `voice ration <n> <seconds>` — at most n hands-free voice-turn requests per window
 * `voice turns [<n>]`          — voice-turn spend; with n, the per-boot ceiling (0 = off)
 * `voice resume`               — reopen a tripped voice-turn ceiling and zero the counters
 * `mic` / `mic dump <n>`       — microphone state; dump PCM frames to the console
 * `listen [seconds]`           — one push-to-talk voice turn (record, ask, answer)
 * `trace`                      — camera + endpoint activity counters since boot
 * `trace led on|off`           — LED activity indicators (counters keep running)
 * `trace reset`                — zero the counters and clear any held fault colour
 *
 * Switching and auditioning are runtime rather than compile-time because only a
 * listener can judge a voice; a reflash per candidate is far too slow a loop.
 *
 * `voice vary` exists for the same reason one step further in: the variation
 * pools (dialogue_style.h) only pay off if their entries read naturally in the
 * persona's language, and the alternative way to see what the model is being
 * told is to wait out a 15 s planner period per sample.
 *
 * `quiet` / `budget` / `repeat` / `scene` are the same argument again: how
 * talkative and how repetitive the robot is allowed to be, and how much the view
 * must move to be worth mentioning, are judgements that need a person in the
 * room, and every trial otherwise costs a rebuild plus a flash. `scene` in
 * particular has no defensible compile-time default — what counts as a changed
 * view depends on the room and the lens, so read the `scene:` field the planner
 * logs each cycle and set the threshold from real numbers. None of the four
 * persist: a boot comes up at the documented default rather than at whatever an
 * experiment left behind. */
/**
 * @brief Two lines on what voice turns have spent (voice_budget.h, issue #623).
 *
 * Shared by `voice`, `voice turns` and a refused `listen`, so all three say the
 * same thing. The ignored count sits beside the hands-free count on purpose:
 * the two close together is the sign that `voice trigger` is too permissive.
 */
static void print_voice_turn_budget(const char *indent, uint32_t now_ms)
{
    const uint32_t ceiling = voice_budget_ceiling();
    const uint32_t requests = voice_budget_requests();
    const uint32_t vad = voice_budget_vad_requests();
    /* Clamped: the two counters are read unlocked, and a charge landing between
     * the reads (or a resume) can leave vad > requests for one line. */
    const uint32_t listen = (requests > vad) ? (requests - vad) : 0u;
    printf("%sturns:  %u", indent, (unsigned)requests);
    if (ceiling) {
        printf("/%u", (unsigned)ceiling);
    }
    printf(" requests (hands-free %u, ignored %u; listen %u, ignored %u)%s\n", (unsigned)vad,
           (unsigned)voice_budget_ignored_vad(), (unsigned)listen,
           (unsigned)voice_budget_ignored_listen(), ceiling ? "" : " — NO CEILING");

    uint8_t ration_max = 0;
    uint32_t ration_window_ms = 0;
    voice_budget_get_ration(&ration_max, &ration_window_ms);
    if (ration_max == 0u || ration_window_ms == 0u) {
        printf("%s        ration: off (cooldown only), refused %u\n", indent,
               (unsigned)voice_budget_refused());
    } else {
        const uint32_t wait_ms = voice_budget_ration_wait_ms(now_ms);
        printf("%s        ration: %u/%u per %us, refused %u", indent,
               (unsigned)voice_budget_ration_used(now_ms), (unsigned)ration_max,
               (unsigned)(ration_window_ms / 1000U), (unsigned)voice_budget_refused());
        if (wait_ms) {
            printf(", next in %us", (unsigned)((wait_ms + 999U) / 1000U));
        }
        printf("\n");
    }
    if (voice_budget_check(now_ms, false) == VOICE_BUDGET_TRIP_CEILING) {
        printf("%s        STOPPED: spend ceiling reached — `voice resume` to continue\n", indent);
    }
}

static void handle_voice_cmd(const char *buf)
{
    char op[24] = {0};
    const int n = sscanf(buf, "voice %23s", op);

    if (n <= 0) {
        char voice[VOICE_PERSONA_VOICE_MAX];
        voice_persona_effective_voice(voice, sizeof(voice));
        const voice_persona_t *cur = voice_persona_get();
        printf("voice: persona=%s lang=%s voice=%s\n", cur->slug, cur->language_code, voice);
        for (size_t i = 0;; ++i) {
            const voice_persona_t *p = voice_persona_at(i);
            if (!p) {
                break;
            }
            printf("  %c %-12s %s\n", (p == cur) ? '*' : ' ', p->slug, p->label);
        }

        uint32_t gap_ms = 0;
        uint32_t window_ms = 0;
        uint8_t max_per = 0;
        speech_budget_get(&gap_ms, &max_per, &window_ms);
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        const uint32_t wait_ms = speech_budget_wait_ms(now_ms);
        printf("  budget: quiet=%us max=%u/%us used=%u repeat=%u%%\n", (unsigned)(gap_ms / 1000U),
               (unsigned)max_per, (unsigned)(window_ms / 1000U),
               (unsigned)speech_budget_used(now_ms), (unsigned)dialogue_style_repeat_pct());
        /* "BLIND": no frame has ever decoded, so novel() fails closed and the 0
         * is "never seen anything", not a still room (issue #703; the mic line's
         * DEAF is the same word for the other sense). "no pose": the head is
         * between dwell poses, so the 0 is "cannot measure", not "unchanged"
         * (ADR-029 §2). */
        printf("  scene:  %u/%u at pose %c (%s)\n", scene_change_score(),
               (unsigned)scene_change_threshold(), scene_pose_letter(scene_change_pose()),
               !scene_change_has_measurement()          ? "BLIND — no frame decoded"
               : scene_change_threshold() == 0          ? "gate off"
               : scene_change_pose() == SCENE_POSE_NONE ? "no pose"
               : scene_change_novel()                   ? "new"
                                                        : "same");
        printf("  loud:   %u/%u dB (floor %d dB)\n", ambient_audio_loud_score(now_ms),
               (unsigned)ambient_audio_loud_threshold(), (int)ambient_audio_floor_db());
        printf("  sound:  %u/%u dB (%s)\n", ambient_audio_shape_score(now_ms),
               (unsigned)ambient_audio_shape_threshold(),
               !ambient_audio_has_measurement() ? "DEAF — nothing ever heard"
               : ambient_audio_novel(now_ms)    ? "new"
                                                : "same");
        uint32_t ep_silence = 0;
        uint32_t ep_min = 0;
        uint32_t ep_max = 0;
        uint8_t ep_margin = 0;
        voice_turn_get_endpoint(&ep_silence, &ep_margin, &ep_min, &ep_max);
        printf("  vad:    %s | endpoint: quiet %u ms at %u dB over floor, min %u ms, max %u ms\n",
               voice_turn_get_vad() ? "on" : "off", (unsigned)ep_silence, (unsigned)ep_margin,
               (unsigned)ep_min, (unsigned)ep_max);
        printf("          trigger: voice band >= %u%%, +%u dB over floor, for %u ms\n",
               (unsigned)speech_trigger_share_pct(), (unsigned)speech_trigger_margin_db(),
               (unsigned)speech_trigger_sustain_ms());
        print_voice_turn_budget("  ", now_ms);
        /* Every gate is reported, and which one is holding, because a silent robot
         * is otherwise indistinguishable from a broken one — and on a static,
         * quiet scene silence is the correct behaviour. The evidence line names
         * BOTH senses now: after the OR, "waiting on the view" would be a lie
         * whenever the room could still open the gate by sounding different. */
        if (wait_ms == UINT32_MAX) {
            printf("  speech: muted\n");
        } else if (wait_ms) {
            printf("  speech: waiting on budget (%us)\n", (unsigned)(wait_ms / 1000U));
        } else if (!scene_change_novel() && !ambient_audio_novel(now_ms)) {
            /* A blind camera has no view to change; only the room can open it. */
            printf("  speech: %s\n", scene_change_has_measurement()
                                         ? "waiting on the view or the room to change"
                                         : "waiting on the room to change (camera BLIND)");
        } else {
            printf("  speech: allowed now\n");
        }

        printf("  volume: %u%% of full scale (boot default %u%%)\n",
               (unsigned)audio_player_volume_pct(), (unsigned)AUDIO_VOLUME_PCT);
        printf("  usage: voice <slug> | say <text> | name <VoiceName|-> | vary | said\n");
        printf("         voice quiet <s> | budget <n> <s> | repeat <pct> | scene <n>\n");
        printf("         voice loud <db> | sound <db> | vad on|off   (see also: mic)\n");
        printf(
            "         voice endpoint <quiet_ms> <db>   when a hands-free turn stops recording\n");
        printf("         voice trigger <band%%> <db> <ms>  what starts one (see `mic`)\n");
        printf("         voice ration <n> <s> | turns <n> | resume   what voice turns may spend\n");
        printf("         voice volume <pct>   amplitude, not loudness: halving = -6 dB\n");
        printf("         voice fx [on|off|body <10..200>|metal <0..85>|drive <10..400>]\n");
        printf(
            "                              retro-robot body resonance; body is TENTHS of a ms\n");
        return;
    }

    if (strcmp(op, "volume") == 0) {
        unsigned pct = 0;
        if (sscanf(buf, "voice volume %u", &pct) != 1 || pct > AUDIO_VOLUME_PCT_MAX) {
            printf("voice: usage: voice volume <0..%u>  (now %u%%)\n",
                   (unsigned)AUDIO_VOLUME_PCT_MAX, (unsigned)audio_player_volume_pct());
            return;
        }
        audio_player_set_volume_pct((uint8_t)pct);
        /* Reported as a ratio against the boot default rather than in dB: the
         * dB would need log10() and a %f, and a float print is silently wrong
         * if CONFIG_NEWLIB_NANO_FORMAT is ever turned on. The conversion worth
         * remembering is fixed anyway and lives in the usage line — halving the
         * percent is -6 dB, and an ear reads about -10 dB as half as loud. */
        printf("voice: volume=%u%% of full scale (boot default %u%%, so %u/100 of it)\n",
               (unsigned)audio_player_volume_pct(), (unsigned)AUDIO_VOLUME_PCT,
               (unsigned)((audio_player_volume_pct() * 100U) / AUDIO_VOLUME_PCT));
        return;
    }

    /* The retro-robot body resonance. Every parameter is an INTEGER on the
     * console even though the effect is float internally: printing a float
     * needs %f, and %f is silently wrong under CONFIG_NEWLIB_NANO_FORMAT — the
     * same reason `voice volume` reports a ratio rather than dB. Body is in
     * tenths of a millisecond, the other two in percent.
     *
     * Not persisted, like every other knob here: a boot comes up at the
     * documented default rather than at last night's experiment. */
    if (strcmp(op, "fx") == 0) {
        voice_fx_t *fx = audio_player_fx();
        char sub[16] = {0};
        unsigned value = 0;
        const int got = sscanf(buf, "voice fx %15s %u", sub, &value);

        if (got >= 1 && strcmp(sub, "on") == 0) {
            voice_fx_set_enabled(fx, true);
        } else if (got >= 1 && strcmp(sub, "off") == 0) {
            voice_fx_set_enabled(fx, false);
        } else if (got == 2 && strcmp(sub, "body") == 0) {
            if (!voice_fx_set_body_ms(fx, (float)value / 10.0f)) {
                printf("voice: fx body out of range (10..200 tenths of a ms)\n");
                return;
            }
        } else if (got == 2 && strcmp(sub, "metal") == 0) {
            if (!voice_fx_set_feedback(fx, (float)value / 100.0f)) {
                printf("voice: fx metal out of range (0..85 %%)\n");
                return;
            }
        } else if (got == 2 && strcmp(sub, "drive") == 0) {
            if (!voice_fx_set_drive(fx, (float)value / 100.0f)) {
                printf("voice: fx drive out of range (10..400 %%)\n");
                return;
            }
        } else if (got >= 1) {
            printf(
                "voice: usage: voice fx [on|off|body <10..200>|metal <0..85>|drive <10..400>]\n");
            return;
        }

        /* Report unconditionally, including after a bare `voice fx`, so the
         * command doubles as the read-back. The resonance frequency is what
         * `body` actually means to an ear, so it is printed beside it. */
        const unsigned body_tenths = (unsigned)(fx->body_ms * 10.0f + 0.5f);
        const unsigned resonance_hz = body_tenths ? (unsigned)(10000u / body_tenths) : 0u;
        printf("voice: fx=%s body=%u.%u ms (~%u Hz, %u samples) metal=%u%% drive=%u%%\n",
               fx->enabled ? "on" : "off", body_tenths / 10u, body_tenths % 10u, resonance_hz,
               (unsigned)fx->delay_samples, (unsigned)(fx->feedback * 100.0f + 0.5f),
               (unsigned)(fx->drive * 100.0f + 0.5f));
        return;
    }

    if (strcmp(op, "scene") == 0) {
        unsigned threshold = 0;
        if (sscanf(buf, "voice scene %u", &threshold) != 1 || threshold > UINT8_MAX) {
            printf("voice: usage: voice scene <0..255>  (0 = gate off)\n");
            return;
        }
        scene_change_set_threshold((uint8_t)threshold);
        printf("voice: scene=%u (now %u)\n", (unsigned)scene_change_threshold(),
               scene_change_score());
        return;
    }

    /* NOTE the inverted zero convention against `voice scene` above, and it is not
     * a slip: scene is ANDed into the evidence term, so its 0 means "always
     * novel"; loud and sound are ORed, so their 0 must mean "never contributes".
     * Both readings say "this gate is not participating" — the polarity just
     * follows the operator. Setting BOTH loud and sound to 0 restores exactly the
     * pre-microphone behaviour, which ambient_audio's host tests pin. */
    if (strcmp(op, "loud") == 0) {
        unsigned db = 0;
        if (sscanf(buf, "voice loud %u", &db) != 1 || db > UINT8_MAX) {
            printf("voice: usage: voice loud <0..255> dB  (0 = sub-gate off)\n");
            return;
        }
        ambient_audio_set_loud_threshold((uint8_t)db);
        printf("voice: loud=%u dB (now %u dB, floor %d dB)\n",
               (unsigned)ambient_audio_loud_threshold(),
               ambient_audio_loud_score((uint32_t)(esp_timer_get_time() / 1000)),
               (int)ambient_audio_floor_db());
        return;
    }

    if (strcmp(op, "sound") == 0) {
        unsigned db = 0;
        if (sscanf(buf, "voice sound %u", &db) != 1 || db > UINT8_MAX) {
            printf("voice: usage: voice sound <0..255> dB  (0 = sub-gate off)\n");
            return;
        }
        ambient_audio_set_shape_threshold((uint8_t)db);
        printf("voice: sound=%u dB (now %u dB)\n", (unsigned)ambient_audio_shape_threshold(),
               ambient_audio_shape_score((uint32_t)(esp_timer_get_time() / 1000)));
        return;
    }

    if (strcmp(op, "said") == 0) {
        char lines[DIALOGUE_RECALL_SLOTS * (DIALOGUE_RECALL_MAX + 4)];
        printf("voice: said %s\n",
               dialogue_style_recent_lines(lines, sizeof(lines)) ? lines : "(nothing yet)");
        return;
    }

    if (strcmp(op, "quiet") == 0) {
        unsigned secs = 0;
        if (sscanf(buf, "voice quiet %u", &secs) != 1) {
            printf("voice: usage: voice quiet <seconds>\n");
            return;
        }
        uint32_t window_ms = 0;
        uint8_t max_per = 0;
        speech_budget_get(NULL, &max_per, &window_ms);
        speech_budget_configure(secs * 1000U, max_per, window_ms);
        printf("voice: quiet=%us\n", secs);
        return;
    }

    if (strcmp(op, "budget") == 0) {
        unsigned max_per = 0;
        unsigned secs = 0;
        if (sscanf(buf, "voice budget %u %u", &max_per, &secs) != 2 || max_per > UINT8_MAX) {
            printf("voice: usage: voice budget <n> <window seconds>  (n=0 mutes)\n");
            return;
        }
        uint32_t gap_ms = 0;
        speech_budget_get(&gap_ms, NULL, NULL);
        speech_budget_configure(gap_ms, (uint8_t)max_per, secs * 1000U);
        /* Read back rather than echo the argument: the cap is clamped to the
         * history ring, and a silently ignored value is worse than none. */
        uint8_t applied = 0;
        speech_budget_get(NULL, &applied, NULL);
        printf("voice: budget=%u/%us\n", (unsigned)applied, secs);
        return;
    }

    if (strcmp(op, "repeat") == 0) {
        unsigned pct = 0;
        if (sscanf(buf, "voice repeat %u", &pct) != 1 || pct < 1U || pct > 100U) {
            printf("voice: usage: voice repeat <1..100>  (percent word overlap)\n");
            return;
        }
        dialogue_style_set_repeat_pct((uint8_t)pct);
        printf("voice: repeat=%u%%\n", (unsigned)dialogue_style_repeat_pct());
        return;
    }

    if (strcmp(op, "vary") == 0) {
        const voice_persona_t *cur = voice_persona_get();
        char directive[DIALOGUE_STYLE_MAX];
        char recent[DIALOGUE_RECENT_SLOTS * (DIALOGUE_OPENING_MAX + 4)];

        /* The avoid-list is printed on its own line as well as inside the
         * directive: it is the only part carried over from what was really
         * spoken, so it is worth seeing even when the persona has no pools. */
        printf("voice: avoiding %s\n",
               dialogue_style_recent_openings(recent, sizeof(recent)) ? recent : "(nothing yet)");
        if (dialogue_style_directive(&cur->openers, &cur->shapes, cur->avoid_lead, directive,
                                     sizeof(directive)) > 0) {
            printf("voice: vary \"%s\"\n", directive);
        } else {
            printf("voice: persona '%s' has no variation pools\n", cur->slug);
        }
        return;
    }

    if (strcmp(op, "say") == 0) {
        const char *text = strstr(buf, "say");
        text = text ? text + 3 : NULL;
        while (text && *text == ' ') {
            ++text;
        }
        if (!text || *text == '\0') {
            printf("voice: usage: voice say <text>\n");
            return;
        }
        printf("voice: speaking \"%s\"\n", text);
        if (speech_queue_post(text) != ESP_OK) {
            printf("voice: speech queue busy\n");
        }
        return;
    }

    if (strcmp(op, "name") == 0) {
        char name[VOICE_PERSONA_VOICE_MAX] = {0};
        if (sscanf(buf, "voice name %23s", name) != 1) {
            printf("voice: usage: voice name <VoiceName|->\n");
            return;
        }
        const bool clear = (strcmp(name, "-") == 0);
        if (voice_persona_set_voice(clear ? NULL : name, true) == ESP_OK) {
            printf("voice: voice=%s\n", clear ? "(persona default)" : name);
        } else {
            printf("voice: could not set voice\n");
        }
        return;
    }

    if (strcmp(op, "vad") == 0) {
        char state[8] = {0};
        if (sscanf(buf, "voice vad %7s", state) != 1 ||
            (strcmp(state, "on") != 0 && strcmp(state, "off") != 0)) {
            printf("voice: usage: voice vad on|off  (now %s)\n",
                   voice_turn_get_vad() ? "on" : "off");
            return;
        }
        const bool enable = (strcmp(state, "on") == 0);
        voice_turn_set_vad(enable);
        printf("voice: vad=%s\n", enable ? "on" : "off");
        ESP_LOGI(TAG, "voice: vad=%s", enable ? "on" : "off");
        return;
    }

    if (strcmp(op, "endpoint") == 0) {
        /* Not persisted, like every other voice knob. Whether 700 ms cuts people
         * off mid-thought or leaves the robot waiting is a judgement that needs
         * someone talking to it; the per-turn `listen: vad clip=` line is what
         * to tune against. */
        unsigned quiet_ms = 0;
        unsigned margin_db = 0;
        if (sscanf(buf, "voice endpoint %u %u", &quiet_ms, &margin_db) != 2 || quiet_ms < 100 ||
            quiet_ms > 5000 || margin_db > 40) {
            uint32_t cur_quiet = 0;
            uint8_t cur_margin = 0;
            voice_turn_get_endpoint(&cur_quiet, &cur_margin, NULL, NULL);
            printf("voice: usage: voice endpoint <100..5000 ms> <0..40 dB>  (now %u ms, %u dB;"
                   " 0 dB records to the ceiling)\n",
                   (unsigned)cur_quiet, (unsigned)cur_margin);
            return;
        }
        voice_turn_set_endpoint((uint32_t)quiet_ms, (uint8_t)margin_db);
        printf("voice: endpoint quiet=%u ms margin=%u dB\n", quiet_ms, margin_db);
        return;
    }

    if (strcmp(op, "trigger") == 0) {
        /* Not persisted. Tune by speaking and reading `mic`: its speech line
         * shows the highest voice-band share and longest run since the last
         * `mic`, so a band threshold set too high shows up as a near miss. The
         * peak band counts only frames that cleared the floor margin, so a
         * margin set too high reads as band 0%. */
        unsigned share = 0;
        unsigned margin_db = 0;
        unsigned sustain_ms = 0;
        if (sscanf(buf, "voice trigger %u %u %u", &share, &margin_db, &sustain_ms) != 3 ||
            share > 100 || margin_db > 40 || sustain_ms > 2000) {
            printf("voice: usage: voice trigger <0..100 %%> <0..40 dB> <0..2000 ms>  (now %u%%, "
                   "%u dB, %u ms; 0 drops a check)\n",
                   (unsigned)speech_trigger_share_pct(), (unsigned)speech_trigger_margin_db(),
                   (unsigned)speech_trigger_sustain_ms());
            return;
        }
        speech_trigger_set_share_pct((uint8_t)share);
        speech_trigger_set_margin_db((uint8_t)margin_db);
        speech_trigger_set_sustain_ms((uint32_t)sustain_ms);
        printf("voice: trigger band=%u%% margin=%u dB sustain=%u ms\n", share, margin_db,
               sustain_ms);
        return;
    }

    /* What voice turns may spend (voice_budget.h, issue #623). Not persisted,
     * like every other knob here — a boot comes up at the documented defaults —
     * and `resume` is the only thing short of a reboot that reopens a tripped
     * ceiling, because a fuse that reopens itself is no use on an unattended
     * board. */
    if (strcmp(op, "ration") == 0) {
        unsigned max_per = 0;
        unsigned secs = 0;
        if (sscanf(buf, "voice ration %u %u", &max_per, &secs) != 2 || max_per > UINT8_MAX ||
            secs > 86400U) {
            printf("voice: usage: voice ration <n> <window seconds>  (hands-free requests per "
                   "window; n=0 or 0 s removes the ration)\n");
            return;
        }
        voice_budget_configure_ration((uint8_t)max_per, secs * 1000U);
        /* Read back rather than echo: the cap is clamped to the history ring. */
        uint8_t applied = 0;
        voice_budget_get_ration(&applied, NULL);
        printf("voice: ration=%u/%us\n", (unsigned)applied, secs);
        return;
    }

    if (strcmp(op, "turns") == 0) {
        unsigned max_requests = 0;
        if (sscanf(buf, "voice turns %u", &max_requests) == 1) {
            voice_budget_configure_ceiling((uint32_t)max_requests);
            printf("voice: turns ceiling=%u per boot%s\n", max_requests,
                   max_requests ? "" : " (disabled)");
            ESP_LOGI(TAG, "voice: turns ceiling=%u", max_requests);
        }
        print_voice_turn_budget("voice: ", (uint32_t)(esp_timer_get_time() / 1000));
        return;
    }

    if (strcmp(op, "resume") == 0) {
        voice_budget_resume();
        printf("voice: voice-turn budget cleared — counters zeroed, ceiling reopened\n");
        ESP_LOGI(TAG, "voice: turns resumed");
        return;
    }

    if (voice_persona_set(op, true) == ESP_OK) {
        voice_turn_reset_history();
        printf("voice: persona=%s\n", op);
    } else {
        printf("voice: unknown persona '%s'\n", op);
    }
}

static void handle_gpio_cmd(const char *buf)
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
static void handle_cam_cmd(const char *buf)
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

static void handle_snap_cmd(const char *buf)
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
 * @brief `mic` / `mic dump <n>` — the microphone's counterpart to `cam`.
 *
 * Bare `mic` is how a dead microphone is told apart from a quiet room, which is
 * the single most useful thing this command does: both produce a silent robot,
 * and only one of them is a fault. A live floor that moves when you clap means
 * the mic works; a floor frozen at one value with frames still accumulating means
 * it does not.
 *
 * `mic dump` is the `snap` of the audio path, and exists for the same reason
 * frame_dump.c does: every claim about the camera was inference until somebody
 * looked at a frame, and the first one dumped disproved the sensor named in the
 * header. Nobody has listened to this microphone either.
 */
/**
 * @brief `trace` — what the robot has been capturing and calling since boot.
 *
 * The console half of activity_trace.h. The LED half is what you read while the
 * robot is driving; this is what you read afterwards to find out whether the
 * planner is being rate-limited, whether captures are silently failing, or which
 * endpoint is eating the wall clock between two planner lines.
 */
static void handle_trace_cmd(const char *buf)
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
 * `plan` — the request gate and the spend ceiling.
 *
 * Every threshold here is a console knob for the same reason `cam gainceiling`
 * and `voice scene` are: whether the robot is sensibly frugal or annoyingly
 * asleep is a judgement that needs somebody standing in the room, and a reflash
 * per trial is far too slow a loop. Like those, they deliberately do NOT persist
 * to NVS — a boot should come up at the documented default, not at whatever last
 * night's experiment left behind.
 */
static void handle_plan_cmd(const char *buf)
{
    char op[16] = {0};
    long value = 0;
    const int n = sscanf(buf, "plan %15s %ld", op, &value);

    if (n >= 1) {
        if (strcmp(op, "on") == 0 || strcmp(op, "off") == 0) {
            const bool on = (op[0] == 'o' && op[1] == 'n');
            plan_activity_set_enabled(on);
            printf("plan: dormancy %s%s\n", on ? "ENABLED" : "DISABLED",
                   on ? "" : " — a request every planner period, budget still enforced");
            return;
        }
        if (strcmp(op, "wake") == 0) {
            plan_activity_wake();
            printf("plan: woken — next planner cycle will make a request\n");
            return;
        }
        if (strcmp(op, "sleep") == 0) {
            plan_activity_sleep();
            printf("plan: dormant — still watching, no requests until something changes\n");
            return;
        }
        if (strcmp(op, "resume") == 0) {
            plan_budget_resume();
            note_gate_resume();
            plan_activity_wake();
            printf("plan: budget and note counters cleared, planner released\n");
            return;
        }
        if (n == 2 && strcmp(op, "scene") == 0) {
            uint8_t range = 0;
            plan_activity_get(NULL, &range);
            plan_activity_configure((uint8_t)value, range);
            printf("plan: wake scene threshold = %ld%s\n", value,
                   (value == 0) ? " (view no longer wakes it)" : "");
            return;
        }
        if (n == 2 && strcmp(op, "range") == 0) {
            uint8_t scene = 0;
            plan_activity_get(&scene, NULL);
            plan_activity_configure(scene, (uint8_t)value);
            printf("plan: wake range threshold = %ld cm%s\n", value,
                   (value == 0) ? " (rangefinder no longer wakes it)" : "");
            return;
        }
        if (n == 2 && strcmp(op, "requests") == 0) {
            uint32_t max_tokens = 0;
            plan_budget_get(NULL, &max_tokens);
            plan_budget_configure((uint32_t)value, max_tokens);
            printf("plan: request ceiling = %ld%s\n", value, (value == 0) ? " (no limit)" : "");
            return;
        }
        if (n == 2 && strcmp(op, "tokens") == 0) {
            uint32_t max_requests = 0;
            plan_budget_get(&max_requests, NULL);
            plan_budget_configure(max_requests, (uint32_t)value);
            printf("plan: token ceiling = %ld%s\n", value, (value == 0) ? " (no limit)" : "");
            return;
        }
        printf("plan: usage: plan | plan on|off|wake|sleep|resume | "
               "plan scene <n> | plan range <cm> | plan requests <n> | plan tokens <n>\n");
        return;
    }

    /* ---- Status ---- */
    uint8_t scene_threshold = 0;
    uint8_t range_threshold = 0;
    plan_activity_get(&scene_threshold, &range_threshold);

    uint32_t max_requests = 0;
    uint32_t max_tokens = 0;
    plan_budget_get(&max_requests, &max_tokens);

    const plan_budget_state_t budget = plan_budget_state();

    printf("plan: gate=%s state=%s\n", plan_activity_enabled() ? "on" : "OFF",
           plan_activity_dormant() ? "DORMANT" : "awake");
    printf("  cadence: step %u/%u -> %" PRIu32 " ms   (base %u ms)\n",
           (unsigned)plan_activity_step() + 1u, (unsigned)PLAN_LADDER_STEPS,
           plan_activity_period_ms(), (unsigned)PLANNER_LOOP_PERIOD_MS);
    printf("  wake on: view %u/%u at pose %c   range %u/%u cm   sound (see `mic`)   why=%s\n",
           plan_activity_scene_score(), (unsigned)scene_threshold,
           scene_pose_letter(plan_activity_pose()), plan_activity_range_score(),
           (unsigned)range_threshold, plan_activity_verdict());

    /* The assumed-cost count is printed unconditionally rather than only when
     * non-zero: it is the one field that says whether the token figure beside it
     * is measured or guessed, and a reader who has to notice an ABSENT line to
     * learn that will not notice it. */
    printf("  spent:   %" PRIu32 " requests", plan_budget_requests());
    if (max_requests != 0u) {
        printf(" / %" PRIu32, max_requests);
    } else {
        printf(" (no ceiling)");
    }
    printf(",  %llu tokens", (unsigned long long)plan_budget_tokens());
    if (max_tokens != 0u) {
        printf(" / %" PRIu32 "\n", max_tokens);
    } else {
        printf(" (no ceiling)\n");
    }
    printf("  of which %" PRIu32 " request(s) charged the assumed %u tokens because the "
           "response carried no usageMetadata\n",
           plan_budget_assumed(), (unsigned)PLAN_BUDGET_ASSUMED_TOKENS);

    if (budget != PLAN_BUDGET_OK) {
        printf("  *** BUDGET EXHAUSTED (%s) — no requests until `plan resume` ***\n",
               (budget == PLAN_BUDGET_TRIP_REQUESTS) ? "request ceiling" : "token ceiling");
    }

    /* The note gate (ADR-033): offers are what is rationed, so they lead. */
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    printf("  notes:   %" PRIu32 "/%u offered, %" PRIu32 " written, next %s", note_gate_offers(),
           (unsigned)NOTE_GATE_MAX_PER_BOOT, note_gate_written(),
           note_gate_spent() ? "after `plan resume`" : "");
    if (!note_gate_spent()) {
        printf("in %" PRIu32 " s", note_gate_wait_ms(now_ms) / 1000u);
    }
    printf(",  pending:");
    const uint32_t pending = note_gate_pending();
    if (pending == 0u) {
        printf(" none");
    }
    for (int t = NOTE_TRIGGER_NONE + 1; t < NOTE_TRIGGER_COUNT; t++) {
        if (pending & (1u << (unsigned)t)) {
            printf(" %s", note_trigger_name((note_trigger_t)t));
        }
    }
    printf("\n");
}

static void handle_mic_cmd(const char *buf)
{
    int frames = 0;
    if (sscanf(buf, "mic dump %d", &frames) == 1 || strncmp(buf, "mic dump", 8) == 0) {
        if (frames <= 0) {
            frames = (strncmp(buf, "mic dump", 8) == 0 && frames == 0) ? 1 : 0;
        }
        mic_dump_arm(frames);
        if (frames == 0) {
            printf("mic: dumping disarmed\n");
        } else {
            printf("mic: will dump the next %d frame(s) — decode with "
                   "tools/decode-mic-dump.py\n",
                   frames);
        }
        return;
    }

    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const uint32_t last = ambient_listener_last_accept_ms();

    printf("mic: driver=%s listener=%s rate=%u Hz\n", mic_pdm_is_ready() ? "ready" : "NOT READY",
           ambient_listener_is_running() ? "running" : "STOPPED", (unsigned)MIC_SAMPLE_RATE_HZ);
    printf("  level:  %d dB      floor: %d dB   (dB above 1 LSB; full scale ~90)\n",
           (int)ambient_listener_level_db(), (int)ambient_audio_floor_db());
    if (!ambient_audio_has_measurement()) {
        printf("  gate:   DEAF — no measurable frame has ever reached it, so it reports "
               "no novelty\n");
    }
    printf("  loud:   %u/%u dB    sound: %u/%u dB\n", ambient_audio_loud_score(now_ms),
           (unsigned)ambient_audio_loud_threshold(), ambient_audio_shape_score(now_ms),
           (unsigned)ambient_audio_shape_threshold());
    /* The hands-free trigger's evidence. "best" is since the previous `mic`, and
     * reading it clears it: speak once, then type `mic`, and the line says how
     * close the voice came to each threshold. */
    uint32_t best_run = 0;
    uint8_t peak_share = 0;
    speech_trigger_best(&best_run, &peak_share, true);
    printf("  speech: now band %u%% run %u ms | best since last mic: band %u%% run %u ms | "
           "need %u%% +%u dB %u ms (vad %s)\n",
           (unsigned)speech_trigger_last_share_pct(), (unsigned)speech_trigger_run_ms(),
           (unsigned)peak_share, (unsigned)best_run, (unsigned)speech_trigger_share_pct(),
           (unsigned)speech_trigger_margin_db(), (unsigned)speech_trigger_sustain_ms(),
           voice_turn_get_vad() ? "on" : "off");
    printf("  frames: %u accepted, %u muted by playback/cue, %u muted by head servos\n",
           (unsigned)ambient_listener_frames_accepted(), (unsigned)ambient_listener_frames_muted(),
           (unsigned)ambient_listener_frames_head_muted());
    if (last == 0U) {
        printf("  last:   never — no frame has reached the gate\n");
    } else {
        printf("  last:   %u ms ago\n", (unsigned)(now_ms - last));
    }
    printf("  usage: mic | mic dump <n>   (thresholds: voice loud <db> | voice sound <db> | "
           "voice trigger)\n");
}

/**
 * @brief `listen [seconds]` — one push-to-talk voice turn.
 *
 * Records, asks Gemini, and speaks the reply through the existing TTS path. The
 * window is a knob rather than a constant because "is four seconds long enough
 * to finish a sentence" is a judgement only somebody talking to the robot can
 * make, and a reflash per trial is far too slow a loop.
 */
static void handle_listen_cmd(const char *buf)
{
    if (strcmp(buf, "listen clear") == 0) {
        voice_turn_reset_history();
        printf("listen: conversational history cleared\n");
        return;
    }

    unsigned secs = 0;
    uint32_t window_ms = AUDIO_CLIP_WINDOW_MS_DEFAULT;
    if (sscanf(buf, "listen %u", &secs) == 1) {
        if (secs < AUDIO_CLIP_WINDOW_S_MIN || secs > AUDIO_CLIP_WINDOW_S_MAX) {
            printf("listen: usage: listen [%d..%d seconds] | listen clear\n",
                   AUDIO_CLIP_WINDOW_S_MIN, AUDIO_CLIP_WINDOW_S_MAX);
            return;
        }
        window_ms = (uint32_t)secs * 1000U;
    }

    const esp_err_t ret = voice_turn_request(window_ms);
    switch (ret) {
        case ESP_OK:
            printf("listen: recording %u ms after the beep — speak now\n", (unsigned)window_ms);
            break;
        case ESP_ERR_NO_MEM:
            printf("listen: busy — a turn is already in flight\n");
            break;
        case ESP_ERR_NOT_ALLOWED:
            /* `listen` is exempt from the hands-free ration, so this is always
             * the ceiling. */
            printf("listen: voice-turn spend ceiling reached — `voice resume` to continue\n");
            print_voice_turn_budget("  ", (uint32_t)(esp_timer_get_time() / 1000));
            break;
        case ESP_ERR_INVALID_STATE:
            /* Three quite different causes, and telling them apart matters:
             * a robot mid-sentence is working correctly, a missing microphone is
             * a fault, and an unstarted task means the AI phase did not come up. */
            if (!wifi_is_connected()) {
                printf("listen: no WiFi link — voice turns wait for the network\n");
            } else if (!mic_pdm_is_ready()) {
                printf("listen: no microphone — voice turns unavailable\n");
            } else if (audio_player_is_active()) {
                printf("listen: the robot is speaking — try again in a moment\n");
            } else {
                printf("listen: voice turns are not running\n");
            }
            break;
        default:
            printf("listen: rejected (%s)\n", esp_err_to_name(ret));
            break;
    }
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
 * @brief `cliff` — the front and rear floor sensors and the cliff reflex.
 *
 * `cliff` alone prints whether the reflex is on and armed, what each sensor
 * sees and the trip count; it only reads, which is why it is on the MQTT
 * read-only list. `cliff on|off` switches the reflex. Not persisted: every
 * boot starts at CLIFF_REFLEX_AT_BOOT (docs/firmware/cliff-sensors.md).
 */
static void handle_cliff_cmd(const char *buf)
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
 * @brief `idle` — the idle behaviour the executor runs on its own (ADR-029).
 *
 * `idle` alone prints the mode, what the head is doing and whether it has
 * settled (whether a frame captured now gets a dwell pose). `idle gaze on|off`
 * switches the gaze; off returns the head to centre whenever nothing directs
 * it, as before ADR-029. Not persisted: every boot starts with the gaze on.
 * `idle` alone only reads, which is why it is on the MQTT read-only list.
 */
static void handle_idle_cmd(const char *buf)
{
    char what[8] = {0};
    char state[8] = {0};
    const int n = sscanf(buf, "idle %7s %7s", what, state);

    if (n == 2 && strcmp(what, "gaze") == 0 &&
        (strcmp(state, "on") == 0 || strcmp(state, "off") == 0)) {
        const bool on = (strcmp(state, "on") == 0);
        reactive_controller_set_gaze(on);
        printf("idle: gaze %s\n", on ? "ON — the head looks around when nothing directs it"
                                     : "OFF — the head re-centres when nothing directs it");
        return;
    }
    if (n > 0) {
        printf("idle: usage: idle | idle gaze on|off\n");
        return;
    }

    reactive_telemetry_t tele = {0};
    const bool have_tele = (reactive_controller_get_telemetry(&tele) == ESP_OK);
    printf("idle: gaze=%s\n", reactive_controller_gaze_enabled() ? "on" : "off");
    if (have_tele) {
        printf("  head: %s, %s\n", reactive_head_mode_name(tele.head_mode),
               tele.head_settled ? "settled (frames get a dwell pose)"
                                 : "moving (frames are off-pose)");
    }
    printf("  usage: idle gaze on|off\n");
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
static void handle_ota_cmd(const char *buf)
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

/**
 * @brief `report` — the GlitchTip event path (ADR-033).
 *
 * `report` prints the event counters and whether the last boot's breadcrumb
 * ring survived its reset. `report test` queues a synthetic event carrying
 * this boot's breadcrumbs, to check the path end to end (robot, broker,
 * bridge, GlitchTip). Not on the MQTT read-only list: `report test` sends.
 */
static void handle_report_cmd(const char *buf)
{
    char sub[8] = {0};
    const int n = sscanf(buf, "report %7s", sub);

#if EVENT_REPORT_CRASH_COMMAND
    if (n == 1 && strcmp(sub, "crash") == 0) {
        printf("report: crashing now (abort) to test the core dump path\n");
        fflush(stdout);
        abort();
    }
#endif
    if (n == 1 && strcmp(sub, "test") == 0) {
        const esp_err_t err = event_report_send_test();
        printf("report: test event %s\n", err == ESP_OK ? "queued" : esp_err_to_name(err));
        return;
    }
    if (n == 1) {
        printf("report: usage: report | report test\n");
        return;
    }

    event_report_stats_t st = {0};
    event_report_get_stats(&st);
    printf("report: topic %s, mqtt %s\n", EVENT_REPORT_TOPIC,
           mqtt_logger_is_connected() ? "connected" : "down");
    printf("  events: queued %u, sent %u, dropped %u, pending %u\n", (unsigned)st.queued,
           (unsigned)st.sent, (unsigned)st.dropped, (unsigned)st.pending);
    if (st.boot_crumbs_present) {
        printf("  last boot's breadcrumbs: %u line(s) survived the reset\n",
               (unsigned)st.boot_crumbs);
    } else {
        printf("  last boot's breadcrumbs: none (power-on, or the reset cleared RTC memory)\n");
    }
    printf("  core dump: %s\n", st.core_dump != NULL ? st.core_dump : "none");
    printf("  usage: report test\n");
}

static void execute_console_line(const char *buf)
{
    if (strncmp(buf, "gpio", 4) == 0) {
        handle_gpio_cmd(buf);
    } else if (strncmp(buf, "voice", 5) == 0) {
        handle_voice_cmd(buf);
    } else if (strncmp(buf, "snap", 4) == 0) {
        handle_snap_cmd(buf);
    } else if (strncmp(buf, "listen", 6) == 0) {
        handle_listen_cmd(buf);
    } else if (strncmp(buf, "trace", 5) == 0) {
        handle_trace_cmd(buf);
    } else if (strncmp(buf, "mic", 3) == 0) {
        handle_mic_cmd(buf);
    } else if (strncmp(buf, "cam", 3) == 0) {
        handle_cam_cmd(buf);
    } else if (strncmp(buf, "plan", 4) == 0) {
        handle_plan_cmd(buf);
    } else if (strncmp(buf, "idle", 4) == 0) {
        handle_idle_cmd(buf);
    } else if (strncmp(buf, "cliff", 5) == 0) {
        handle_cliff_cmd(buf);
    } else if (strncmp(buf, "ota", 3) == 0) {
        handle_ota_cmd(buf);
    } else if (strncmp(buf, "report", 6) == 0) {
        handle_report_cmd(buf);
    } else if (strncmp(buf, "sound", 5) == 0 || strncmp(buf, "servo", 5) == 0 ||
               strncmp(buf, "led", 3) == 0) {
        handle_periph_cmd(buf);
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
 * @brief `mqtt` — broker-credential state and NVS provisioning (issue #626).
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
        printf("mqtt: commands=%s (credentials from %s, user=%s) broker=%s\n",
               mqtt_command_access_name(s_mqtt_access), get_mqtt_credentials_source(),
               get_mqtt_username() ? get_mqtt_username() : "-", MQTT_BROKER_URI);
        printf("  usage: mqtt auth <user> <pass> | mqtt auth clear   (applied at next boot;\n");
        printf("         max 32/64 chars, no spaces; NVS wins over credentials.h)\n");
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
    printf("mqtt: usage: mqtt | mqtt auth <user> <pass> | mqtt auth clear\n");
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
        if (s_improv_active) {
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

        /* Feed every byte to the Improv parser first; it ignores anything
         * that is not an Improv packet, so ASCII console commands still reach
         * the switch below. The bytes of a packet it claims never become a
         * console line (console_line.h): a Send Wi-Fi settings packet carries
         * the password, and a line is logged below and mirrored to UART0 and
         * the GlitchTip breadcrumb ring. */
        const bool improv_owned = s_improv_active && improv_wifi_process_byte((uint8_t)ch);

        const char *const buf = console_line_feed(&line, (char)ch, improv_owned);
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
    if (mdns_hostname_set("robocar-unified") != ESP_OK ||
        mdns_instance_name_set("Robocar Unified") != ESP_OK) {
        ESP_LOGW(TAG, "Could not set mDNS hostname");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "mDNS: robocar-unified.local");

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
/** First link-up creates the client; later ones resume it. */
static void start_mqtt_service(void)
{
    static bool s_created;
    if (s_created) {
        mqtt_logger_resume();
        return;
    }
    const mqtt_logger_config_t mqtt_cfg = {
        .broker_uri = MQTT_BROKER_URI,
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
        return;
    }
    s_created = true;
    /* create_tasks() runs before the supervisor starts, so the queues the
     * command handler dispatches into already exist. */
    mqtt_logger_set_command_handler(mqtt_command_received);
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
        const net_actions_t act = net_supervisor_step(&sup, link_up, now_ms);

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
#endif
        }
        if (act.start_mqtt) {
#if MQTT_LOGGING_ENABLED
            start_mqtt_service();
#endif
        }
        if (act.stop_mqtt) {
#if MQTT_LOGGING_ENABLED
            mqtt_logger_pause();
#endif
        }
        if (act.arm_improv && s_improv_ready) {
            /* Ready again: the remembered state may still be PROVISIONED from
             * an earlier provisioning, and command_task's announcement repeats
             * whatever is remembered. */
            improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);
            s_improv_active = true;
            ESP_LOGW(TAG, "No WiFi link — Improv provisioning armed (ESP Web Tools)");
        }
        if (act.disarm_improv) {
            s_improv_active = false;
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
    /* Before anything logs: the previous boot's last log lines are still in
     * RTC memory, and the first ESP_LOG line from here on overwrites them
     * (ADR-033). Makes no logging calls itself. */
    event_report_boot_capture();

    /* First, so the banner and every boot phase after it reach J15 (UART0 TX)
     * as well as the USB console (issue #798). It needs only PSRAM, which the
     * startup code has already brought up. Non-fatal: without it the log
     * stays on USB alone, as it always has. */
    const esp_err_t mirror_ret = log_mirror_init();

    ESP_LOGI(TAG, "=== Robocar Unified (XIAO ESP32-S3 Sense) ===");
    if (mirror_ret != ESP_OK) {
        ESP_LOGW(TAG, "UART0 log mirror unavailable (%s) — log on USB only",
                 esp_err_to_name(mirror_ret));
    }
    log_boot_banner();

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

    /* Boot diagnostic: prove the drivetrain turns over before any later phase
     * claims the robot is ready. Subject to the obstacle reflex, and skipped
     * gracefully when the motors are absent or the executor never started. */
    run_wheel_exercise();

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
