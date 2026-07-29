/**
 * @file main.c
 * @brief Unified robocar firmware for XIAO ESP32-S3 Sense
 *
 * Hierarchical AI controller architecture:
 *   - Core 0: motor control, peripheral I/O, serial commands, reactive executor (30 Hz)
 *   - Core 1: camera capture, Gemini planner (PLANNER_LOOP_PERIOD_MS, 15 s default), WiFi/MQTT/OTA
 *
 * The planner (planner_task, Core 1) captures frames, calls Gemini ER 1.6,
 * and writes structured goals into goal_state.  The reactive executor
 * (reactive_controller, Core 0) reads goals at 30 Hz and drives motors.
 * Serial / MQTT commands remain available for manual override.
 */

#include <string.h>
#include "esp_app_desc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"

#include "audio_player.h"
#include "buzzer.h"
#include "camera.h"
#include "config.h"
#include "credentials_loader.h"
#include "credentials_validator.h"
#include "dialogue_style.h"
#include "frame_dump.h"
#include "gemini_tts.h"
#include "goal_state.h"
#include "gpio_expander.h"
#include "i2c_bus.h"
#include "improv_wifi.h"
#include "led_controller.h"
#include "mdns.h"
/* Only for motor_controller_init() in the hardware phase — motor *output* is
 * the reactive executor's job, and nothing here writes the motors directly. */
#include "motor_controller.h"
#include "mqtt_logger.h"
#include "ota_manager.h"
#include "pin_config.h"
#include "planner_task.h"
#include "reactive_controller.h"
#include "self_report.h"
#include "servo_controller.h"
#include "speech_budget.h"
#include "speech_queue.h"
#include "voice_persona.h"
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
static bool s_improv_active;

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

    while (1) {
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
        if (sscanf(buf, "sound %15s", arg) != 1) {
            printf("sound: usage: sound beep|melody|alert\n");
            return;
        }
        dispatch_sound(arg);
        printf("sound: %s\n", arg);
        return;
    }

    if (strncmp(buf, "servo", 5) == 0) {
        if (sscanf(buf, "servo %15s %d", arg, &a) != 2) {
            printf("servo: usage: servo pan|tilt <deg>\n");
            return;
        }
        periph_cmd_t cmd = {.angle = (int16_t)a};
        if (strcmp(arg, "pan") == 0) {
            cmd.type = PERIPH_CMD_SERVO_PAN;
        } else if (strcmp(arg, "tilt") == 0) {
            cmd.type = PERIPH_CMD_SERVO_TILT;
        } else {
            printf("servo: usage: servo pan|tilt <deg>\n");
            return;
        }
        dispatch_periph_cmd(&cmd);
        printf("servo: %s=%d\n", arg, a);
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
//   gpio mode <pin> in|up|out - set direction (up = input with pullup)
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
 *
 * Switching and auditioning are runtime rather than compile-time because only a
 * listener can judge a voice; a reflash per candidate is far too slow a loop.
 *
 * `voice vary` exists for the same reason one step further in: the variation
 * pools (dialogue_style.h) only pay off if their entries read naturally in the
 * persona's language, and the alternative way to see what the model is being
 * told is to wait out a 15 s planner period per sample.
 *
 * `quiet` / `budget` / `repeat` are the same argument again: how talkative and
 * how repetitive the robot is allowed to be are judgements that need a person in
 * the room, and every trial otherwise costs a rebuild plus a flash. They are not
 * persisted — a boot comes up at the documented default rather than at whatever
 * an experiment left behind. */
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
        if (wait_ms == UINT32_MAX) {
            printf("  speech: muted\n");
        } else {
            printf("  speech: %s (next in %us)\n", wait_ms ? "waiting" : "allowed now",
                   (unsigned)(wait_ms / 1000U));
        }

        printf("  usage: voice <slug> | say <text> | name <VoiceName|-> | vary | said\n");
        printf("         voice quiet <s> | voice budget <n> <s> | voice repeat <pct>\n");
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

    if (voice_persona_set(op, true) == ESP_OK) {
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
        if (ret == ESP_OK)
            printf("gpio: pin %u mode=%s\n", pin, arg);
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
    } else {
        printf("cam: usage: cam | cam gainceiling 0-%d | cam ae -2..2 | cam brightness -2..2\n",
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

// ========================================
// Improv WiFi provisioning (serial)
// ========================================
/* Improv Serial rides the same UART as the console: ESP Web Tools opens the
 * port after flashing and speaks the protocol over it, which is why the byte
 * feed lives inside command_task rather than in a task of its own. */
static void on_improv_credentials(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Improv: credentials received for SSID '%s'", ssid ? ssid : "");
    improv_wifi_send_state(IMPROV_STATE_PROVISIONING);

    if (!ssid || strlen(ssid) == 0) {
        improv_wifi_send_error(IMPROV_ERROR_INVALID_RPC);
        return;
    }

    if (wifi_connect(ssid, password ? password : "") != ESP_OK) {
        ESP_LOGW(TAG, "Improv: could not join '%s'", ssid);
        improv_wifi_send_error(IMPROV_ERROR_UNABLE_CONNECT);
        return;
    }

    /* Persist only after the credentials are proven to work, so a typo cannot
     * overwrite a known-good stored network. */
    if (!credentials_nvs_save_wifi(ssid, password ? password : "")) {
        improv_wifi_send_error(IMPROV_ERROR_UNKNOWN);
        return;
    }
    credentials_reload();

    s_improv_active = false;
    improv_wifi_send_state(IMPROV_STATE_PROVISIONED);
    improv_wifi_send_provisioned_result(NULL);
    ESP_LOGI(TAG, "Improv: provisioning complete");
}

// ========================================
// Serial command task (Core 0, priority 5)
// ========================================
static void command_task(void *pvParameters)
{
    (void)pvParameters;
    char buf[32];
    int buf_pos = 0;
    int64_t last_improv_announce = 0;

    ESP_LOGI(TAG, "Command task started on core %d", xPortGetCoreID());

    while (1) {
        /* While unprovisioned, announce ourselves ~1 Hz so ESP Web Tools can
         * discover the device on the port it just flashed. */
        if (s_improv_active) {
            int64_t now = esp_timer_get_time();
            if (now - last_improv_announce > 1000000LL) {
                improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);
                last_improv_announce = now;
            }
        }

        int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_SHORT_MS));
            continue;
        }

        /* Feed every byte to the Improv parser first; it silently ignores
         * anything that is not a well-formed Improv packet, so ASCII console
         * commands still reach the switch below. */
        if (s_improv_active) {
            improv_wifi_process_byte((uint8_t)ch);
        }

        if (ch == '\n' || ch == '\r') {
            if (buf_pos > 0) {
                buf[buf_pos] = '\0';
                ESP_LOGI(TAG, "Serial cmd: %s", buf);

                // Single-letter movement commands (manual override / debug)
                if (buf_pos == 1) {
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
                } else if (strncmp(buf, "gpio", 4) == 0) {
                    handle_gpio_cmd(buf);
                } else if (strncmp(buf, "voice", 5) == 0) {
                    handle_voice_cmd(buf);
                } else if (strncmp(buf, "snap", 4) == 0) {
                    handle_snap_cmd(buf);
                } else if (strncmp(buf, "cam", 3) == 0) {
                    handle_cam_cmd(buf);
                } else if (strncmp(buf, "sound", 5) == 0 || strncmp(buf, "servo", 5) == 0 ||
                           strncmp(buf, "led", 3) == 0) {
                    handle_periph_cmd(buf);
                }

                buf_pos = 0;
            }
        } else if (buf_pos < (int)sizeof(buf) - 1) {
            buf[buf_pos++] = (char)ch;
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

static esp_err_t init_hardware(void)
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
        ESP_RETURN_ON_ERROR(buzzer_init(), TAG, "Buzzer init failed");
        buzzer_beep();
        return ESP_OK;
    }

    // Optional hardware: returns ESP_OK even when no expander is fitted
    ESP_RETURN_ON_ERROR(gpio_expander_init(), TAG, "GPIO expander init failed");
    ESP_RETURN_ON_ERROR(motor_controller_init(), TAG, "Motor init failed");
    ESP_RETURN_ON_ERROR(led_controller_init(), TAG, "LED init failed");
    ESP_RETURN_ON_ERROR(servo_controller_init(), TAG, "Servo init failed");
    ESP_RETURN_ON_ERROR(buzzer_init(), TAG, "Buzzer init failed");

    // Startup indication
    led_set_both(&LED_COLOR_GREEN);
    buzzer_beep();

    return ESP_OK;
}

static esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Phase 2: Camera");
    return camera_init();
}

static esp_err_t init_network(void)
{
    ESP_LOGI(TAG, "Phase 3: WiFi + network");

    /* Warns when credentials.h still holds placeholders, which is exactly the
     * state a web-flasher build boots in — the message tells the user that
     * Improv provisioning (started below) is the expected next step. */
    validate_credentials_at_runtime();

    // Load credentials and connect WiFi
    credentials_t creds = {0};
    load_credentials(&creds);
    wifi_init();
    if (creds.credentials_loaded && strlen(creds.wifi_ssid) > 0) {
        wifi_connect(creds.wifi_ssid, creds.wifi_password);
    }

    /* Start Improv whenever we are not on a network — both for a board that has
     * never been provisioned and for one whose stored SSID no longer exists.
     * A firmware built without credentials (CI, the web flasher) has no other
     * way to be given any, which is the whole point of the protocol. */
    if (!wifi_is_connected()) {
        ESP_LOGW(TAG, "No WiFi connection — starting Improv provisioning");
        esp_err_t improv_ret = improv_wifi_init(on_improv_credentials);
        if (improv_ret == ESP_OK) {
            s_improv_active = true;
            ESP_LOGI(TAG, "Improv WiFi ready — connect with ESP Web Tools to provision");
        } else {
            ESP_LOGE(TAG, "improv_wifi_init failed (%s) — no provisioning path",
                     esp_err_to_name(improv_ret));
        }
    }

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

    // How often the planner is even offered the `speak` tool. Must be reset
    // before the planner task starts, since it reads the budget while building
    // its first request.
    speech_budget_init();

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

    // reactive_controller spawns the 30 Hz executor on Core 0. This one *is*
    // required: without the executor nothing drives the motors at all.
    ESP_RETURN_ON_ERROR(reactive_controller_init(), TAG, "reactive_controller_init failed");

    /* planner_task spawns the Gemini planner on Core 1 (requires WiFi + an API
     * key). Non-fatal by design: a board flashed from the web flasher has no
     * credentials at all, and panicking here would boot-loop the documented
     * default configuration. Without a planner the executor simply holds STOP,
     * which is safe — and the console, Improv provisioning and self-report all
     * stay reachable so the device can be told what it is missing. */
    esp_err_t planner_ret = planner_task_init();
    if (planner_ret != ESP_OK) {
        ESP_LOGW(TAG, "planner_task_init failed (%s) — AI planner disabled, robot holds STOP",
                 esp_err_to_name(planner_ret));
    }
    self_report_note_init(SELF_REPORT_SUBSYS_PLANNER, planner_ret == ESP_OK);

    return ESP_OK;
}

/**
 * @brief Start the network-dependent background services.
 *
 * Both are non-fatal and both are skipped without a link: MQTT would only
 * buffer into a queue nothing drains, and the OTA poller needs to reach GitHub.
 */
static void init_network_services(void)
{
    if (!wifi_is_connected()) {
        ESP_LOGW(TAG, "No WiFi — skipping MQTT logging and OTA");
        return;
    }

#if MQTT_LOGGING_ENABLED
    const mqtt_logger_config_t mqtt_cfg = {
        .broker_uri = MQTT_BROKER_URI,
        .client_id = MQTT_CLIENT_ID,
        .username = MQTT_USERNAME,
        .password = MQTT_PASSWORD,
        .log_topic = MQTT_LOG_TOPIC_BASE,
        .status_topic = MQTT_STATUS_TOPIC,
        .command_topic = MQTT_COMMAND_TOPIC,
        .buffer_size = MQTT_LOG_BUFFER_SIZE,
        .keepalive_interval = MQTT_KEEPALIVE_INTERVAL,
        .qos_level = MQTT_QOS_LEVEL,
        .min_level = MQTT_MIN_LOG_LEVEL,
        .retain_status = MQTT_RETAIN_STATUS,
    };
    esp_err_t mqtt_ret = mqtt_logger_init(&mqtt_cfg);
    if (mqtt_ret != ESP_OK) {
        ESP_LOGW(TAG, "mqtt_logger_init failed (%s) — remote logging disabled",
                 esp_err_to_name(mqtt_ret));
    }
#endif

#if OTA_ENABLED
    /* Starting the OTA manager is what arms ota_manager_confirm_valid(). With
     * CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y an image that is never confirmed
     * is rolled back on the next boot, so leaving this uncalled would make any
     * future OTA update silently revert. */
    esp_err_t ota_ret = ota_manager_init();
    if (ota_ret != ESP_OK) {
        ESP_LOGW(TAG, "ota_manager_init failed (%s) — updates disabled", esp_err_to_name(ota_ret));
    }
#endif
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

// ========================================
// Application entry point
// ========================================
void app_main(void)
{
    ESP_LOGI(TAG, "=== Robocar Unified (XIAO ESP32-S3 Sense) ===");
    ESP_LOGI(TAG, "Firmware version: %s", esp_app_get_description()->version);

    ESP_ERROR_CHECK(init_nvs());

    // init_hardware() degrades gracefully (bare board still returns ESP_OK), so
    // read the real bus state from its accessor for the self-report note.
    ESP_ERROR_CHECK(init_hardware());
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

    init_network_services();

    // Spoken self-introduction + status diagnostic (announces once voice-able,
    // re-announces on health change). Non-fatal: a robot that cannot start the
    // monitor still drives.
    if (self_report_start() != ESP_OK) {
        ESP_LOGW(TAG, "self_report_start failed — no spoken self-report");
    }

    ESP_LOGI(TAG, "=== System ready ===");

    // app_main returns; FreeRTOS scheduler runs tasks
}
