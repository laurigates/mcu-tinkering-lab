/**
 * @file main.c
 * @brief ESP32-S3 Gamepad Synth — Bluetooth controller drives a piezo speaker
 *
 * Four sound modes cycled via View button (Xbox) / Share (PS):
 *   1. Theremin  — sticks control pitch, vibrato, triggers bend pitch
 *   2. Scale     — face buttons + d-pad play notes, shoulders shift octave
 *   3. Arpeggio  — face buttons pick chord, stick controls speed/pattern
 *   4. Retro SFX — buttons trigger classic game sound effects
 *
 * Bluepad32 runs on Core 0 (BTstack event loop, blocks forever).
 * Sound engine runs on Core 1 at 50 Hz.
 *
 * Tested with: Xbox Series X/S controller (BLE, firmware v5.15+)
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

/* Bluepad32 headers */
#include "uni_bt.h"
#include "uni_controller.h"
#include "uni_esp32.h"
#include "uni_gamepad.h"
#include "uni_hid_device.h"
#include "uni_platform.h"

static const char *TAG = "gamepad_synth";

/* ── Pin Definitions ─────────────────────────────────────── */

#define PIEZO_PIN GPIO_NUM_4
#define LED_PIN GPIO_NUM_2

/* ── LEDC Configuration ──────────────────────────────────── */

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_DUTY_ON 128 /* 50% duty = square wave */

/* ── Bluepad32 Button Constants ──────────────────────────── */

#define BTN_A (1 << 0)          /* A (Xbox) / Cross (PS) */
#define BTN_B (1 << 1)          /* B (Xbox) / Circle (PS) */
#define BTN_X (1 << 2)          /* X (Xbox) / Square (PS) */
#define BTN_Y (1 << 3)          /* Y (Xbox) / Triangle (PS) */
#define BTN_SHOULDER_L (1 << 4) /* LB (Xbox) / L1 (PS) */
#define BTN_SHOULDER_R (1 << 5) /* RB (Xbox) / R1 (PS) */

#define DPAD_UP (1 << 0)
#define DPAD_DOWN (1 << 1)
#define DPAD_RIGHT (1 << 2)
#define DPAD_LEFT (1 << 3)

#define MISC_BACK (1 << 1)   /* View (Xbox) / Share (PS) */
#define MISC_HOME (1 << 2)   /* Xbox button / PS button */
#define MISC_SELECT (1 << 3) /* Menu (Xbox) / Options (PS) */

/* ── Sound Parameters ────────────────────────────────────── */

#define MIN_FREQ 100
#define MAX_FREQ 2000
#define STICK_DEADZONE 50 /* ignore stick values below this */
#define STICK_MAX 512
#define TRIGGER_MAX 1023

#define SOUND_TASK_HZ 50
#define SOUND_TASK_PERIOD_MS (1000 / SOUND_TASK_HZ)

/* ── Musical Note Table (C4–B6, equal temperament) ───────── */

static const uint16_t NOTE_FREQ[] = {
    262,  277,  294,  311,  330,  349,  370,  392,  415,  440,  466, 494, /* C4–B4 */
    523,  554,  587,  622,  659,  698,  740,  784,  831,  880,  932, 988, /* C5–B5 */
    1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760,           /* C6–A6 */
    1865, 1976,                                                           /* A#6–B6 */
};
#define NOTE_COUNT (sizeof(NOTE_FREQ) / sizeof(NOTE_FREQ[0]))

/* C major scale: indices into NOTE_FREQ within one octave */
static const uint8_t SCALE_MAJOR[] = {0, 2, 4, 5, 7, 9, 11, 12};
#define SCALE_LEN 8

/* Chord patterns (semitone offsets from root) */
static const uint8_t CHORD_MAJOR[] = {0, 4, 7, 12};
static const uint8_t CHORD_MINOR[] = {0, 3, 7, 12};
static const uint8_t CHORD_7TH[] = {0, 4, 7, 10};
static const uint8_t CHORD_DIM[] = {0, 3, 6, 9};
#define CHORD_LEN 4

/* ── Shared Gamepad State ────────────────────────────────── */

typedef struct {
    uint16_t buttons;
    uint8_t dpad;
    uint8_t misc_buttons;
    int16_t axis_x;   /* left stick X: -511..512 */
    int16_t axis_y;   /* left stick Y: -511..512 */
    int16_t axis_rx;  /* right stick X */
    int16_t axis_ry;  /* right stick Y */
    int16_t brake;    /* L2 trigger: 0..1023 */
    int16_t throttle; /* R2 trigger: 0..1023 */
    bool connected;
} gamepad_state_t;

static volatile gamepad_state_t s_gp;

/* ── Sound Modes ─────────────────────────────────────────── */

typedef enum {
    MODE_THEREMIN = 0,
    MODE_SCALE,
    MODE_ARPEGGIO,
    MODE_SFX,
    MODE_COUNT,
} sound_mode_t;

static sound_mode_t s_mode = MODE_THEREMIN;

/* Arpeggiator state */
static const uint8_t *s_arp_chord = CHORD_MAJOR;
static int s_arp_root = 0;      /* semitone offset from C4 */
static int s_arp_index = 0;     /* current note in chord */
static int s_arp_direction = 1; /* 1=up, -1=down */
static bool s_arp_running = false;
static uint32_t s_arp_last_ms = 0;

/* SFX state */
typedef struct {
    float freq;
    float freq_end;
    float step;
    int ticks_left;
} sfx_state_t;
static sfx_state_t s_sfx = {0};

/* Scale octave: 0=C4, 1=C5, 2=C6 */
static int s_octave = 1;

/* Mode switch debounce */
static uint8_t s_prev_misc = 0;

/* ── Tone Helpers ────────────────────────────────────────── */

static void tone_play(uint32_t freq_hz)
{
    if (freq_hz < MIN_FREQ)
        freq_hz = MIN_FREQ;
    if (freq_hz > MAX_FREQ)
        freq_hz = MAX_FREQ;
    ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq_hz);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ON);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void tone_stop(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static uint16_t note_at(int semitone)
{
    if (semitone < 0)
        semitone = 0;
    if (semitone >= (int)NOTE_COUNT)
        semitone = NOTE_COUNT - 1;
    return NOTE_FREQ[semitone];
}

/* Map a value from [in_min..in_max] to [out_min..out_max] */
static float map_range(float val, float in_min, float in_max, float out_min, float out_max)
{
    if (val < in_min)
        val = in_min;
    if (val > in_max)
        val = in_max;
    return out_min + (out_max - out_min) * ((val - in_min) / (in_max - in_min));
}

/* ── LED Helpers ─────────────────────────────────────────── */

static void led_on(void)
{
    gpio_set_level(LED_PIN, 1);
}
static void led_off(void)
{
    gpio_set_level(LED_PIN, 0);
}

static void led_blink(int count, int on_ms, int off_ms)
{
    for (int i = 0; i < count; i++) {
        led_on();
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        led_off();
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

/* ── Mode 1: Theremin ────────────────────────────────────── */

static void mode_theremin(const gamepad_state_t *gp)
{
    int16_t ly = gp->axis_y;
    int16_t lx = gp->axis_x;
    int16_t ry = gp->axis_ry;

    /* Dead zone check — silence if stick is centered */
    if (abs(ly) < STICK_DEADZONE && abs(lx) < STICK_DEADZONE && gp->brake == 0 &&
        gp->throttle == 0) {
        tone_stop();
        led_off();
        return;
    }

    /* Base pitch from left stick Y */
    float pitch = map_range((float)ly, -STICK_MAX, STICK_MAX, MIN_FREQ, MAX_FREQ);

    /* Pitch bend from triggers: LT bends down, RT bends up */
    float bend = map_range((float)gp->throttle, 0, TRIGGER_MAX, 0, 300) -
                 map_range((float)gp->brake, 0, TRIGGER_MAX, 0, 300);
    pitch += bend;

    /* Vibrato from left stick X (depth) and right stick Y (speed) */
    float vib_depth = map_range((float)abs(lx), 0, STICK_MAX, 0, 100);
    float vib_speed = map_range((float)abs(ry), 0, STICK_MAX, 1, 20);

    /* Simple sine vibrato using tick count */
    static uint32_t tick = 0;
    tick++;
    float vib = vib_depth * sinf(2.0f * 3.14159f * vib_speed * (float)tick / SOUND_TASK_HZ);
    pitch += vib;

    tone_play((uint32_t)pitch);
    led_on();
}

/* ── Mode 2: Scale Player ────────────────────────────────── */

static void mode_scale(const gamepad_state_t *gp)
{
    /* LB/RB shift octave (rising edge) */
    static uint16_t prev_buttons = 0;
    uint16_t pressed = gp->buttons & ~prev_buttons;
    prev_buttons = gp->buttons;

    if (pressed & BTN_SHOULDER_L && s_octave > 0)
        s_octave--;
    if (pressed & BTN_SHOULDER_R && s_octave < 2)
        s_octave++;

    /* Map buttons to scale degrees */
    int note_idx = -1;
    if (gp->buttons & BTN_A)
        note_idx = 0; /* A = Do */
    if (gp->buttons & BTN_B)
        note_idx = 1; /* B = Re */
    if (gp->buttons & BTN_X)
        note_idx = 2; /* X = Mi */
    if (gp->buttons & BTN_Y)
        note_idx = 3; /* Y = Fa */
    if (gp->dpad & DPAD_UP)
        note_idx = 4; /* Sol */
    if (gp->dpad & DPAD_RIGHT)
        note_idx = 5; /* La */
    if (gp->dpad & DPAD_DOWN)
        note_idx = 6; /* Ti */
    if (gp->dpad & DPAD_LEFT)
        note_idx = 7; /* Do (high) */

    if (note_idx < 0) {
        tone_stop();
        led_off();
        return;
    }

    int semitone = (s_octave * 12) + SCALE_MAJOR[note_idx];
    float freq = (float)note_at(semitone);

    /* Pitch bend from right stick Y */
    float bend = map_range((float)gp->axis_ry, -STICK_MAX, STICK_MAX, -50, 50);
    freq += bend;

    tone_play((uint32_t)freq);
    led_on();
}

/* ── Mode 3: Arpeggiator ────────────────────────────────── */

static void mode_arpeggio(const gamepad_state_t *gp)
{
    /* Face buttons select chord (rising edge) */
    static uint16_t prev_buttons = 0;
    uint16_t pressed = gp->buttons & ~prev_buttons;
    prev_buttons = gp->buttons;

    if (pressed & BTN_A)
        s_arp_chord = CHORD_MAJOR; /* A = major */
    if (pressed & BTN_B)
        s_arp_chord = CHORD_MINOR; /* B = minor */
    if (pressed & BTN_X)
        s_arp_chord = CHORD_7TH; /* X = 7th */
    if (pressed & BTN_Y)
        s_arp_chord = CHORD_DIM; /* Y = dim */

    /* LB/RB shift octave */
    if (pressed & BTN_SHOULDER_L && s_octave > 0)
        s_octave--;
    if (pressed & BTN_SHOULDER_R && s_octave < 2)
        s_octave++;

    /* D-pad transpose root note */
    if (gp->dpad & DPAD_UP) {
        static uint32_t last_up = 0;
        uint32_t now = xTaskGetTickCount();
        if (now - last_up > pdMS_TO_TICKS(200)) {
            s_arp_root++;
            if (s_arp_root > 11)
                s_arp_root = 11;
            last_up = now;
        }
    }
    if (gp->dpad & DPAD_DOWN) {
        static uint32_t last_down = 0;
        uint32_t now = xTaskGetTickCount();
        if (now - last_down > pdMS_TO_TICKS(200)) {
            s_arp_root--;
            if (s_arp_root < 0)
                s_arp_root = 0;
            last_down = now;
        }
    }

    /* RT trigger toggles arpeggio on/off (rising edge) */
    static bool prev_r2 = false;
    bool r2 = gp->throttle > 100;
    if (r2 && !prev_r2) {
        s_arp_running = !s_arp_running;
        s_arp_index = 0;
        s_arp_direction = 1;
        if (!s_arp_running)
            tone_stop();
    }
    prev_r2 = r2;

    if (!s_arp_running) {
        led_off();
        return;
    }

    /* Speed from left stick Y: 50ms (fast) to 500ms (slow) */
    float speed_ms = map_range((float)gp->axis_y, -STICK_MAX, STICK_MAX, 50, 500);

    /* Pattern from left stick X quadrant */
    /* Left = down, right = up, center = up-down */
    int pattern = 0; /* 0=up, 1=down, 2=up-down */
    if (gp->axis_x < -STICK_DEADZONE)
        pattern = 1;
    else if (gp->axis_x > STICK_DEADZONE)
        pattern = 0;
    else
        pattern = 2;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now_ms - s_arp_last_ms >= (uint32_t)speed_ms) {
        s_arp_last_ms = now_ms;

        int semitone = (s_octave * 12) + s_arp_root + s_arp_chord[s_arp_index];
        tone_play(note_at(semitone));
        led_on();

        /* Advance index based on pattern */
        if (pattern == 0) {
            s_arp_index = (s_arp_index + 1) % CHORD_LEN;
        } else if (pattern == 1) {
            s_arp_index--;
            if (s_arp_index < 0)
                s_arp_index = CHORD_LEN - 1;
        } else {
            s_arp_index += s_arp_direction;
            if (s_arp_index >= CHORD_LEN) {
                s_arp_index = CHORD_LEN - 2;
                s_arp_direction = -1;
            } else if (s_arp_index < 0) {
                s_arp_index = 1;
                s_arp_direction = 1;
            }
        }
    }
}

/* ── Mode 4: Retro SFX ──────────────────────────────────── */

static void sfx_start(float start, float end, int ticks)
{
    s_sfx.freq = start;
    s_sfx.freq_end = end;
    s_sfx.step = (end - start) / (float)ticks;
    s_sfx.ticks_left = ticks;
}

static void sfx_tick(void)
{
    if (s_sfx.ticks_left <= 0) {
        tone_stop();
        led_off();
        return;
    }
    tone_play((uint32_t)s_sfx.freq);
    led_on();
    s_sfx.freq += s_sfx.step;
    s_sfx.ticks_left--;
}

static void mode_sfx(const gamepad_state_t *gp)
{
    /* Speed multiplier from RT trigger: 0.5x to 2x */
    float speed = map_range((float)gp->throttle, 0, TRIGGER_MAX, 1.0f, 0.3f);
    int base_ticks = 25;
    int ticks = (int)((float)base_ticks * speed);
    if (ticks < 5)
        ticks = 5;

    /* Trigger SFX on button press (rising edge) */
    static uint16_t prev_btn = 0;
    static uint8_t prev_dpad = 0;
    uint16_t btn_pressed = gp->buttons & ~prev_btn;
    uint8_t dpad_pressed = gp->dpad & ~prev_dpad;
    prev_btn = gp->buttons;
    prev_dpad = gp->dpad;

    /* A = Laser (descending sweep) */
    if (btn_pressed & BTN_A)
        sfx_start(1800, 200, ticks);

    /* B = Explosion (descending with low end) */
    if (btn_pressed & BTN_B)
        sfx_start(800, 100, ticks * 2);

    /* X = Power-up (ascending sweep) */
    if (btn_pressed & BTN_X)
        sfx_start(200, 1600, ticks);

    /* Y = Coin (high double-beep handled via short sweep) */
    if (btn_pressed & BTN_Y)
        sfx_start(1200, 1800, ticks / 2);

    /* D-pad Up = Siren (oscillating — start high, go low, we'll reverse) */
    if (dpad_pressed & DPAD_UP)
        sfx_start(400, 1200, ticks * 3);

    /* D-pad Down = Engine (low rumble) */
    if (dpad_pressed & DPAD_DOWN)
        sfx_start(100, 250, ticks * 3);

    /* D-pad Left = Jump (quick ascending chirp) */
    if (dpad_pressed & DPAD_LEFT)
        sfx_start(300, 1400, ticks / 2);

    /* D-pad Right = Warp (sweep up then down via two-phase) */
    if (dpad_pressed & DPAD_RIGHT)
        sfx_start(400, 1800, ticks);

    sfx_tick();
}

/* ── Bluepad32 Custom Platform ───────────────────────────── */

static void plat_init(int argc, const char **argv)
{
    (void)argc;
    (void)argv;
    ESP_LOGI(TAG, "Bluepad32 platform initialized");
}

static void plat_on_init_complete(void)
{
    ESP_LOGI(TAG, "Scanning for BLE controllers...");
    uni_bt_enable_new_connections_safe(true);
}

static void plat_on_device_connected(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Device connected: %p", d);
}

static void plat_on_device_disconnected(uni_hid_device_t *d)
{
    ESP_LOGW(TAG, "Device disconnected: %p", d);
    gamepad_state_t empty = {0};
    memcpy((void *)&s_gp, &empty, sizeof(s_gp));
}

static uni_error_t plat_on_device_ready(uni_hid_device_t *d)
{
    ESP_LOGI(TAG, "Device ready: VID=0x%04X PID=0x%04X", uni_hid_device_get_vendor_id(d),
             uni_hid_device_get_product_id(d));
    s_gp.connected = true;
    return UNI_ERROR_SUCCESS;
}

static void plat_on_controller_data(uni_hid_device_t *d, uni_controller_t *ctl)
{
    (void)d;
    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD)
        return;
    const uni_gamepad_t *gp = &ctl->gamepad;
    s_gp.buttons = gp->buttons;
    s_gp.dpad = gp->dpad;
    s_gp.misc_buttons = gp->misc_buttons;
    s_gp.axis_x = gp->axis_x;
    s_gp.axis_y = gp->axis_y;
    s_gp.axis_rx = gp->axis_rx;
    s_gp.axis_ry = gp->axis_ry;
    s_gp.brake = gp->brake;
    s_gp.throttle = gp->throttle;
    s_gp.connected = true;
}

static int32_t plat_get_property(uni_platform_property_t key)
{
    (void)key;
    return -1;
}

static void plat_on_oob_event(uni_platform_oob_event_t event, void *data)
{
    (void)data;
    ESP_LOGD(TAG, "OOB event: %d", event);
}

static struct uni_platform s_platform = {
    .name = "gamepad_synth",
    .init = plat_init,
    .on_init_complete = plat_on_init_complete,
    .on_device_connected = plat_on_device_connected,
    .on_device_disconnected = plat_on_device_disconnected,
    .on_device_ready = plat_on_device_ready,
    .on_gamepad_data = NULL,
    .on_controller_data = plat_on_controller_data,
    .get_property = plat_get_property,
    .on_oob_event = plat_on_oob_event,
    .device_dump = NULL,
    .register_console_cmds = NULL,
};

struct uni_platform *uni_platform_custom_create(void)
{
    return &s_platform;
}

/* ── Hardware Init ───────────────────────────────────────── */

static void init_ledc(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = 440,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIEZO_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    ESP_LOGI(TAG, "LEDC initialized on GPIO%d", PIEZO_PIN);
}

static void init_led(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    led_off();
}

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

/* ── Startup Jingle ──────────────────────────────────────── */

static void play_startup_jingle(void)
{
    /* C5 E5 G5 C6 — major arpeggio */
    const uint16_t notes[] = {523, 659, 784, 1047};
    for (int i = 0; i < 4; i++) {
        tone_play(notes[i]);
        led_on();
        vTaskDelay(pdMS_TO_TICKS(120));
        tone_stop();
        led_off();
        vTaskDelay(pdMS_TO_TICKS(60));
    }
}

/* ── Sound Task (Core 1) ────────────────────────────────── */

static void sound_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Sound task started on core %d", xPortGetCoreID());

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        gamepad_state_t gp;
        memcpy(&gp, (const void *)&s_gp, sizeof(gp));

        if (!gp.connected) {
            tone_stop();
            led_off();
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SOUND_TASK_PERIOD_MS));
            continue;
        }

        /* Mode switch on Share button (rising edge) */
        uint8_t misc_pressed = gp.misc_buttons & ~s_prev_misc;
        s_prev_misc = gp.misc_buttons;

        if (misc_pressed & MISC_BACK) {
            tone_stop();
            s_mode = (s_mode + 1) % MODE_COUNT;
            s_arp_running = false;
            s_sfx.ticks_left = 0;
            ESP_LOGI(TAG, "Mode: %d", s_mode);
            led_blink(s_mode + 1, 80, 80);
        }

        /* Dispatch to current mode */
        switch (s_mode) {
            case MODE_THEREMIN:
                mode_theremin(&gp);
                break;
            case MODE_SCALE:
                mode_scale(&gp);
                break;
            case MODE_ARPEGGIO:
                mode_arpeggio(&gp);
                break;
            case MODE_SFX:
                mode_sfx(&gp);
                break;
            default:
                break;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SOUND_TASK_PERIOD_MS));
    }
}

/* ── Entry Point ─────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Gamepad Synth starting!");

    init_nvs();
    init_ledc();
    init_led();

    play_startup_jingle();

    ESP_LOGI(TAG, "Starting sound task on Core 1...");
    xTaskCreatePinnedToCore(sound_task, "sound", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Starting Bluepad32 on Core 0 (will not return)...");
    ESP_LOGI(TAG, "Put controller in pairing mode (Xbox: hold pair button on top)");
    uni_esp32_main();
}
