/**
 * @file checks.c
 * @brief The bringup checks. See checks.h for the three rules every one obeys.
 *
 * ORDER IS LOAD-BEARING, and not merely cosmetic:
 *
 *   - `buzzer` runs third, before anything that can fail interestingly, because
 *     it is the channel every later result is reported on. A sweep whose
 *     indicator is broken reports nothing and looks like a dead board.
 *   - `i2c-mux` gates everything behind the multiplexer. If it does not answer,
 *     six later checks SKIP rather than each rediscovering the same fault and
 *     sounding six failures for one missing wire.
 *   - `oled` runs early, straight after the scan, so the remaining checks can
 *     be displayed live on it while they run.
 *   - `mic` runs last, after `amp`. The two sit centimetres apart, so measuring
 *     the room while the amplifier is still sounding measures the amplifier.
 */

#include "checks.h"

#include <inttypes.h>
#include <stdarg.h> /* result() is variadic */
#include <stdio.h>
#include <stdlib.h> /* qsort, for the ultrasonic median */
#include <string.h>

#include "audio_player.h"
#include "buzzer.h"
#include "cues.h"
#include "esp_err.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_expander.h"
#include "i2c_bus.h"
#include "i2cdev.h"
#include "led_controller.h"
#include "mic_pdm.h"
#include "motor_controller.h"
#include "oled.h"
#include "pin_config.h"
#include "servo_controller.h"
#include "ultrasonic.h"

static const char *TAG = "checks";

/** Ultrasonic readings taken per sweep. Odd, so the median needs no averaging. */
#define SONAR_SAMPLES 5

/** Per-motor pulse. Short and slow on purpose — see check_motors(). */
#define MOTOR_PULSE_MS 220
#define MOTOR_PULSE_SPEED 70 /* of 255 */

/** Microphone samples measured, and the floor below which the mic reads dead. */
#define MIC_SAMPLES 2048
#define MIC_SILENT_PEAK 4 /* LSB; below this nothing is arriving at all */

const char *check_status_label(check_status_t status)
{
    switch (status) {
        case CHECK_PASS:
            return "PASS";
        case CHECK_WARN:
            return "WARN";
        case CHECK_SKIP:
            return "SKIP";
        case CHECK_FAIL:
        default:
            return "FAIL";
    }
}

/** Build a result without every check repeating the snprintf dance. */
static check_result_t result(check_status_t status, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

static check_result_t result(check_status_t status, const char *fmt, ...)
{
    check_result_t r = {.status = status, .detail = {0}};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(r.detail, sizeof(r.detail), fmt, ap);
    va_end(ap);
    return r;
}

/* -------------------------------------------------------------------------- */
/* Silicon                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * PSRAM. Reported rather than merely asserted because the failure this catches
 * is a boot loop, not a wrong number: CONFIG_SPIRAM_MODE_OCT is mandatory on the
 * XIAO ESP32-S3 Sense (octal PSRAM) and a quad-mode build never reaches this
 * code at all. Zero here therefore means PSRAM was configured off, not that the
 * chip is faulty — and 512 kB of it is the audio ring, so the amp check below
 * cannot run without it.
 */
static check_result_t check_psram(void)
{
    const size_t total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const size_t freeb = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    if (total == 0) {
        return result(CHECK_FAIL, "no psram - check SPIRAM_MODE_OCT");
    }
    return result(CHECK_PASS, "%uk total %uk free", (unsigned)(total / 1024),
                  (unsigned)(freeb / 1024));
}

/** Flash size and internal heap. Cheap, and it dates every other reading. */
static check_result_t check_flash(void)
{
    uint32_t size = 0;
    if (esp_flash_get_size(NULL, &size) != ESP_OK) {
        return result(CHECK_FAIL, "flash size read failed");
    }
    const size_t iram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    return result(CHECK_PASS, "%uM flash %uk int heap", (unsigned)(size / (1024 * 1024)),
                  (unsigned)(iram / 1024));
}

/* -------------------------------------------------------------------------- */
/* Buzzer — the reporting channel itself                                        */
/* -------------------------------------------------------------------------- */

/**
 * There is no way to sense whether a piezo actually sounded, so this cannot
 * report PASS honestly on the evidence available — it reports that the driver
 * came up and tones were emitted, and the listener supplies the rest. Claiming
 * PASS for an unverifiable thing is the same error as rendering an unread
 * sensor as a measurement.
 */
static check_result_t check_buzzer(void)
{
    if (buzzer_init() != ESP_OK) {
        return result(CHECK_FAIL, "buzzer_init failed");
    }
    buzzer_play_tone(1047, 90);
    vTaskDelay(pdMS_TO_TICKS(60));
    buzzer_play_tone(1568, 90);
    return result(CHECK_PASS, "2 tones sent - audible?");
}

/* -------------------------------------------------------------------------- */
/* I2C                                                                          */
/* -------------------------------------------------------------------------- */

/**
 * The multiplexer, and the gate for six later checks.
 *
 * i2c_bus_init() also brings up the PCA9685 on channel 0, so a failure here is
 * ambiguous between the two parts — which is why the scan below runs next and
 * says which addresses actually answered.
 *
 * Note this project is wired with a PCA9548A where the code says TCA9548A. They
 * are register-identical (one control byte, one bit per channel, base address
 * 0x70) and the tca9548 driver drives either unchanged. The one board-level
 * difference worth knowing is that RESET is active-low on both and must be
 * pulled high; not every breakout populates that pull-up, and a floating RESET
 * shows up as intermittent channel-select failures rather than a clean absence.
 */
static check_result_t check_i2c_mux(void)
{
    if (i2c_bus_is_ready()) {
        return result(CHECK_PASS, "already up at 0x%02X", TCA9548A_ADDR);
    }

    const esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        return result(CHECK_SKIP, "no mux at 0x%02X (%s)", TCA9548A_ADDR, esp_err_to_name(err));
    }
    return result(CHECK_PASS, "mux 0x%02X + pca9685 0x%02X", TCA9548A_ADDR, PCA9685_ADDR);
}

/**
 * Walk every multiplexer channel and list what answers.
 *
 * This is the single most useful line in the sweep while a soldering iron is
 * hot: it says what the bus can see right now, per channel, without needing to
 * know what was supposed to be there.
 *
 * 0x70 is skipped deliberately. The multiplexer lives on the PRIMARY bus, so it
 * ACKs on every channel and would appear eight times — and the PCA9685's
 * ALLCALL address is also 0x70 by default, so the hit is doubly ambiguous.
 * Reporting it per channel would be eight lines of noise hiding the one device
 * that matters.
 */
static check_result_t check_i2c_scan(void)
{
    if (!i2c_bus_is_ready()) {
        return result(CHECK_SKIP, "bus down");
    }

    char found[CHECK_DETAIL_MAX];
    size_t used = 0;
    int total = 0;
    found[0] = '\0';

    for (uint8_t ch = 0; ch < 8; ch++) {
        if (i2c_bus_select_channel(ch) != ESP_OK) {
            continue;
        }
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            if (addr == TCA9548A_ADDR) {
                continue;
            }
            i2c_dev_t dev = {0};
            dev.port = I2C_NUM_0;
            dev.addr = addr;
            dev.cfg.sda_io_num = I2C_SDA_PIN;
            dev.cfg.scl_io_num = I2C_SCL_PIN;
            dev.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;

            if (i2c_dev_probe(&dev, I2C_DEV_WRITE) != ESP_OK) {
                continue;
            }
            total++;
            ESP_LOGI(TAG, "  ch%u: 0x%02X", (unsigned)ch, (unsigned)addr);
            const int n = snprintf(found + used, sizeof(found) - used, "%s%u:%02X", used ? " " : "",
                                   (unsigned)ch, (unsigned)addr);
            if (n > 0 && (size_t)n < sizeof(found) - used) {
                used += (size_t)n;
            }
        }
        i2c_bus_release();
    }

    if (total == 0) {
        return result(CHECK_WARN, "mux ok but no devices behind it");
    }
    return result(CHECK_PASS, "%d dev %s", total, found);
}

/* -------------------------------------------------------------------------- */
/* Devices behind the multiplexer                                               */
/* -------------------------------------------------------------------------- */

static check_result_t check_oled(void)
{
    if (!i2c_bus_is_ready()) {
        return result(CHECK_SKIP, "bus down");
    }

    const esp_err_t err = oled_init();
    if (err == ESP_ERR_NOT_FOUND) {
        return result(CHECK_SKIP, "no panel on ch%d", I2C_BUS_CHANNEL_OLED);
    }
    if (err != ESP_OK) {
        return result(CHECK_FAIL, "init failed: %s", esp_err_to_name(err));
    }
    return result(CHECK_PASS, "ssd1306 0x%02X ch%d", OLED_I2C_ADDR, I2C_BUS_CHANNEL_OLED);
}

/**
 * Both RGB LEDs through the PCA9685. Also the second visual channel the sweep
 * reports on, so like the buzzer it runs before the checks that use it.
 */
static check_result_t check_leds(void)
{
    if (!i2c_bus_is_ready()) {
        return result(CHECK_SKIP, "bus down");
    }
    if (led_controller_init() != ESP_OK) {
        return result(CHECK_FAIL, "led_controller_init failed");
    }

    static const rgb_color_t seq[] = {
        {255, 0, 0},
        {0, 255, 0},
        {0, 0, 255},
        {255, 255, 255},
    };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        if (led_set_both(&seq[i]) != ESP_OK) {
            led_turn_off_all();
            return result(CHECK_FAIL, "write failed at step %u", (unsigned)i);
        }
        vTaskDelay(pdMS_TO_TICKS(180));
    }
    led_turn_off_all();
    return result(CHECK_PASS, "r/g/b/white both - correct?");
}

/** Pan and tilt through a small excursion, then centre. */
static check_result_t check_servos(void)
{
    if (!i2c_bus_is_ready()) {
        return result(CHECK_SKIP, "bus down");
    }
    if (servo_controller_init() != ESP_OK) {
        return result(CHECK_FAIL, "servo_controller_init failed");
    }

    /* Small excursions: a servo that is not fitted costs nothing, and one that
     * is fitted into a half-built chassis should not slam into it. */
    const struct {
        int16_t pan;
        int16_t tilt;
    } poses[] = {{0, 0}, {-30, 0}, {30, 0}, {0, -20}, {0, 20}, {0, 0}};

    for (size_t i = 0; i < sizeof(poses) / sizeof(poses[0]); i++) {
        if (servo_set_position(poses[i].pan, poses[i].tilt) != ESP_OK) {
            return result(CHECK_FAIL, "write failed at pose %u", (unsigned)i);
        }
        vTaskDelay(pdMS_TO_TICKS(280));
    }

    /* Released rather than left holding: a stalled SG90 draws hundreds of mA
     * continuously, and the amplifier check further down shares this rail. */
    servo_disable_all();
    return result(CHECK_PASS, "pan +-30 tilt +-20 - moved?");
}

/**
 * Pulse each motor briefly in each direction.
 *
 * DELIBERATELY BRIEF AND SLOW. This is the only check with a physical
 * consequence: a robot with wheels on, sitting near the edge of a bench, will
 * drive off it. CUE_MOTOR_ARMED sounds first so the wheels never move without
 * warning, and the four pulses are short enough to identify direction and long
 * enough to hear a stalled gearbox.
 *
 * Each pulse also names which motor and which direction in the log, because
 * "the motors ran" is not the question — the question is whether left is left
 * and forward is forward, and that is only answerable if you know which pulse
 * you are watching.
 */
static check_result_t check_motors(void)
{
    if (!i2c_bus_is_ready()) {
        return result(CHECK_SKIP, "bus down");
    }
    if (motor_controller_init() != ESP_OK) {
        return result(CHECK_FAIL, "motor_controller_init failed");
    }

    cues_play(CUE_MOTOR_ARMED);
    vTaskDelay(pdMS_TO_TICKS(400));

    const struct {
        const char *what;
        uint8_t left;
        uint8_t right;
        uint8_t ldir;
        uint8_t rdir;
    } pulses[] = {
        {"left fwd", MOTOR_PULSE_SPEED, 0, 1, 1},
        {"left rev", MOTOR_PULSE_SPEED, 0, 0, 1},
        {"right fwd", 0, MOTOR_PULSE_SPEED, 1, 1},
        {"right rev", 0, MOTOR_PULSE_SPEED, 1, 0},
    };

    for (size_t i = 0; i < sizeof(pulses) / sizeof(pulses[0]); i++) {
        ESP_LOGI(TAG, "  motor pulse: %s", pulses[i].what);
        const esp_err_t err =
            motor_set_individual(pulses[i].left, pulses[i].right, pulses[i].ldir, pulses[i].rdir);
        if (err != ESP_OK) {
            motor_stop();
            return result(CHECK_FAIL, "%s write failed", pulses[i].what);
        }
        vTaskDelay(pdMS_TO_TICKS(MOTOR_PULSE_MS));
        motor_stop();
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    return result(CHECK_PASS, "4 pulses - see log for order");
}

static check_result_t check_mcp23017(void)
{
    if (!i2c_bus_is_ready()) {
        return result(CHECK_SKIP, "bus down");
    }
    if (gpio_expander_init() != ESP_OK || !gpio_expander_available()) {
        return result(CHECK_SKIP, "no mcp23017 on ch%d", I2C_BUS_CHANNEL_MCP23017);
    }

    /* Drive pin 0 both ways and read the latch back. This proves the I2C path
     * and the chip's registers, NOT the pin's external wiring — nothing is
     * connected to it, so there is nothing further to verify from here. */
    if (gpio_expander_set_mode(0, GPIO_EXPANDER_OUTPUT) != ESP_OK) {
        return result(CHECK_FAIL, "set_mode failed");
    }
    for (int level = 0; level <= 1; level++) {
        bool read_back = false;
        if (gpio_expander_write(0, level != 0) != ESP_OK ||
            gpio_expander_read(0, &read_back) != ESP_OK) {
            return result(CHECK_FAIL, "pin0 io failed");
        }
        if (read_back != (level != 0)) {
            return result(CHECK_FAIL, "pin0 wrote %d read %d", level, (int)read_back);
        }
    }
    gpio_expander_set_mode(0, GPIO_EXPANDER_INPUT);
    return result(CHECK_PASS, "0x%02X pin0 loopback ok", MCP23017_ADDR);
}

/* -------------------------------------------------------------------------- */
/* Direct-GPIO peripherals                                                      */
/* -------------------------------------------------------------------------- */

static int cmp_u16(const void *a, const void *b)
{
    const uint16_t x = *(const uint16_t *)a;
    const uint16_t y = *(const uint16_t *)b;
    return (x > y) - (x < y);
}

/**
 * Take several readings and report the median plus the success count.
 *
 * A single reading proves almost nothing here: an unfitted sensor times out,
 * and a fitted one aimed at open space ALSO times out, so one failed read is
 * not evidence of a fault. The count separates "nothing is connected" (0 of 5)
 * from "connected but out of range" and from a flaky solder joint (3 of 5),
 * which look identical from any one measurement.
 */
static check_result_t check_ultrasonic(void)
{
    if (ultrasonic_init() != ESP_OK) {
        return result(CHECK_FAIL, "ultrasonic_init failed");
    }

    uint16_t good[SONAR_SAMPLES];
    size_t n = 0;

    for (int i = 0; i < SONAR_SAMPLES; i++) {
        uint16_t cm = 0;
        if (ultrasonic_measure(&cm) == ESP_OK && cm != ULTRASONIC_DIST_ERROR) {
            good[n++] = cm;
        }
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    if (n == 0) {
        return result(CHECK_SKIP, "0/%d reads - unfitted or open space", SONAR_SAMPLES);
    }
    qsort(good, n, sizeof(good[0]), cmp_u16);
    const uint16_t median = good[n / 2];

    if (n < SONAR_SAMPLES) {
        return result(CHECK_WARN, "%u/%d reads med %ucm", (unsigned)n, SONAR_SAMPLES,
                      (unsigned)median);
    }
    return result(CHECK_PASS, "%u/%d reads med %ucm", (unsigned)n, SONAR_SAMPLES, (unsigned)median);
}

/**
 * The MAX98357A, driven with a locally generated tone.
 *
 * No network, no API key, no cost — and it is the control the TTS path has
 * never had. See cues_amp_tone_sequence() for why a synthesised sine
 * discriminates a streaming fault from an amplifier or supply fault.
 */
static check_result_t check_amp(void)
{
    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM) < AUDIO_RING_BYTES) {
        return result(CHECK_SKIP, "no psram for the %uk ring", (unsigned)(AUDIO_RING_BYTES / 1024));
    }
    if (audio_player_init() != ESP_OK) {
        return result(CHECK_FAIL, "audio_player_init failed");
    }

    const esp_err_t err = cues_amp_tone_sequence();
    if (err != ESP_OK) {
        return result(CHECK_FAIL, "tone failed: %s", esp_err_to_name(err));
    }
    return result(CHECK_PASS, "440/880/sweep - clean?");
}

/**
 * The onboard PDM microphone.
 *
 * Reports peak and RMS in raw LSB rather than a verdict, because the two ways
 * this fails are indistinguishable from a pass/fail alone: a dead or unseated
 * Sense board delivers no frames, while a working microphone in a silent room
 * delivers frames of near-zero samples. The first is a fault and the second is
 * a Tuesday. Frames-but-no-signal is therefore WARN, not FAIL — clap and re-run
 * to settle it.
 */
static check_result_t check_mic(void)
{
    if (mic_pdm_init() != ESP_OK || !mic_pdm_is_ready()) {
        return result(CHECK_SKIP, "no pdm mic - sense board seated?");
    }
    if (mic_pdm_lock(1000) != ESP_OK) {
        return result(CHECK_FAIL, "mic busy");
    }

    static int16_t buf[MIC_SAMPLES];
    mic_pdm_flush();

    size_t got = 0;
    const esp_err_t err = mic_pdm_read(buf, MIC_SAMPLES, &got, 1000);
    mic_pdm_unlock();

    if (err != ESP_OK) {
        return result(CHECK_FAIL, "read failed: %s", esp_err_to_name(err));
    }
    if (got == 0) {
        return result(CHECK_FAIL, "0 samples - mic is deaf");
    }

    int32_t peak = 0;
    uint64_t sumsq = 0;
    for (size_t i = 0; i < got; i++) {
        const int32_t v = buf[i];
        const int32_t a = (v < 0) ? -v : v;
        if (a > peak) {
            peak = a;
        }
        sumsq += (uint64_t)((int64_t)v * v);
    }
    const uint32_t rms = (uint32_t)(sumsq / got);

    if (peak < MIC_SILENT_PEAK) {
        return result(CHECK_WARN, "%u smp peak %d - silent?", (unsigned)got, (int)peak);
    }
    return result(CHECK_PASS, "%u smp peak %d rms2 %u", (unsigned)got, (int)peak, (unsigned)rms);
}

/* -------------------------------------------------------------------------- */
/* The sweep                                                                    */
/* -------------------------------------------------------------------------- */

const check_t g_checks[] = {
    {"psram", check_psram},       {"flash", check_flash},       {"buzzer", check_buzzer},
    {"i2c-mux", check_i2c_mux},   {"i2c-scan", check_i2c_scan}, {"oled", check_oled},
    {"leds", check_leds},         {"servos", check_servos},     {"motors", check_motors},
    {"mcp23017", check_mcp23017}, {"sonar", check_ultrasonic},  {"amp", check_amp},
    {"mic", check_mic},
};

const size_t g_check_count = sizeof(g_checks) / sizeof(g_checks[0]);
