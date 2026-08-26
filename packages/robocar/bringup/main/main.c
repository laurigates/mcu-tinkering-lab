/**
 * @file main.c
 * @brief robocar-bringup: a hardware self-test for the XIAO ESP32-S3 Sense build.
 *
 * WHAT THIS IS FOR
 *
 * robocar-unified is the robot. It wants WiFi credentials, a Gemini API key and
 * a finished board, and when something is miswired the symptom arrives filtered
 * through a planner, a TLS stack and a 15-second loop. That is the wrong
 * instrument for the half-hour where wires are being soldered one at a time.
 *
 * This firmware does one thing: power up, exercise every peripheral once, and
 * say what answered. No network, no API key, no cost, no credentials, and a
 * binary small enough to flash between two solder joints.
 *
 * HOW IT REPORTS, AND WHY THAT SHAPE
 *
 * It runs the whole sweep once at boot and then idles. It is not console-driven,
 * because both serial monitors in use here — `just robocar-bringup::monitor` and
 * the `serial-monitor` helper in the dotfiles — are READ-ONLY and reset the
 * board on attach. A design that needed commands typed at it would mean putting
 * the iron down. Re-running is a tap on RESET.
 *
 * Four output channels, in the order they become available on a part-built
 * board, because the earlier ones are all you have at the start:
 *
 *   buzzer  (GPIO2, always)         — a verdict per check, by pitch
 *   serial  (USB-C, always)         — the full table, with the numbers
 *   LEDs    (needs mux + PCA9685)   — running colour, then a final verdict
 *   OLED    (needs mux + SSD1306)   — which check failed, untethered
 *
 * SKIP IS THE NORMAL RESULT. Hardware that is not fitted is not a failure —
 * see checks.h. A sweep on a bare board should sound calm and report eleven
 * SKIPs, or you learn to ignore it and it stops being worth running.
 */

#include <stdio.h>
#include <string.h>

#include "checks.h"
#include "cues.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "led_controller.h"
#include "oled.h"
#include "pin_config.h"

static const char *TAG = "bringup";

/** Rows of the OLED given to the rolling result list (0 = header, last = tally). */
#define OLED_LIST_ROWS (OLED_ROWS - 2)

/** Heartbeat once the sweep is done, so a finished board is distinguishable
 *  from a crashed one at a glance. */
#define IDLE_BLINK_PERIOD_MS 2000

static check_result_t s_results[16];

static const rgb_color_t k_color_pass = {0, 255, 0};
static const rgb_color_t k_color_warn = {255, 140, 0};
static const rgb_color_t k_color_skip = {0, 0, 60};
static const rgb_color_t k_color_fail = {255, 0, 0};

static const rgb_color_t *status_color(check_status_t s)
{
    switch (s) {
        case CHECK_PASS:
            return &k_color_pass;
        case CHECK_WARN:
            return &k_color_warn;
        case CHECK_SKIP:
            return &k_color_skip;
        case CHECK_FAIL:
        default:
            return &k_color_fail;
    }
}

static cue_t status_cue(check_status_t s)
{
    switch (s) {
        case CHECK_PASS:
            return CUE_PASS;
        case CHECK_WARN:
            return CUE_WARN;
        case CHECK_SKIP:
            return CUE_SKIP;
        case CHECK_FAIL:
        default:
            return CUE_FAIL;
    }
}

/**
 * Redraw the OLED: header, the most recent results, and a running tally.
 *
 * Shows the TAIL of the list rather than the head. Thirteen checks do not fit
 * six rows, and the interesting one is almost always the check that just ran —
 * scrolling off the top costs nothing because the serial table has the rest,
 * while scrolling off the bottom would hide the live one.
 */
static void oled_progress(size_t done, int pass, int warn, int skip, int fail)
{
    if (!oled_available()) {
        return;
    }

    char line[OLED_COLS + 1];

    oled_clear();
    oled_text(0, 0, "ROBOCAR BRINGUP");

    const size_t first = (done > OLED_LIST_ROWS) ? done - OLED_LIST_ROWS : 0;
    for (size_t i = first; i < done; i++) {
        snprintf(line, sizeof(line), "%-10s %s", g_checks[i].name,
                 check_status_label(s_results[i].status));
        oled_text(0, (uint8_t)(1 + (i - first)), line);
    }

    snprintf(line, sizeof(line), "P%d W%d S%d F%d", pass, warn, skip, fail);
    oled_text(0, OLED_ROWS - 1, line);
    oled_flush();
}

void app_main(void)
{
    const esp_app_desc_t *app = esp_app_get_description();

    printf("\n");
    printf("========================================\n");
    printf(" robocar-bringup %s\n", app ? app->version : "?");
    printf(" XIAO ESP32-S3 Sense hardware self-test\n");
    printf("========================================\n");
    printf("SKIP means 'not fitted' — that is normal on a part-built board.\n");
    printf("The motor check DRIVES THE WHEELS after three quick high beeps.\n\n");

    /* Buzzer first and unconditionally: it is the channel every result below is
     * announced on, and check_buzzer() only confirms it a moment later. */
    cues_play(CUE_SWEEP_START);

    int pass = 0, warn = 0, skip = 0, fail = 0;

    for (size_t i = 0; i < g_check_count && i < sizeof(s_results) / sizeof(s_results[0]); i++) {
        printf("[%2u/%2u] %-10s ... ", (unsigned)(i + 1), (unsigned)g_check_count,
               g_checks[i].name);
        fflush(stdout);

        s_results[i] = g_checks[i].run();

        printf("%s  %s\n", check_status_label(s_results[i].status), s_results[i].detail);
        fflush(stdout);

        switch (s_results[i].status) {
            case CHECK_PASS:
                pass++;
                break;
            case CHECK_WARN:
                warn++;
                break;
            case CHECK_SKIP:
                skip++;
                break;
            case CHECK_FAIL:
                fail++;
                break;
        }

        /* The LEDs are themselves under test, so they are only usable as an
         * indicator once their own check has run and the bus is up. */
        if (led_is_initialized()) {
            led_set_both(status_color(s_results[i].status));
        }
        oled_progress(i + 1, pass, warn, skip, fail);
        cues_play(status_cue(s_results[i].status));
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    printf("\n---------------- summary ----------------\n");
    for (size_t i = 0; i < g_check_count; i++) {
        printf("  %-10s %-4s  %s\n", g_checks[i].name, check_status_label(s_results[i].status),
               s_results[i].detail);
    }
    printf("-----------------------------------------\n");
    printf("  %d pass, %d warn, %d skip, %d fail\n", pass, warn, skip, fail);
    printf("  tap RESET to re-run\n\n");

    ESP_LOGI(TAG, "sweep complete: %d pass %d warn %d skip %d fail", pass, warn, skip, fail);

    if (oled_available()) {
        char line[OLED_COLS + 1];
        oled_clear();
        oled_text(0, 0, "BRINGUP DONE");
        snprintf(line, sizeof(line), "PASS %d  WARN %d", pass, warn);
        oled_text(0, 2, line);
        snprintf(line, sizeof(line), "SKIP %d  FAIL %d", skip, fail);
        oled_text(0, 3, line);

        /* Name the failures explicitly. A count tells you to go and read the
         * serial log; a name lets you pick the iron back up without moving. */
        uint8_t row = 5;
        for (size_t i = 0; i < g_check_count && row < OLED_ROWS; i++) {
            if (s_results[i].status == CHECK_FAIL) {
                snprintf(line, sizeof(line), "FAIL %s", g_checks[i].name);
                oled_text(0, row++, line);
            }
        }
        if (fail == 0) {
            oled_text(0, 5, row == 5 ? "NOTHING FAILED" : "");
        }
        oled_flush();
    }

    cues_play(fail > 0 ? CUE_ANY_FAIL : CUE_ALL_PASS);

    const rgb_color_t *final_color =
        (fail > 0) ? &k_color_fail : ((warn > 0) ? &k_color_warn : &k_color_pass);

    /* Idle with a slow blink rather than a solid colour or a bare `for(;;)`.
     * A solid LED and a hung board look identical; a blink says the firmware is
     * still running and the sweep is simply over. */
    for (;;) {
        if (led_is_initialized()) {
            led_set_both(final_color);
            vTaskDelay(pdMS_TO_TICKS(IDLE_BLINK_PERIOD_MS / 4));
            led_turn_off_all();
            vTaskDelay(pdMS_TO_TICKS(IDLE_BLINK_PERIOD_MS * 3 / 4));
        } else {
            vTaskDelay(pdMS_TO_TICKS(IDLE_BLINK_PERIOD_MS));
        }
    }
}
