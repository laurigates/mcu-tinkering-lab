/**
 * @file motor_hold.c
 * @brief BOOT-button motor hold for DMM measurements. See motor_hold.h.
 */

#include "motor_hold.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "cues.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "motor_controller.h"
#include "pin_config.h"

/** The XIAO ESP32-S3's BOOT button. Unused by pin_config.h; active low. */
#define MOTOR_HOLD_BUTTON_PIN GPIO_NUM_0

/** A held state drops back to off after this long. Enough for a round of DMM
 *  readings; short enough that a robot driving off a bench stops on its own. */
#define MOTOR_HOLD_TIMEOUT_MS 60000

/** Consecutive low polls that count as a press. At a 20 ms poll, 40 ms. */
#define MOTOR_HOLD_DEBOUNCE_POLLS 2

/** Stopped time between two held states, so a reversal never lands at full
 *  duty on a motor still spinning the other way. */
#define MOTOR_HOLD_GAP_MS 250

typedef struct {
    const char *name;
    bool left;    /**< true = left motor (TB6612 B side), false = right (A side). */
    bool forward; /**< motor_controller direction 1: IN1 high, IN2 low. */
} hold_state_t;

/** Index 0 is off; the rest are driven in order, one per BOOT press. */
static const hold_state_t k_states[] = {
    {"off", false, false},      {"left fwd", true, true},    {"left rev", true, false},
    {"right fwd", false, true}, {"right rev", false, false},
};
#define HOLD_STATE_COUNT (sizeof(k_states) / sizeof(k_states[0]))

static size_t s_state;
static int64_t s_held_since_us;
static unsigned s_low_polls;
static bool s_press_latched;
static bool s_available;

/** Print the voltage each node of the driven motor's path should read. The
 *  channel numbers come from pin_config.h, never literals, so a renumbering
 *  cannot leave this table pointing at the old pins. */
static void print_expectations(const hold_state_t *s)
{
    const char side = s->left ? 'B' : 'A';
    const int in1 = s->left ? MOTOR_LEFT_IN1_CHANNEL : MOTOR_RIGHT_IN1_CHANNEL;
    const int in2 = s->left ? MOTOR_LEFT_IN2_CHANNEL : MOTOR_RIGHT_IN2_CHANNEL;
    const int pwm = s->left ? MOTOR_LEFT_PWM_CHANNEL : MOTOR_RIGHT_PWM_CHANNEL;

    printf("\nHOLD %s @ 100%% — BOOT for next, auto-off in %d s\n", s->name,
           MOTOR_HOLD_TIMEOUT_MS / 1000);
    printf("  measure against GND:\n");
    printf("    STBY          <- XIAO GPIO%d      3.3 V\n", (int)MOTOR_STBY_PIN);
    printf("    %cIN1          <- PCA9685 ch%-2d    %s\n", side, in1, s->forward ? "3.3 V" : "0 V");
    printf("    %cIN2          <- PCA9685 ch%-2d    %s\n", side, in2, s->forward ? "0 V" : "3.3 V");
    printf("    PWM%c          <- PCA9685 ch%-2d    ~3.3 V (100%% duty)\n", side, pwm);
    printf("    VCC / VM                         3.3 V / ~5 V under load\n");
    printf("  across the motor:\n");
    printf("    %cO1 - %cO2                        %c~VM\n", side, side, s->forward ? '+' : '-');
}

static void enter_state(size_t next)
{
    /* Always pass through a stop: off is a stop, and every drive state starts
     * from one so the other motor is never left running. */
    motor_stop();
    s_state = next;

    if (next == 0) {
        printf("\nHOLD off — motors stopped. BOOT to start again.\n");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(MOTOR_HOLD_GAP_MS));
    cues_play(CUE_MOTOR_ARMED);
    vTaskDelay(pdMS_TO_TICKS(400));

    const hold_state_t *s = &k_states[next];
    const uint8_t full = 255;
    /* The idle motor gets speed 0 with IN1 high: the TB6612's short brake. */
    const esp_err_t err = s->left ? motor_set_individual(full, 0, s->forward, 1)
                                  : motor_set_individual(0, full, 1, s->forward);
    if (err != ESP_OK) {
        motor_stop();
        s_state = 0;
        printf("\nHOLD %s: PCA9685 write failed (%s) — motors stopped\n", s->name,
               esp_err_to_name(err));
        return;
    }

    s_held_since_us = esp_timer_get_time();
    print_expectations(s);
}

void motor_hold_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << MOTOR_HOLD_BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io) != ESP_OK) {
        printf("  motor hold: BOOT button unavailable\n\n");
        return;
    }

    /* Starts latched, so a button still held from the reset is not a press. */
    s_press_latched = true;

    if (!i2c_bus_is_ready()) {
        printf("  motor hold: unavailable, I2C bus down (PCA9685 not answering)\n\n");
        return;
    }
    if (motor_controller_init() != ESP_OK) {
        printf("  motor hold: unavailable, motor_controller_init failed\n\n");
        return;
    }

    s_available = true;
    printf("  motor hold: press BOOT to drive one motor at 100%% and keep it there\n");
    printf("              (left fwd, left rev, right fwd, right rev, off)\n\n");
}

void motor_hold_poll(void)
{
    if (!s_available) {
        return;
    }

    if (s_state != 0 &&
        esp_timer_get_time() - s_held_since_us >= (int64_t)MOTOR_HOLD_TIMEOUT_MS * 1000) {
        printf("\nHOLD %s timed out after %d s\n", k_states[s_state].name,
               MOTOR_HOLD_TIMEOUT_MS / 1000);
        enter_state(0);
    }

    if (gpio_get_level(MOTOR_HOLD_BUTTON_PIN) == 0) {
        if (s_low_polls < MOTOR_HOLD_DEBOUNCE_POLLS) {
            s_low_polls++;
        }
    } else {
        s_low_polls = 0;
        s_press_latched = false;
    }

    if (s_low_polls >= MOTOR_HOLD_DEBOUNCE_POLLS && !s_press_latched) {
        s_press_latched = true;
        enter_state((s_state + 1) % HOLD_STATE_COUNT);
    }
}
