#include "status_led.h"

#include "config.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pin_config.h"
#include "status_color.h"
#include "ws2812.pio.h"

#define RGB_PIO pio1
#define RGB_UPDATE_MS 20u  // the colour changes at most this often

static int rgb_sm = -1;
static rgb_t rgb_last = {1, 1, 1};  // differs from every colour sent
static uint32_t rgb_last_ms;

void status_led_init(void)
{
    gpio_init(PIN_STATUS_LED);
    gpio_set_dir(PIN_STATUS_LED, GPIO_OUT);
    gpio_put(PIN_STATUS_LED, 1);  // active low: off

    // The RGB LED's 5 V supply is switched by a load switch on NEO_PWR.
    gpio_init(PIN_RGB_POWER);
    gpio_set_dir(PIN_RGB_POWER, GPIO_OUT);
    gpio_put(PIN_RGB_POWER, 1);
    rgb_sm = pio_claim_unused_sm(RGB_PIO, false);
    if (rgb_sm >= 0 && pio_can_add_program(RGB_PIO, &ws2812_program)) {
        uint offset = pio_add_program(RGB_PIO, &ws2812_program);
        ws2812_program_init(RGB_PIO, (uint)rgb_sm, offset, PIN_RGB_DATA, 800000.0f);
    } else {
        rgb_sm = -1;  // no PIO room: the yellow LED still works
    }
}

static void rgb_send(rgb_t c)
{
    // WS2812 order is GRB, MSB first; the state machine shifts 24 bits out
    // of the top of each word.
    const uint32_t grb = ((uint32_t)c.g << 16) | ((uint32_t)c.r << 8) | c.b;
    pio_sm_put_blocking(RGB_PIO, (uint)rgb_sm, grb << 8u);
}

void status_led_poll(const balance_snapshot_t *s)
{
    const uint32_t ms = to_ms_since_boot(get_absolute_time());
    bool on;
    switch ((balance_state_t)s->state) {
        case BAL_STATE_RUN:
            on = true;
            break;
        case BAL_STATE_FAULT:
            on = (ms / 100) % 2 == 0;  // 5 Hz
            break;
        case BAL_STATE_IDLE:
        default:
            on = (ms / 500) % 2 == 0;  // 1 Hz
            break;
    }
    gpio_put(PIN_STATUS_LED, on ? 0 : 1);

    if (rgb_sm < 0 || ms - rgb_last_ms < RGB_UPDATE_MS) {
        return;
    }
    rgb_last_ms = ms;
    const status_inputs_t in = {
        .state = s->state,
        .starting = s->tick == 0u,
        .imu_ok = s->imu_ok,
        .pitch_deg = s->pitch_deg,
        .arm_holdoff_ticks = s->arm_holdoff_ticks,
        .gesture_ticks = s->gesture_ticks,
        .gesture_needed = (uint32_t)(ARM_GESTURE_S * CONTROL_LOOP_HZ),
        .arm_window_deg = ARM_GESTURE_DEG,
        .ms = ms,
    };
    const rgb_t c = status_color(&in);
    if (c.r != rgb_last.r || c.g != rgb_last.g || c.b != rgb_last.b) {
        rgb_send(c);
        rgb_last = c;
    }
}
