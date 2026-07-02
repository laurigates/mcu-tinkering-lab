#include "status_led.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pin_config.h"

void status_led_init(void)
{
    gpio_init(PIN_STATUS_LED);
    gpio_set_dir(PIN_STATUS_LED, GPIO_OUT);
    gpio_put(PIN_STATUS_LED, 1);  // active low: off
}

void status_led_poll(balance_state_t state)
{
    uint32_t ms = to_ms_since_boot(get_absolute_time());
    bool on;
    switch (state) {
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
}
