#include "pack.h"

#include "balance_control.h"
#include "config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pin_config.h"

static const balance_params_t *mirror;
static pack_reading_t reading;
static pack_monitor_t monitor;
static absolute_time_t next_due;
static absolute_time_t last_poll;
static bool sent_low;  // what core 1 was last told

void pack_init(const balance_params_t *params)
{
    mirror = params;
    adc_gpio_init(PIN_PACK_SENSE);
    pack_monitor_init(&monitor);
    reading = (pack_reading_t){.present = false, .clipped = false, .volts = 0.0f};
    sent_low = false;
    last_poll = get_absolute_time();
    next_due = last_poll;
}

static pack_reading_t sample(void)
{
    uint16_t raw[PACK_SAMPLES];
    adc_select_input(PIN_PACK_SENSE - 26);  // GPIO26 = ADC0
    for (unsigned i = 0; i < PACK_SAMPLES; i++) {
        raw[i] = adc_read();
    }
    return pack_reading_from_counts(raw, PACK_SAMPLES, mirror->pack_r_low_kohm);
}

void pack_poll(void)
{
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(now, next_due) > 0) {
        return;
    }
    next_due = make_timeout_time_ms(PACK_POLL_MS);
    // The real elapsed time, so a stall (a CLI command, a log save) does not
    // shorten the cutoff's minimum duration.
    const float dt_s = (float)absolute_time_diff_us(last_poll, now) * 1e-6f;
    last_poll = now;

    reading = sample();
    pack_monitor_update(&monitor, &reading, mirror->pack_cells, dt_s);

    // Send on change; a full queue leaves sent_low as it was, so the next poll
    // tries again.
    if (monitor.fault != sent_low) {
        balance_cmd_t c = {.cmd = CMD_SET_PACK_LOW, .value = monitor.fault ? 1.0f : 0.0f};
        if (balance_control_send_cmd(&c)) {
            sent_low = monitor.fault;
        }
    }
}

pack_reading_t pack_get_reading(void)
{
    return reading;
}

pack_level_t pack_get_level(void)
{
    return monitor.level;
}

uint16_t pack_get_centivolts(void)
{
    return pack_reading_centivolts(&reading);
}
