#include "stepper.h"

#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "pin_config.h"
#include "stepper.pio.h"

#define STEPPER_PIO pio0
#define STEP_TICK_HZ 1000000.0f  // SM clock from stepper_program_init()

typedef struct {
    uint sm;
    uint step_pin;
    uint dir_pin;
    bool dir_invert;     // wired mirror-image: true for one side
    float current_rate;  // signed steps/s actually commanded to the SM
    bool running;        // SM emitting pulses
} motor_t;

static motor_t motors[STEPPER_COUNT] = {
    {.sm = 0, .step_pin = PIN_STEP_LEFT, .dir_pin = PIN_DIR_LEFT, .dir_invert = false},
    {.sm = 1, .step_pin = PIN_STEP_RIGHT, .dir_pin = PIN_DIR_RIGHT, .dir_invert = true},
};

static uint program_offset;
static bool enabled;

static void motor_park(motor_t *m)
{
    if (!m->running) {
        return;
    }
    pio_sm_set_enabled(STEPPER_PIO, m->sm, false);
    pio_sm_clear_fifos(STEPPER_PIO, m->sm);
    // Force STEP low and restart from the program top so the next start
    // begins with a clean pull.
    pio_sm_exec(STEPPER_PIO, m->sm, pio_encode_set(pio_pins, 0));
    pio_sm_restart(STEPPER_PIO, m->sm);
    pio_sm_exec(STEPPER_PIO, m->sm, pio_encode_jmp(program_offset));
    m->running = false;
}

static uint32_t rate_to_count(float rate_abs)
{
    float period_ticks = STEP_TICK_HZ / rate_abs;
    if (period_ticks < (float)(STEPPER_PIO_OVERHEAD_TICKS + 1)) {
        period_ticks = (float)(STEPPER_PIO_OVERHEAD_TICKS + 1);
    }
    return (uint32_t)(period_ticks - (float)STEPPER_PIO_OVERHEAD_TICKS);
}

static void motor_apply(motor_t *m, float rate)
{
    if (fabsf(rate) < MIN_RATE_SPS) {
        motor_park(m);
        m->current_rate = 0.0f;
        return;
    }

    bool reverse = rate < 0.0f;
    bool was_reverse = m->current_rate < 0.0f;
    if (m->running && reverse != was_reverse) {
        // Direction changes only through the parked state; the reversal
        // completes on the next control tick.
        motor_park(m);
        m->current_rate = 0.0f;
        return;
    }

    gpio_put(m->dir_pin, reverse ^ m->dir_invert);

    uint32_t count = rate_to_count(fabsf(rate));
    if (!m->running) {
        // DIR setup time (200 ns) is satisfied by the instructions between
        // the gpio_put above and the first pulse.
        pio_sm_put(STEPPER_PIO, m->sm, count);
        pio_sm_set_enabled(STEPPER_PIO, m->sm, true);
        m->running = true;
    } else if (pio_sm_get_tx_fifo_level(STEPPER_PIO, m->sm) == 0) {
        // Keep at most one pending period queued so a pushed rate is stale by
        // at most one step period; between pushes the SM re-uses the last
        // value (pull noblock).
        pio_sm_put(STEPPER_PIO, m->sm, count);
    }
    m->current_rate = rate;
}

void stepper_init(void)
{
    // Drivers disabled before anything else touches the pins. The external
    // pull-up already holds nENABLE high while the GPIO is hi-Z.
    gpio_init(PIN_MOTOR_NENABLE);
    gpio_put(PIN_MOTOR_NENABLE, 1);
    gpio_set_dir(PIN_MOTOR_NENABLE, GPIO_OUT);
    enabled = false;

    program_offset = pio_add_program(STEPPER_PIO, &stepper_program);

    for (int i = 0; i < STEPPER_COUNT; i++) {
        motor_t *m = &motors[i];
        pio_sm_claim(STEPPER_PIO, m->sm);
        gpio_init(m->dir_pin);
        gpio_put(m->dir_pin, 0);
        gpio_set_dir(m->dir_pin, GPIO_OUT);
        stepper_program_init(STEPPER_PIO, m->sm, program_offset, m->step_pin);
        m->current_rate = 0.0f;
        m->running = false;
    }
}

void stepper_enable(bool en)
{
    if (!en) {
        for (int i = 0; i < STEPPER_COUNT; i++) {
            motor_park(&motors[i]);
            motors[i].current_rate = 0.0f;
        }
    }
    gpio_put(PIN_MOTOR_NENABLE, en ? 0 : 1);
    enabled = en;
}

bool stepper_is_enabled(void)
{
    return enabled;
}

void stepper_set_rate(stepper_id_t id, float steps_per_s, float max_rate, float dt)
{
    motor_t *m = &motors[id];

    if (steps_per_s > max_rate) {
        steps_per_s = max_rate;
    }
    if (steps_per_s < -max_rate) {
        steps_per_s = -max_rate;
    }

    // Slew-limit towards the request regardless of what the PID asks for.
    float max_delta = MAX_ACCEL_SPS2 * dt;
    float delta = steps_per_s - m->current_rate;
    if (delta > max_delta) {
        steps_per_s = m->current_rate + max_delta;
    } else if (delta < -max_delta) {
        steps_per_s = m->current_rate - max_delta;
    }

    if (!enabled) {
        m->current_rate = 0.0f;
        return;
    }
    motor_apply(m, steps_per_s);
}

float stepper_get_rate(stepper_id_t id)
{
    return motors[id].current_rate;
}
