// Balancebot — self-balancing robot on a Seeed XIAO RP2350.
// Core 0 (this file): USB CDC CLI, telemetry, status LED.
// Core 1: hard-real-time balancing loop (see balance_control.c).
#include <stdio.h>

#include "balance_control.h"
#include "config.h"
#include "hardware/gpio.h"
#include "param_store.h"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pin_config.h"
#include "status_led.h"
#include "telemetry.h"

#include "cli.h"

#ifndef BALANCEBOT_VERSION
#define BALANCEBOT_VERSION "dev"
#endif

// Core 0's mirror of the tunable parameters (CLI edits it; core 1 holds the
// live copy and receives updates via the command queue).
static balance_params_t params;

int main(void)
{
    // Motors stay disabled from the first instruction: drive nENABLE high
    // before anything else (the external pull-up covers us until here).
    gpio_init(PIN_MOTOR_NENABLE);
    gpio_put(PIN_MOTOR_NENABLE, 1);
    gpio_set_dir(PIN_MOTOR_NENABLE, GPIO_OUT);

    stdio_init_all();
    status_led_init();

    param_store_load(&params);

    balance_control_init(&params);
    cli_init(&params);
    telemetry_init();

    // Core 1 saves params to flash (CMD_SAVE); this core must cooperate with
    // flash_safe_execute so XIP is quiesced during erase/program.
    flash_safe_execute_core_init();

    multicore_launch_core1(balance_control_core1_main);

    printf("\nbalancebot %s — XIAO RP2350\n", BALANCEBOT_VERSION);
    printf("type 'help' for commands\n");

    while (true) {
        cli_poll();
        telemetry_poll();

        balance_snapshot_t s;
        balance_control_get_snapshot(&s);
        status_led_poll((balance_state_t)s.state);

        sleep_ms(1);
    }
}
