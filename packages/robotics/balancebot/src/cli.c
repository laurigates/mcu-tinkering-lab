#include "cli.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "balance_control.h"
#include "config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "param_store.h"
#include "pico/stdlib.h"
#include "pin_config.h"
#include "telemetry.h"

#define CLI_LINE_MAX 96

static balance_params_t *mirror;
static char line[CLI_LINE_MAX];
static size_t line_len;

static const char *state_name(uint8_t s)
{
    switch ((balance_state_t)s) {
        case BAL_STATE_RUN:
            return "RUN";
        case BAL_STATE_FAULT:
            return "FAULT";
        case BAL_STATE_IDLE:
        default:
            return "IDLE";
    }
}

typedef struct {
    const char *name;
    param_id_t id;
    float *field;
} param_entry_t;

static param_entry_t param_table[PARAM_COUNT];

static void param_table_init(void)
{
    param_table[0] = (param_entry_t){"kp", PARAM_ANGLE_KP, &mirror->angle_kp};
    param_table[1] = (param_entry_t){"ki", PARAM_ANGLE_KI, &mirror->angle_ki};
    param_table[2] = (param_entry_t){"kd", PARAM_ANGLE_KD, &mirror->angle_kd};
    param_table[3] = (param_entry_t){"vkp", PARAM_VEL_KP, &mirror->vel_kp};
    param_table[4] = (param_entry_t){"vki", PARAM_VEL_KI, &mirror->vel_ki};
    param_table[5] = (param_entry_t){"alpha", PARAM_FILTER_ALPHA, &mirror->filter_alpha};
    param_table[6] = (param_entry_t){"maxrate", PARAM_MAX_RATE, &mirror->max_rate_sps};
}

static void send_param(param_id_t id, float value)
{
    balance_cmd_t cmd = {.cmd = CMD_SET_PARAM, .param_id = (uint8_t)id, .value = value};
    if (!balance_control_send_cmd(&cmd)) {
        printf("ERR: command queue full\n");
    }
}

static void send_all_params(void)
{
    for (int i = 0; i < PARAM_COUNT; i++) {
        send_param(param_table[i].id, *param_table[i].field);
    }
}

static void cmd_get(void)
{
    for (int i = 0; i < PARAM_COUNT; i++) {
        printf("%-8s %g\n", param_table[i].name, (double)*param_table[i].field);
    }
}

static void cmd_set(const char *name, const char *value_str)
{
    char *end = NULL;
    float value = strtof(value_str, &end);
    if (end == value_str || isnan(value)) {
        printf("ERR: bad value '%s'\n", value_str);
        return;
    }
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(name, param_table[i].name) == 0) {
            *param_table[i].field = value;
            send_param(param_table[i].id, value);
            printf("OK %s = %g\n", name, (double)value);
            return;
        }
    }
    printf("ERR: unknown param '%s' (see 'get')\n", name);
}

static float battery_volts(void)
{
    gpio_put(PIN_BATT_ADC_ENABLE, 1);
    sleep_us(50);
    adc_select_input(PIN_BATT_ADC - 26);  // GPIO29 = ADC3
    uint16_t raw = adc_read();
    gpio_put(PIN_BATT_ADC_ENABLE, 0);
    return (float)raw * 3.3f / 4096.0f * 2.0f;  // /2 divider on board
}

static void cmd_stat(void)
{
    balance_snapshot_t s;
    balance_control_get_snapshot(&s);
    printf("state:    %s\n", state_name(s.state));
    printf("imu:      %s\n", s.imu_ok ? "ok" : "NOT RESPONDING");
    printf("angle:    %.2f deg (setpoint %.2f)\n", (double)s.pitch_deg, (double)s.angle_setpoint);
    printf("rate:     %.0f steps/s\n", (double)s.rate_cmd_sps);
    printf("loop:     %.0f us (tick %lu)\n", (double)s.loop_us, (unsigned long)s.tick);
    printf("battery:  %.2f V\n", (double)battery_volts());
    printf("saves:    %lu (last %s)\n", (unsigned long)s.save_count,
           s.save_count == 0 ? "n/a" : (s.save_ok ? "ok" : "FAILED"));
    printf("stream:   %s\n", telemetry_is_enabled() ? "on" : "off");
}

static void cmd_help(void)
{
    printf("balancebot commands:\n");
    printf("  stat                 system status\n");
    printf("  get                  list tunable params\n");
    printf("  set <name> <value>   set param (kp ki kd vkp vki alpha maxrate)\n");
    printf("  stream on|off [hz]   CSV telemetry (default %d Hz)\n", TELEMETRY_DEFAULT_HZ);
    printf("  arm | disarm         start/stop balancing\n");
    printf("  rate <l|r> <sps>     open-loop motor test (disarmed only)\n");
    printf("  cal                  gyro bias calibration (robot still!)\n");
    printf("  save | load          persist / reload params (flash)\n");
}

static void handle_line(char *l)
{
    const char *tok = strtok(l, " \t");
    if (tok == NULL) {
        return;
    }

    if (strcmp(tok, "help") == 0) {
        cmd_help();
    } else if (strcmp(tok, "stat") == 0) {
        cmd_stat();
    } else if (strcmp(tok, "get") == 0) {
        cmd_get();
    } else if (strcmp(tok, "set") == 0) {
        const char *name = strtok(NULL, " \t");
        const char *value = strtok(NULL, " \t");
        if (name != NULL && value != NULL) {
            cmd_set(name, value);
        } else {
            printf("usage: set <name> <value>\n");
        }
    } else if (strcmp(tok, "stream") == 0) {
        const char *arg = strtok(NULL, " \t");
        const char *hz_str = strtok(NULL, " \t");
        int hz = hz_str != NULL ? atoi(hz_str) : 0;
        if (arg != NULL && strcmp(arg, "on") == 0) {
            telemetry_set_enabled(true, hz);
        } else if (arg != NULL && strcmp(arg, "off") == 0) {
            telemetry_set_enabled(false, 0);
        } else {
            printf("usage: stream on|off [hz]\n");
        }
    } else if (strcmp(tok, "arm") == 0) {
        balance_cmd_t cmd = {.cmd = CMD_ARM};
        balance_control_send_cmd(&cmd);
        printf("OK\n");
    } else if (strcmp(tok, "disarm") == 0) {
        balance_cmd_t cmd = {.cmd = CMD_DISARM};
        balance_control_send_cmd(&cmd);
        printf("OK\n");
    } else if (strcmp(tok, "rate") == 0) {
        const char *motor = strtok(NULL, " \t");
        const char *sps_str = strtok(NULL, " \t");
        if (motor == NULL || sps_str == NULL || (motor[0] != 'l' && motor[0] != 'r')) {
            printf("usage: rate <l|r> <steps_per_s>\n");
            return;
        }
        balance_cmd_t cmd = {.cmd = CMD_OPEN_LOOP_RATE,
                             .param_id = motor[0] == 'l' ? 0 : 1,
                             .value = strtof(sps_str, NULL)};
        balance_control_send_cmd(&cmd);
        printf("OK (disarmed bench mode only)\n");
    } else if (strcmp(tok, "cal") == 0) {
        balance_cmd_t cmd = {.cmd = CMD_CALIBRATE};
        balance_control_send_cmd(&cmd);
        printf("calibrating gyro — keep the robot still for ~1 s\n");
    } else if (strcmp(tok, "save") == 0) {
        balance_cmd_t cmd = {.cmd = CMD_SAVE};
        balance_control_send_cmd(&cmd);
        printf("saving (check 'stat' for result; disarmed only)\n");
    } else if (strcmp(tok, "load") == 0) {
        bool found = param_store_load(mirror);
        send_all_params();
        printf("%s\n", found ? "OK loaded from flash" : "no stored params — defaults loaded");
    } else {
        printf("ERR: unknown command '%s' — try 'help'\n", tok);
    }
}

void cli_init(balance_params_t *params)
{
    mirror = params;
    line_len = 0;
    param_table_init();

    adc_init();
    adc_gpio_init(PIN_BATT_ADC);
    gpio_init(PIN_BATT_ADC_ENABLE);
    gpio_put(PIN_BATT_ADC_ENABLE, 0);
    gpio_set_dir(PIN_BATT_ADC_ENABLE, GPIO_OUT);
}

void cli_poll(void)
{
    for (;;) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) {
            return;
        }
        if (c == '\r' || c == '\n') {
            if (line_len > 0) {
                line[line_len] = '\0';
                handle_line(line);
                line_len = 0;
            }
        } else if (c == 0x7F || c == '\b') {
            if (line_len > 0) {
                line_len--;
            }
        } else if (line_len < CLI_LINE_MAX - 1 && c >= 0x20 && c < 0x7F) {
            line[line_len++] = (char)c;
        }
    }
}
