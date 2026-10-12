#include "cli.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "balance_control.h"
#include "config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "lqr.h"
#include "param_store.h"
#include "pico/stdlib.h"
#include "pin_config.h"
#include "telemetry.h"

#define CLI_LINE_MAX 96

static balance_params_t *mirror;
static char line[CLI_LINE_MAX];
static size_t line_len;

// LQR solve state. Core 0 only: lqr_solve() is double math with ~2 KB of
// stack and runs here (boot, `set` of a model/weight param, `load`), never on
// core 1. Core 1 only ever receives the finished K.
static lqr_result_t lqr_res;
static int lqr_status = LQR_ERR_INVALID;  // status of the last solve attempt
static bool lqr_have_k;                   // a K matching *mirror was handed to core 1
static char boot_note[112];               // printed after the banner and in `stat`

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

static const char *ctrl_name(uint32_t m)
{
    return m == CTRL_LQR ? "lqr" : "pid";
}

static const char *lqr_status_name(int st)
{
    switch ((lqr_status_t)st) {
        case LQR_OK:
            return "ok";
        case LQR_ERR_INVALID:
            return "invalid model or weights";
        case LQR_ERR_SINGULAR:
            return "singular matrix";
        case LQR_ERR_NO_CONVERGE:
            return "did not converge";
        case LQR_ERR_NON_FINITE:
            return "non-finite result";
        case LQR_ERR_UNSTABLE:
            return "closed loop not stable (rho >= 1; is q_x 0?)";
        default:
            return "unknown error";
    }
}

typedef enum {
    RANGE_ANY,       // PID-side: forwarded as typed (existing behaviour)
    RANGE_POSITIVE,  // > 0, finite
    RANGE_NONNEG,    // >= 0, finite
} range_t;

typedef struct {
    const char *name;
    param_id_t id;
    size_t offset;  // of the float field within balance_params_t
    range_t range;
    bool lqr;  // a change re-solves K before it is applied
} param_entry_t;

#define PENTRY(n, id, field, range, lqr) {n, id, offsetof(balance_params_t, field), range, lqr}

static const param_entry_t param_table[PARAM_COUNT] = {
    PENTRY("kp", PARAM_ANGLE_KP, angle_kp, RANGE_ANY, false),
    PENTRY("ki", PARAM_ANGLE_KI, angle_ki, RANGE_ANY, false),
    PENTRY("kd", PARAM_ANGLE_KD, angle_kd, RANGE_ANY, false),
    PENTRY("vkp", PARAM_VEL_KP, vel_kp, RANGE_ANY, false),
    PENTRY("vki", PARAM_VEL_KI, vel_ki, RANGE_ANY, false),
    PENTRY("alpha", PARAM_FILTER_ALPHA, filter_alpha, RANGE_ANY, false),
    PENTRY("maxrate", PARAM_MAX_RATE, max_rate_sps, RANGE_ANY, false),
    PENTRY("l_eff_m", PARAM_L_EFF, l_eff_m, RANGE_POSITIVE, true),
    PENTRY("wheel_diam_m", PARAM_WHEEL_DIAM, wheel_diam_m, RANGE_POSITIVE, true),
    PENTRY("steps_per_rev", PARAM_STEPS_PER_REV, steps_per_rev, RANGE_POSITIVE, true),
    PENTRY("q_theta", PARAM_Q_THETA, q_theta, RANGE_NONNEG, true),
    PENTRY("q_dtheta", PARAM_Q_DTHETA, q_dtheta, RANGE_NONNEG, true),
    PENTRY("q_x", PARAM_Q_X, q_x, RANGE_NONNEG, true),
    PENTRY("q_dx", PARAM_Q_DX, q_dx, RANGE_NONNEG, true),
    PENTRY("r", PARAM_R, r, RANGE_POSITIVE, true),
};

static float *field_of(balance_params_t *p, const param_entry_t *e)
{
    return (float *)((unsigned char *)p + e->offset);
}

static bool in_range(const param_entry_t *e, float v)
{
    switch (e->range) {
        case RANGE_POSITIVE:
            return isfinite(v) && v > 0.0f;
        case RANGE_NONNEG:
            return isfinite(v) && v >= 0.0f;
        case RANGE_ANY:
        default:
            return true;
    }
}

static int solve_for(const balance_params_t *p, lqr_result_t *out)
{
    const lqr_model_t model = {.l_eff_m = p->l_eff_m, .g = LQR_G_MPS2};
    const lqr_weights_t weights = {
        .q = {p->q_theta, p->q_dtheta, p->q_x, p->q_dx},
        .r = p->r,
    };
    return lqr_solve(&model, &weights, 1.0 / CONTROL_LOOP_HZ, out);
}

static balance_cmd_t gains_cmd(const lqr_result_t *r)
{
    balance_cmd_t c = {.cmd = CMD_SET_LQR_GAINS};
    for (int i = 0; i < LQR_N; i++) {
        c.lqr_K[i] = r->K[i];
    }
    return c;
}

static balance_cmd_t param_cmd(const param_entry_t *e, float value)
{
    return (balance_cmd_t){.cmd = CMD_SET_PARAM, .param_id = (uint8_t)e->id, .value = value};
}

static balance_cmd_t ctrl_cmd(uint32_t mode)
{
    return (balance_cmd_t){.cmd = CMD_SET_CONTROLLER, .ctrl_mode = (uint8_t)mode};
}

static void print_lqr(void)
{
    if (!lqr_have_k) {
        printf("lqr:      no valid K (last solve: %s)\n", lqr_status_name(lqr_status));
        return;
    }
    printf("lqr:      K = [%.5g %.5g %.5g %.5g]  rho %.6f  residual %.2e  (%d iter)\n",
           (double)lqr_res.K[0], (double)lqr_res.K[1], (double)lqr_res.K[2], (double)lqr_res.K[3],
           lqr_res.rho, lqr_res.residual, lqr_res.iterations);
}

// Solve for *mirror and push every parameter, the gains and the controller to
// core 1 in one all-or-nothing burst. A saved LQR mode whose solve fails
// falls back to PID, and the reason is kept in boot_note.
static bool sync_all(bool include_params)
{
    lqr_result_t r;
    lqr_status = solve_for(mirror, &r);
    lqr_have_k = false;
    if (lqr_status == LQR_OK) {
        lqr_res = r;
    } else if (mirror->ctrl_mode == CTRL_LQR) {
        mirror->ctrl_mode = CTRL_PID;
        snprintf(boot_note, sizeof boot_note,
                 "saved ctrl lqr, but the LQR solve failed (%s): running PID",
                 lqr_status_name(lqr_status));
    }

    balance_cmd_t cmds[PARAM_COUNT + 2];
    unsigned n = 0;
    if (include_params) {
        for (int i = 0; i < PARAM_COUNT; i++) {
            cmds[n++] = param_cmd(&param_table[i], *field_of(mirror, &param_table[i]));
        }
    }
    if (lqr_status == LQR_OK) {
        cmds[n++] = gains_cmd(&lqr_res);
    }
    cmds[n++] = ctrl_cmd(mirror->ctrl_mode);
    if (!balance_control_send_cmds(cmds, n)) {
        printf("ERR: command queue full — nothing sent, retry\n");
        return false;
    }
    lqr_have_k = (lqr_status == LQR_OK);
    return true;
}

static void cmd_get(void)
{
    for (int i = 0; i < PARAM_COUNT; i++) {
        printf("%-14s %g\n", param_table[i].name, (double)*field_of(mirror, &param_table[i]));
    }
    printf("%-14s %s\n", "ctrl", ctrl_name(mirror->ctrl_mode));
}

static void cmd_set(const char *name, const char *value_str)
{
    char *end = NULL;
    float value = strtof(value_str, &end);
    if (end == value_str || isnan(value)) {
        printf("ERR: bad value '%s'\n", value_str);
        return;
    }
    const param_entry_t *e = NULL;
    for (int i = 0; i < PARAM_COUNT; i++) {
        if (strcmp(name, param_table[i].name) == 0) {
            e = &param_table[i];
        }
    }
    if (e == NULL) {
        printf("ERR: unknown param '%s' (see 'get')\n", name);
        return;
    }
    if (!in_range(e, value)) {
        printf("ERR: %s must be %s\n", name, e->range == RANGE_NONNEG ? ">= 0" : "> 0");
        return;
    }
    if (!e->lqr) {
        balance_cmd_t c = param_cmd(e, value);
        if (!balance_control_send_cmd(&c)) {
            printf("ERR: command queue full — %s unchanged\n", name);
            return;
        }
        *field_of(mirror, e) = value;
        printf("OK %s = %g\n", name, (double)value);
        return;
    }

    // Model / weight change: solve first, apply only if the solve succeeds,
    // and hand core 1 the new K and the parameter in one burst.
    balance_params_t cand = *mirror;
    *field_of(&cand, e) = value;
    lqr_result_t r;
    int st = solve_for(&cand, &r);
    if (st != LQR_OK) {
        printf("ERR: LQR solve failed (%s) — %s unchanged\n", lqr_status_name(st), name);
        return;
    }
    const balance_cmd_t cmds[2] = {gains_cmd(&r), param_cmd(e, value)};
    if (!balance_control_send_cmds(cmds, 2)) {
        printf("ERR: command queue full — %s unchanged\n", name);
        return;
    }
    *mirror = cand;
    lqr_res = r;
    lqr_status = LQR_OK;
    lqr_have_k = true;
    printf("OK %s = %g\n", name, (double)value);
    print_lqr();
}

static void cmd_ctrl(const char *arg)
{
    if (arg == NULL) {
        balance_snapshot_t s;
        balance_control_get_snapshot(&s);
        printf("ctrl: %s\n", ctrl_name(s.ctrl_mode));
        return;
    }
    uint32_t mode;
    if (strcmp(arg, "pid") == 0) {
        mode = CTRL_PID;
    } else if (strcmp(arg, "lqr") == 0) {
        mode = CTRL_LQR;
        if (!lqr_have_k) {
            printf("ERR: lqr refused — no valid K (last solve: %s); still on %s\n",
                   lqr_status_name(lqr_status), ctrl_name(mirror->ctrl_mode));
            return;
        }
    } else {
        printf("usage: ctrl [pid|lqr]\n");
        return;
    }
    balance_cmd_t c = ctrl_cmd(mode);
    if (!balance_control_send_cmd(&c)) {
        printf("ERR: command queue full — still on %s\n", ctrl_name(mirror->ctrl_mode));
        return;
    }
    mirror->ctrl_mode = mode;
    printf("OK ctrl = %s (switches at the next tick; 'save' to keep)\n", ctrl_name(mode));
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
    printf("ctrl:     %s\n", ctrl_name(s.ctrl_mode));
    printf("imu:      %s\n", s.imu_ok ? "ok" : "NOT RESPONDING");
    printf("angle:    %.2f deg (setpoint %.2f)\n", (double)s.pitch_deg, (double)s.angle_setpoint);
    printf("rate:     %.0f steps/s\n", (double)s.rate_cmd_sps);
    printf("pos:      %.3f m since arm\n", (double)s.pos_m);
    if (s.lqr_k_valid) {
        printf("K core1:  [%.5g %.5g %.5g %.5g]\n", (double)s.lqr_K[0], (double)s.lqr_K[1],
               (double)s.lqr_K[2], (double)s.lqr_K[3]);
    } else {
        printf("K core1:  none\n");
    }
    print_lqr();
    printf("loop:     %.0f us (tick %lu)\n", (double)s.loop_us, (unsigned long)s.tick);
    printf("battery:  %.2f V\n", (double)battery_volts());
    printf("saves:    %lu (last %s)\n", (unsigned long)s.save_count,
           s.save_count == 0 ? "n/a" : (s.save_ok ? "ok" : "FAILED"));
    printf("stream:   %s\n", telemetry_is_enabled() ? "on" : "off");
    if (boot_note[0] != '\0') {
        printf("note:     %s\n", boot_note);
    }
}

static void cmd_help(void)
{
    printf("balancebot commands:\n");
    printf("  stat                 system status\n");
    printf("  get                  list tunable params\n");
    printf("  set <name> <value>   set param: kp ki kd vkp vki alpha maxrate\n");
    printf("                       LQR (re-solves K): l_eff_m wheel_diam_m steps_per_rev\n");
    printf("                       q_theta q_dtheta q_x q_dx r\n");
    printf("  ctrl [pid|lqr]       show / switch the balance law (live, bumpless)\n");
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
    } else if (strcmp(tok, "ctrl") == 0) {
        cmd_ctrl(strtok(NULL, " \t"));
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
        boot_note[0] = '\0';
        if (sync_all(true)) {
            printf("%s\n", found ? "OK loaded from flash" : "no stored params — defaults loaded");
        }
        if (boot_note[0] != '\0') {
            printf("%s\n", boot_note);
        }
        print_lqr();
    } else {
        printf("ERR: unknown command '%s' — try 'help'\n", tok);
    }
}

void cli_init(balance_params_t *params)
{
    mirror = params;
    line_len = 0;
    boot_note[0] = '\0';

    // Boot solve. balance_control_init() already holds the parameters and
    // starts core 1 on the PID; this hands over K and, if the saved mode is
    // LQR and the solve succeeded, the switch. Runs before core 1 launches.
    sync_all(false);

    adc_init();
    adc_gpio_init(PIN_BATT_ADC);
    gpio_init(PIN_BATT_ADC_ENABLE);
    gpio_put(PIN_BATT_ADC_ENABLE, 0);
    gpio_set_dir(PIN_BATT_ADC_ENABLE, GPIO_OUT);
}

void cli_print_boot_notes(void)
{
    printf("ctrl: %s\n", ctrl_name(mirror->ctrl_mode));
    if (boot_note[0] != '\0') {
        printf("%s\n", boot_note);
    }
    print_lqr();
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
