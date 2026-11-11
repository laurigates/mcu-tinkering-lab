#include "cli.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "balance_control.h"
#include "cli_gate.h"
#include "config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "lqr.h"
#include "param_store.h"
#include "pico/stdlib.h"
#include "pin_config.h"
#include "telemetry.h"

#define CLI_LINE_MAX 96
// Longest core 0 waits for core 1 to drain what it was sent: a few ticks at the
// 2 ms nominal period, or the 3 ms fallback with no IMU interrupt.
#define CLI_CORE1_SYNC_TIMEOUT_US 20000

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

// The first snapshot tick that reflects every command sent through note_sent().
static uint32_t core1_sync_tick;
static bool core1_sync_pending;

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

static const char *mount_name(uint32_t m)
{
    return m == IMU_MOUNT_DOWN ? "down" : "up";
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

static balance_cmd_t mount_cmd(uint32_t mount)
{
    return (balance_cmd_t){.cmd = CMD_SET_IMU_MOUNT, .imu_mount = (uint8_t)mount};
}

// Call right after queueing a command whose effect a later check reads from
// the snapshot. Core 1 drains its queue at the start of a tick and publishes
// at the end, so with tick T published after the add, the command is drained
// no later than the tick that publishes T + 2.
static void note_sent(void)
{
    balance_snapshot_t s;
    balance_control_get_snapshot(&s);
    core1_sync_tick = s.tick + 2u;
    core1_sync_pending = true;
}

// Fill *s with a snapshot that reflects every command note_sent() covered,
// waiting a few ticks at most. Returns false, with the latest snapshot in *s,
// if core 1 did not get there within CLI_CORE1_SYNC_TIMEOUT_US.
static bool wait_core1(balance_snapshot_t *s)
{
    const absolute_time_t deadline = make_timeout_time_us(CLI_CORE1_SYNC_TIMEOUT_US);
    for (;;) {
        balance_control_get_snapshot(s);
        if (!core1_sync_pending || (int32_t)(s->tick - core1_sync_tick) >= 0) {
            core1_sync_pending = false;
            return true;
        }
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;
        }
        sleep_us(100);
    }
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

    balance_cmd_t cmds[PARAM_COUNT + 3];
    unsigned n = 0;
    if (include_params) {
        for (int i = 0; i < PARAM_COUNT; i++) {
            cmds[n++] = param_cmd(&param_table[i], *field_of(mirror, &param_table[i]));
        }
        // Core 1 ignores it while armed; `stat` shows the mounting it applies.
        cmds[n++] = mount_cmd(mirror->imu_mount);
    }
    if (lqr_status == LQR_OK) {
        cmds[n++] = gains_cmd(&lqr_res);
    }
    cmds[n++] = ctrl_cmd(mirror->ctrl_mode);
    if (!balance_control_send_cmds(cmds, n)) {
        printf("ERR: command queue full — nothing sent, retry\n");
        return false;
    }
    note_sent();  // may carry a mount change
    lqr_have_k = (lqr_status == LQR_OK);
    return true;
}

static void cmd_get(void)
{
    for (int i = 0; i < PARAM_COUNT; i++) {
        printf("%-14s %g\n", param_table[i].name, (double)*field_of(mirror, &param_table[i]));
    }
    printf("%-14s %s\n", "ctrl", ctrl_name(mirror->ctrl_mode));
    printf("%-14s %s\n", "mount", mount_name(mirror->imu_mount));
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

// After a CMD_SET_IMU_MOUNT was sent (`mount`, or `load`'s burst): wait for
// core 1 to drain it and give the mirror the mount core 1 applies, so `get`
// and a later `save` never disagree with `stat`. Core 1 drops the mount if the
// hold-upright gesture armed it first. Returns true when the mount applied.
static bool adopt_mount(uint32_t requested)
{
    balance_snapshot_t s;
    const bool synced = wait_core1(&s);
    switch (cli_mount_outcome(synced, s.imu_mount, requested)) {
        case CLI_MOUNT_APPLIED:
            mirror->imu_mount = s.imu_mount;
            return true;
        case CLI_MOUNT_DROPPED:
            mirror->imu_mount = s.imu_mount;
            printf("ERR: core 1 armed (state %s) before the mount arrived — mount stays %s; "
                   "'disarm' and try again\n",
                   state_name(s.state), mount_name(s.imu_mount));
            return false;
        case CLI_MOUNT_NO_ANSWER:
        default:
            // Still queued: core 1 applies it unless it arms first.
            mirror->imu_mount = requested;
            printf("sent mount %s, but core 1 has not answered — check 'stat'\n",
                   mount_name(requested));
            return false;
    }
}

// GY-521 orientation: `up` = component side up (X forward, Z up), `down` =
// component side down, turned 180 degrees about the axle (the delivered
// 8d18cd8d carrier). Disarmed only: the pitch would jump 180 degrees. The
// state is judged after earlier commands (a pasted `disarm`) have reached
// core 1, as `arm` does.
static void cmd_mount(const char *arg)
{
    balance_snapshot_t s;
    const bool synced = wait_core1(&s);
    if (arg == NULL) {
        printf("mount: %s\n", mount_name(s.imu_mount));
        return;
    }
    uint32_t mount;
    if (strcmp(arg, "up") == 0) {
        mount = IMU_MOUNT_UP;
    } else if (strcmp(arg, "down") == 0) {
        mount = IMU_MOUNT_DOWN;
    } else {
        printf("usage: mount [up|down]\n");
        return;
    }
    switch (cli_gate_mount(synced, s.state)) {
        case CLI_GATE_SEND:
            break;
        case CLI_GATE_BUSY:
            printf("ERR: core 1 busy — earlier commands not applied yet, mount still %s; "
                   "mount again\n",
                   mount_name(s.imu_mount));
            return;
        default:
            printf("ERR: disarm first — mount still %s\n", mount_name(s.imu_mount));
            return;
    }
    balance_cmd_t c = mount_cmd(mount);
    if (!balance_control_send_cmd(&c)) {
        printf("ERR: command queue full — mount still %s\n", mount_name(mirror->imu_mount));
        return;
    }
    note_sent();
    if (adopt_mount(mount)) {
        printf("OK mount = %s ('save' to keep)\n", mount_name(mount));
    }
}

static void print_upside_down(const char *prefix, float up_g, uint32_t mount)
{
    printf("ERR: %sIMU reads upside down (body Z = %.2f g); check 'mount' (now %s)\n", prefix,
           (double)up_g, mount_name(mount));
}

// At rest, body Z below -0.5 g is a wrong `mount`, not a lean: the controller
// would push the wrong way. The check runs on a snapshot taken after any
// `mount` just sent (a paste, a script) has reached core 1, so it uses the
// mount core 1 applies. Core 1 checks again under its mount when CMD_ARM
// runs; a refusal there shows in arm_refusals and is reported here. If core 1
// has not caught up (a `save` before it stalls core 1 past the wait), nothing
// is judged on the stale snapshot and nothing is sent. Core 1 arms only from
// IDLE, so any other state is refused here rather than reported as OK. Core 1
// arms in its command drain and runs the RUN branch in the same tick, so a
// robot lying past the tilt cutoff (or a silent IMU) shows FAULT in the first
// snapshot after the arm; that is reported instead of OK.
static void cmd_arm(void)
{
    balance_snapshot_t s;
    const bool synced = wait_core1(&s);
    switch (cli_gate_arm(synced, s.state, s.imu_ok, &s.gravity)) {
        case CLI_GATE_SEND:
            break;
        case CLI_GATE_BUSY:
            printf("ERR: core 1 busy — earlier commands (a mount?) not applied yet, arm not sent; "
                   "arm again\n");
            return;
        case CLI_GATE_UPSIDE_DOWN:
            print_upside_down("", s.gravity.up_g, s.imu_mount);
            return;
        case CLI_GATE_FAULT_NO_IMU:
            // Core 1 clears FAULT by a hold only while the IMU reads.
            printf("ERR: state FAULT, IMU not responding — only 'disarm' clears it; "
                   "'stat' shows when the IMU reads again\n");
            return;
        default:
            if (s.state == BAL_STATE_FAULT) {
                // Tilt cutoff, or an IMU read that failed and has recovered:
                // either way the hold clears it now the IMU reads.
                printf("ERR: state FAULT — 'disarm' to clear it, or hold upright within %.0f deg "
                       "for %.0f s; then arm\n",
                       (double)FAULT_CLEAR_DEG, (double)FAULT_CLEAR_S);
            } else {
                printf("ERR: state %s — already armed; 'disarm' to stop\n", state_name(s.state));
            }
            return;
    }
    const uint32_t refusals = s.arm_refusals;
    balance_cmd_t cmd = {.cmd = CMD_ARM};
    if (!balance_control_send_cmd(&cmd)) {
        printf("ERR: command queue full — arm not sent\n");
        return;
    }
    note_sent();
    const bool answered = wait_core1(&s);
    switch (cli_arm_outcome(answered, refusals, s.arm_refusals, s.state, s.imu_ok)) {
        case CLI_ARM_OK:
            printf("OK\n");
            return;
        case CLI_ARM_NO_ANSWER:
            printf("OK sent, but core 1 has not answered — check 'stat'\n");
            return;
        case CLI_ARM_REFUSED:
            print_upside_down("core 1 refused arm: ", s.arm_refused_up_g, s.imu_mount);
            return;
        case CLI_ARM_FAULTED_TILT:
            printf("ERR: armed, then FAULT (pitch %.1f deg, tilt cutoff %.0f deg) — stand it "
                   "upright, 'disarm', then arm\n",
                   (double)s.pitch_deg, (double)TILT_CUTOFF_DEG);
            return;
        case CLI_ARM_FAULTED_NO_IMU:
            printf("ERR: armed, then FAULT — IMU not responding; only 'disarm' clears it\n");
            return;
        case CLI_ARM_NOT_ARMED:
        default:
            printf("ERR: core 1 did not arm (state %s) — check 'stat'\n", state_name(s.state));
            return;
    }
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
    printf("imu:      %s (mount %s)\n", s.imu_ok ? "ok" : "NOT RESPONDING",
           mount_name(s.imu_mount));
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
    if (s.arm_refusals != 0) {
        printf("arm:      %lu refused by core 1, IMU upside down (last body Z = %.2f g)\n",
               (unsigned long)s.arm_refusals, (double)s.arm_refused_up_g);
    }
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
    printf("  mount [up|down]      show / set GY-521 side: up, or down = flipped about\n");
    printf("                       the axle (8d18cd8d carrier); disarmed only\n");
    printf("  stream on|off [hz]   CSV telemetry (default %d Hz)\n", TELEMETRY_DEFAULT_HZ);
    printf("  arm | disarm         start/stop balancing\n");
    printf("  rate <l|r> <sps>     open-loop motor test (disarmed only)\n");
    printf("  cal                  gyro bias calibration (robot still!)\n");
    printf("  save | load          persist / reload params (flash; disarmed only)\n");
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
    } else if (strcmp(tok, "mount") == 0) {
        cmd_mount(strtok(NULL, " \t"));
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
        cmd_arm();
    } else if (strcmp(tok, "disarm") == 0) {
        balance_cmd_t cmd = {.cmd = CMD_DISARM};
        balance_control_send_cmd(&cmd);
        note_sent();  // a pasted 'arm' or 'load' after it checks the state it leaves
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
        // Core 1 applies parameters, gains and controller in RUN but ignores
        // the mount there, so a load while armed would leave the mirror (and
        // `get`) on a mount core 1 is not applying. Disarmed only, like save.
        balance_snapshot_t s;
        const cli_gate_t gate = cli_gate_load(wait_core1(&s), s.state);
        if (gate == CLI_GATE_BUSY) {
            printf("ERR: core 1 busy — earlier commands not applied yet, nothing loaded; "
                   "load again\n");
            return;
        }
        if (gate != CLI_GATE_SEND) {
            printf("ERR: disarm first — load could change the mount, which core 1 keeps while "
                   "armed; nothing loaded\n");
            return;
        }
        bool found = param_store_load(mirror);
        boot_note[0] = '\0';
        // The solve in sync_all() takes a few ms; if the hold-upright gesture
        // arms core 1 before the burst drains, core 1 drops the mount.
        // adopt_mount() reports that and keeps the mirror on what core 1
        // applies.
        if (sync_all(true) && adopt_mount(mirror->imu_mount)) {
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
