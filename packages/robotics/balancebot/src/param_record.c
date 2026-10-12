#include "param_record.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "crc32.h"

_Static_assert(sizeof(balance_params_t) == 16u * 4u, "balance_params_t v2 layout changed");
_Static_assert(offsetof(balance_params_t, ctrl_mode) == PARAM_V1_FLOATS * 4u,
               "the v1 fields must stay a prefix of balance_params_t");

static uint32_t rd32(const uint8_t *p)
{
    uint32_t v;
    memcpy(&v, p, sizeof v);
    return v;
}

void param_defaults(balance_params_t *out)
{
    out->angle_kp = DEFAULT_ANGLE_KP;
    out->angle_ki = DEFAULT_ANGLE_KI;
    out->angle_kd = DEFAULT_ANGLE_KD;
    out->vel_kp = DEFAULT_VEL_KP;
    out->vel_ki = DEFAULT_VEL_KI;
    out->filter_alpha = DEFAULT_FILTER_ALPHA;
    out->max_rate_sps = DEFAULT_MAX_RATE_SPS;
    out->ctrl_mode = CTRL_PID;
    out->l_eff_m = DEFAULT_L_EFF_M;
    out->wheel_diam_m = DEFAULT_WHEEL_DIAM_M;
    out->steps_per_rev = DEFAULT_STEPS_PER_REV;
    out->q_theta = DEFAULT_Q_THETA;
    out->q_dtheta = DEFAULT_Q_DTHETA;
    out->q_x = DEFAULT_Q_X;
    out->q_dx = DEFAULT_Q_DX;
    out->r = DEFAULT_R;
}

size_t param_record_encode(const balance_params_t *p, uint8_t *buf, size_t cap)
{
    if (cap < PARAM_V2_SIZE) {
        return 0;
    }
    const uint32_t magic = PARAM_MAGIC;
    const uint32_t version = PARAM_VERSION;
    memcpy(buf, &magic, 4);
    memcpy(buf + 4, &version, 4);
    memcpy(buf + 8, p, sizeof *p);
    const uint32_t crc = crc32_calc(buf, PARAM_V2_SIZE - 4u);
    memcpy(buf + PARAM_V2_SIZE - 4u, &crc, 4);
    return PARAM_V2_SIZE;
}

static bool positive(float v)
{
    return isfinite(v) && v > 0.0f;
}

// Replace any LQR-side field the solver or the unit conversion would reject
// with its default. A ctrl_mode outside the enum reads as PID (fail closed).
static void sanitize_lqr_fields(balance_params_t *p)
{
    balance_params_t d;
    param_defaults(&d);
    if (p->ctrl_mode != CTRL_PID && p->ctrl_mode != CTRL_LQR) {
        p->ctrl_mode = CTRL_PID;
    }
    if (!positive(p->l_eff_m)) {
        p->l_eff_m = d.l_eff_m;
    }
    if (!positive(p->wheel_diam_m)) {
        p->wheel_diam_m = d.wheel_diam_m;
    }
    if (!positive(p->steps_per_rev)) {
        p->steps_per_rev = d.steps_per_rev;
    }
    float *q[] = {&p->q_theta, &p->q_dtheta, &p->q_x, &p->q_dx};
    const float qd[] = {d.q_theta, d.q_dtheta, d.q_x, d.q_dx};
    for (int i = 0; i < 4; i++) {
        if (!isfinite(*q[i]) || *q[i] < 0.0f) {
            *q[i] = qd[i];
        }
    }
    if (!positive(p->r)) {
        p->r = d.r;
    }
}

static bool crc_ok(const uint8_t *buf, size_t size)
{
    return rd32(buf + size - 4u) == crc32_calc(buf, size - 4u);
}

bool param_record_decode(const uint8_t *buf, size_t len, balance_params_t *out, uint32_t *version)
{
    if (len < 8u || rd32(buf) != PARAM_MAGIC) {
        return false;
    }
    const uint32_t ver = rd32(buf + 4);
    balance_params_t p;
    if (ver == 1u) {
        if (len < PARAM_V1_SIZE || !crc_ok(buf, PARAM_V1_SIZE)) {
            return false;
        }
        // v1 -> v2: keep the saved PID-side fields, default everything new.
        param_defaults(&p);
        memcpy(&p, buf + 8, PARAM_V1_FLOATS * 4u);
    } else if (ver == 2u) {
        if (len < PARAM_V2_SIZE || !crc_ok(buf, PARAM_V2_SIZE)) {
            return false;
        }
        memcpy(&p, buf + 8, sizeof p);
        sanitize_lqr_fields(&p);
    } else {
        return false;
    }
    *out = p;
    if (version != NULL) {
        *version = ver;
    }
    return true;
}
