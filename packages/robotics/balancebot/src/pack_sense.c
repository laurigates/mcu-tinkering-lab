#include "pack_sense.h"

#include <math.h>

bool pack_r_low_valid(float r_low_kohm)
{
    return isfinite(r_low_kohm) && r_low_kohm >= 1.0f && r_low_kohm <= 1000.0f;
}

bool pack_cells_valid(float cells)
{
    return isfinite(cells) && cells >= 2.0f && cells <= 6.0f && cells == floorf(cells);
}

float pack_scale(float r_low_kohm)
{
    return (PACK_R_TOP_KOHM + r_low_kohm) / r_low_kohm;
}

static float counts_to_pin_volts(float counts)
{
    return counts * PACK_ADC_VREF_V / PACK_ADC_COUNTS;
}

pack_reading_t pack_reading_from_counts(const uint16_t *raw, unsigned n, float r_low_kohm)
{
    pack_reading_t out = {.present = false, .clipped = false, .volts = 0.0f};
    if (n == 0u) {
        return out;
    }
    uint32_t sum = 0;
    uint16_t lo = raw[0];
    uint16_t hi = raw[0];
    for (unsigned i = 0; i < n; i++) {
        sum += raw[i];
        if (raw[i] < lo) {
            lo = raw[i];
        }
        if (raw[i] > hi) {
            hi = raw[i];
        }
    }
    const float mean = (float)sum / (float)n;
    const float pin_v = counts_to_pin_volts(mean);
    if (pin_v < PACK_ABSENT_PIN_V || counts_to_pin_volts((float)(hi - lo)) > PACK_NOISE_PIN_V) {
        return out;
    }
    out.present = true;
    out.clipped = mean >= (float)PACK_CLIP_COUNT;
    out.volts = pin_v * pack_scale(r_low_kohm);
    return out;
}

uint16_t pack_reading_centivolts(const pack_reading_t *r)
{
    if (!r->present) {
        return 0u;
    }
    const float cv = roundf(r->volts * 100.0f);
    if (cv >= 65535.0f) {
        return 65535u;
    }
    return cv > 0.0f ? (uint16_t)cv : 0u;
}

float pack_warn_volts(float cells)
{
    return cells * PACK_WARN_V_PER_CELL;
}

float pack_fault_volts(float cells)
{
    return cells * PACK_FAULT_V_PER_CELL;
}

void pack_monitor_init(pack_monitor_t *m)
{
    m->level = PACK_ABSENT;
    m->fault = false;
    m->warned = false;
    m->below_s = 0.0f;
}

void pack_monitor_update(pack_monitor_t *m, const pack_reading_t *r, float cells, float dt_s)
{
    if (!r->present) {
        pack_monitor_init(m);
        return;
    }
    const float v = r->volts;
    const float fault_v = pack_fault_volts(cells);

    if (m->fault) {
        if (v >= fault_v + PACK_FAULT_HYST_V_PER_CELL * cells) {
            m->fault = false;
            m->below_s = 0.0f;
        }
    } else if (v < fault_v) {
        m->below_s += dt_s;
        if (m->below_s >= PACK_FAULT_MIN_S) {
            m->fault = true;
        }
    } else {
        m->below_s = 0.0f;
    }

    if (v < pack_warn_volts(cells)) {
        m->warned = true;
    } else if (v >= pack_warn_volts(cells) + PACK_WARN_HYST_V_PER_CELL * cells) {
        m->warned = false;
    }

    m->level = m->fault ? PACK_LOW : (m->warned ? PACK_WARN : PACK_OK);
}
