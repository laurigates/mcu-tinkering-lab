#include "param_store.h"

#include <stddef.h>
#include <string.h>

#include "config.h"
#include "crc32.h"
#include "hardware/flash.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

#define PARAM_MAGIC 0x42414C42u  // "BALB"
#define PARAM_VERSION 1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    balance_params_t params;
    uint32_t crc;  // over magic..params
} param_record_t;

_Static_assert(sizeof(param_record_t) <= FLASH_PAGE_SIZE, "record must fit one flash page");

void param_store_defaults(balance_params_t *out)
{
    out->angle_kp = DEFAULT_ANGLE_KP;
    out->angle_ki = DEFAULT_ANGLE_KI;
    out->angle_kd = DEFAULT_ANGLE_KD;
    out->vel_kp = DEFAULT_VEL_KP;
    out->vel_ki = DEFAULT_VEL_KI;
    out->filter_alpha = DEFAULT_FILTER_ALPHA;
    out->max_rate_sps = DEFAULT_MAX_RATE_SPS;
}

bool param_store_load(balance_params_t *out)
{
    const param_record_t *rec = (const param_record_t *)(XIP_BASE + PARAM_FLASH_OFFSET);
    if (rec->magic == PARAM_MAGIC && rec->version == PARAM_VERSION &&
        rec->crc == crc32_calc(rec, offsetof(param_record_t, crc))) {
        *out = rec->params;
        return true;
    }
    param_store_defaults(out);
    return false;
}

static void do_flash_write(void *param)
{
    const uint8_t *page = (const uint8_t *)param;
    flash_range_erase(PARAM_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(PARAM_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
}

bool param_store_save(const balance_params_t *params)
{
    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);

    param_record_t rec;
    rec.magic = PARAM_MAGIC;
    rec.version = PARAM_VERSION;
    rec.params = *params;
    rec.crc = crc32_calc(&rec, offsetof(param_record_t, crc));
    memcpy(page, &rec, sizeof rec);

    if (flash_safe_execute(do_flash_write, page, 100) != PICO_OK) {
        return false;
    }

    balance_params_t check;
    return param_store_load(&check) && memcmp(&check, params, sizeof check) == 0;
}
