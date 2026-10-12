#include "param_store.h"

#include <string.h>

#include "config.h"
#include "hardware/flash.h"
#include "param_record.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

_Static_assert(PARAM_RECORD_MAX_SIZE <= FLASH_PAGE_SIZE, "record must fit one flash page");

void param_store_defaults(balance_params_t *out)
{
    param_defaults(out);
}

bool param_store_load(balance_params_t *out)
{
    // Record layout and the v1 -> v2 migration live in param_record.c.
    const uint8_t *rec = (const uint8_t *)(XIP_BASE + PARAM_FLASH_OFFSET);
    if (param_record_decode(rec, FLASH_PAGE_SIZE, out, NULL)) {
        return true;
    }
    param_defaults(out);
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
    if (param_record_encode(params, page, sizeof page) == 0) {
        return false;
    }

    if (flash_safe_execute(do_flash_write, page, 100) != PICO_OK) {
        return false;
    }

    balance_params_t check;
    return param_store_load(&check) && memcmp(&check, params, sizeof check) == 0;
}
