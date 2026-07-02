// Persistence of balance_params_t in the last 4 KB flash sector.
// Record framing: magic + version + params + CRC32. Writes go through
// flash_safe_execute() and MUST only happen while the motors are disarmed
// (a sector erase stalls XIP for both cores). Called from core 1.
#ifndef BALANCEBOT_PARAM_STORE_H
#define BALANCEBOT_PARAM_STORE_H

#include <stdbool.h>

#include "ipc.h"

// Fill `out` from flash. Returns false (and writes defaults from config.h
// into `out`) if the stored record is missing or corrupt.
bool param_store_load(balance_params_t *out);

// Persist `params`. Returns false on flash error.
bool param_store_save(const balance_params_t *params);

// Reset `out` to the compile-time defaults (no flash access).
void param_store_defaults(balance_params_t *out);

#endif  // BALANCEBOT_PARAM_STORE_H
