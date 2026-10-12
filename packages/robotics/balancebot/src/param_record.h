// Flash record framing for balance_params_t: magic + version + params +
// CRC32, plus the v1 -> v2 migration. Pure C (no SDK dependencies) so the
// layout and the migration are host-tested (tests/host/test_param_record.c);
// param_store.c only moves the bytes to and from flash.
#ifndef BALANCEBOT_PARAM_RECORD_H
#define BALANCEBOT_PARAM_RECORD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ipc.h"

#define PARAM_MAGIC 0x42414C42u  // "BALB"
#define PARAM_VERSION 2u

// v1 (PID only): magic, version, 7 floats, CRC over magic..floats = 40 bytes.
#define PARAM_V1_FLOATS 7u
#define PARAM_V1_SIZE (4u + 4u + PARAM_V1_FLOATS * 4u + 4u)
// v2: magic, version, balance_params_t, CRC over magic..params.
#define PARAM_V2_SIZE (4u + 4u + sizeof(balance_params_t) + 4u)
#define PARAM_RECORD_MAX_SIZE PARAM_V2_SIZE

// Compile-time defaults (config.h), no flash access.
void param_defaults(balance_params_t *out);

// Encode `p` as a current-version record into `buf`. Returns the record
// size, or 0 if `cap` is too small.
size_t param_record_encode(const balance_params_t *p, uint8_t *buf, size_t cap);

// Decode a record of any supported version. On success fills `out`
// (migrating older versions: saved fields kept, new fields defaulted; an
// out-of-range LQR-side field is replaced by its default), stores the
// record's version in *version if non-NULL, and returns true. On a bad
// magic, unknown version, short buffer or CRC mismatch returns false and
// leaves `out` untouched.
bool param_record_decode(const uint8_t *buf, size_t len, balance_params_t *out, uint32_t *version);

#endif  // BALANCEBOT_PARAM_RECORD_H
