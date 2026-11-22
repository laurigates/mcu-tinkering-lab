// Flash record framing for balance_params_t: magic + version + params +
// CRC32, plus the migrations from v1, v2 and v3 to v4. Pure C (no SDK dependencies) so the
// layout and the migration are host-tested (tests/host/test_param_record.c);
// param_store.c only moves the bytes to and from flash.
#ifndef BALANCEBOT_PARAM_RECORD_H
#define BALANCEBOT_PARAM_RECORD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ipc.h"

#define PARAM_MAGIC 0x42414C42u  // "BALB"
#define PARAM_VERSION 4u

// v1 (PID only): magic, version, 7 floats, CRC over magic..floats = 40 bytes.
#define PARAM_V1_FLOATS 7u
#define PARAM_V1_SIZE (4u + 4u + PARAM_V1_FLOATS * 4u + 4u)
// v2 (adds LQR): magic, version, 16 four-byte fields, CRC = 76 bytes.
#define PARAM_V2_FIELDS 16u
#define PARAM_V2_SIZE (4u + 4u + PARAM_V2_FIELDS * 4u + 4u)
// v3 (adds imu_mount): magic, version, 17 four-byte fields, CRC = 80 bytes.
#define PARAM_V3_FIELDS 17u
#define PARAM_V3_SIZE (4u + 4u + PARAM_V3_FIELDS * 4u + 4u)
// v4 (adds pitch_trim_deg): magic, version, balance_params_t, CRC over magic..params.
#define PARAM_V4_SIZE (4u + 4u + sizeof(balance_params_t) + 4u)
#define PARAM_RECORD_MAX_SIZE PARAM_V4_SIZE

// Compile-time defaults (config.h), no flash access.
void param_defaults(balance_params_t *out);

// Encode `p` as a current-version record into `buf`. Returns the record
// size, or 0 if `cap` is too small.
size_t param_record_encode(const balance_params_t *p, uint8_t *buf, size_t cap);

// Decode a record of any supported version. On success fills `out`
// (migrating older versions: saved fields kept, new fields defaulted; an
// out-of-range LQR-side field, IMU mounting or pitch trim is replaced by its default), stores the
// record's version in *version if non-NULL, and returns true. On a bad
// magic, unknown version, short buffer or CRC mismatch returns false and
// leaves `out` untouched.
bool param_record_decode(const uint8_t *buf, size_t len, balance_params_t *out, uint32_t *version);

// A pitch trim the firmware accepts: finite and within +/- PITCH_TRIM_MAX_DEG.
// The CLI, core 1 and the decoder all use it.
bool pitch_trim_valid(float deg);

#endif  // BALANCEBOT_PARAM_RECORD_H
