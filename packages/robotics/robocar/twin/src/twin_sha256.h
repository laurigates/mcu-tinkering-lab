/**
 * @file twin_sha256.h
 * @brief SHA-256 (FIPS 180-4) for the twin's cassette keys.
 *
 * The harness keys cassettes by sha256 of the canonical request descriptor
 * (ADR-030 §5); the twin computes the same digest so a key mismatch is caught
 * at the first request rather than as a cassette that is never found.
 */

#ifndef TWIN_SHA256_H
#define TWIN_SHA256_H

#include <stddef.h>
#include <stdint.h>

/** Hex digest of @p len bytes at @p data into @p out_hex (65 bytes, NUL-terminated). */
void twin_sha256_hex(const void *data, size_t len, char out_hex[65]);

#endif /* TWIN_SHA256_H */
