// Software CRC-32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF).
// Pure C (no SDK dependencies) so the parameter-store framing can be
// host-unit-tested.
#ifndef BALANCEBOT_CRC32_H
#define BALANCEBOT_CRC32_H

#include <stddef.h>
#include <stdint.h>

uint32_t crc32_calc(const void *data, size_t len);

#endif  // BALANCEBOT_CRC32_H
