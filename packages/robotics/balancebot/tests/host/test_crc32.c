// Host unit tests for crc32.c (no Pico SDK).
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crc32.h"

static void test_known_vector(void)
{
    // IEEE 802.3 check value for "123456789".
    assert(crc32_calc("123456789", 9) == 0xCBF43926u);
}

static void test_empty(void)
{
    assert(crc32_calc("", 0) == 0x00000000u);
}

static void test_detects_corruption(void)
{
    // A param-record-shaped payload: any single-bit flip must change the CRC.
    uint8_t rec[44];
    for (size_t i = 0; i < sizeof rec; i++) {
        rec[i] = (uint8_t)(i * 37u + 5u);
    }
    uint32_t good = crc32_calc(rec, sizeof rec);
    for (size_t i = 0; i < sizeof rec; i++) {
        rec[i] ^= 0x01u;
        assert(crc32_calc(rec, sizeof rec) != good);
        rec[i] ^= 0x01u;
    }
    assert(crc32_calc(rec, sizeof rec) == good);
}

int main(void)
{
    test_known_vector();
    test_empty();
    test_detects_corruption();
    printf("test_crc32: all tests passed\n");
    return 0;
}
