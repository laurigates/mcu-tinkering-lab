/**
 * @file unity_compat.h
 * @brief Minimal Unity-compatible test framework for host-based testing.
 *
 * Provides a subset of the Unity API that compiles without the full Unity
 * library. This allows running protocol and logic tests on the host (Linux/macOS)
 * without ESP-IDF. Compatible macro names make migration to full ESP-IDF Unity
 * straightforward.
 */
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int _unity_total    = 0;
static int _unity_failures = 0;

static void _unity_begin(void)
{
    _unity_total    = 0;
    _unity_failures = 0;
    printf("\n-----------------------\n");
    printf("Running host unit tests\n");
    printf("-----------------------\n\n");
}

static int _unity_end(void)
{
    printf("\n-----------------------\n");
    printf("%d Tests %d Failures\n", _unity_total, _unity_failures);
    printf("%s\n", _unity_failures == 0 ? "OK" : "FAIL");
    printf("-----------------------\n\n");
    return _unity_failures;
}

#define UNITY_BEGIN() _unity_begin()
#define UNITY_END()   _unity_end()

#define RUN_TEST(fn)                 \
    do {                             \
        printf("TEST(%s)\n", #fn);   \
        fn();                        \
    } while (0)

/* ---------------------------------------------------------------------- */
/* Internal failure reporter                                               */
/* ---------------------------------------------------------------------- */
#define _UNITY_FAIL(fmt, ...)                                                      \
    do {                                                                           \
        _unity_failures++;                                                         \
        printf("  FAIL at %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    } while (0)

/* ---------------------------------------------------------------------- */
/* Assertion macros                                                        */
/* ---------------------------------------------------------------------- */
#define TEST_ASSERT_TRUE(cond)       \
    do {                             \
        _unity_total++;              \
        if (!(cond))                 \
            _UNITY_FAIL("expected TRUE was FALSE"); \
    } while (0)

#define TEST_ASSERT_FALSE(cond) TEST_ASSERT_TRUE(!(cond))

#define TEST_ASSERT_EQUAL(expected, actual)                             \
    do {                                                                \
        _unity_total++;                                                 \
        if ((int64_t)(expected) != (int64_t)(actual))                  \
            _UNITY_FAIL("expected %lld was %lld",                      \
                        (long long)(expected), (long long)(actual));   \
    } while (0)

#define TEST_ASSERT_NOT_EQUAL(expected, actual)                         \
    do {                                                                \
        _unity_total++;                                                 \
        if ((int64_t)(expected) == (int64_t)(actual))                  \
            _UNITY_FAIL("expected values to differ (both %lld)",       \
                        (long long)(actual));                           \
    } while (0)

#define TEST_ASSERT_EQUAL_UINT8  TEST_ASSERT_EQUAL
#define TEST_ASSERT_EQUAL_UINT16 TEST_ASSERT_EQUAL
#define TEST_ASSERT_EQUAL_UINT32 TEST_ASSERT_EQUAL
#define TEST_ASSERT_EQUAL_INT    TEST_ASSERT_EQUAL
#define TEST_ASSERT_EQUAL_INT8   TEST_ASSERT_EQUAL

#define TEST_ASSERT_NULL(ptr)                           \
    do {                                                \
        _unity_total++;                                 \
        if ((ptr) != NULL)                              \
            _UNITY_FAIL("expected NULL pointer");       \
    } while (0)

#define TEST_ASSERT_NOT_NULL(ptr)                       \
    do {                                                \
        _unity_total++;                                 \
        if ((ptr) == NULL)                              \
            _UNITY_FAIL("expected non-NULL pointer");  \
    } while (0)

#define TEST_ASSERT_EQUAL_STRING(expected, actual)                               \
    do {                                                                         \
        _unity_total++;                                                          \
        if (strcmp((expected), (actual)) != 0)                                   \
            _UNITY_FAIL("expected \"%s\" was \"%s\"", (expected), (actual));    \
    } while (0)

#define TEST_ASSERT_EQUAL_MEMORY(expected, actual, len)             \
    do {                                                            \
        _unity_total++;                                             \
        if (memcmp((expected), (actual), (size_t)(len)) != 0)      \
            _UNITY_FAIL("memory blocks differ (%d bytes)", (int)(len)); \
    } while (0)
