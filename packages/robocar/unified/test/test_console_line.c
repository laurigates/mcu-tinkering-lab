/**
 * @file test_console_line.c
 * @brief Host tests for console_line.c run together with the real Improv parser.
 *
 * command_task() feeds each console byte to improv_wifi_process_byte() and
 * then to console_line_feed(); every completed line is logged as
 * `Serial cmd: %s` (mirrored to UART0 and written into the GlitchTip
 * breadcrumb ring) and dispatched. A Send Wi-Fi settings packet carries the
 * password in plaintext, so before this module the whole packet became a
 * console line and the password reached all three sinks. These tests drive
 * the bytes ESP Web Tools sends through both pieces, compiled unmodified, and
 * assert that no packet byte ever reaches a completed line.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "console_line.h"
#include "improv_wifi.h"

static int test_count = 0;
static int test_pass = 0;

#define TEST_ASSERT(cond)                                                            \
    do {                                                                             \
        if (!(cond)) {                                                               \
            printf("FAIL: %s:%d assertion failed: %s\n", __FILE__, __LINE__, #cond); \
            fflush(stdout);                                                          \
            assert(cond);                                                            \
        }                                                                            \
    } while (0)

#define RUN_TEST(fn)            \
    do {                        \
        test_count++;           \
        printf("  %-62s", #fn); \
        fflush(stdout);         \
        reset();                \
        fn();                   \
        test_pass++;            \
        printf("PASS\n");       \
    } while (0)

/* --- Stand-ins for the component's ESP-IDF-only hooks --------------------- */

void improv_wifi_uart0_write(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
}

void improv_wifi_device_info(improv_device_info_t *out)
{
    out->firmware_name = "RoboCar Unified";
    out->firmware_version = "1.2.3";
    out->chip = "ESP32-S3";
    out->device_name = "robocar-unified";
}

static void discard_writer(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
}

/* --- What command_task would do with each completed line ------------------ */

static char s_lines[16][CONSOLE_LINE_MAX];
static int s_line_count;
static char s_pass[IMPROV_WIFI_MAX_PASS_LEN + 1];
static int s_creds_calls;
static console_line_t s_line;

static void on_creds(const char *ssid, const char *password)
{
    (void)ssid;
    snprintf(s_pass, sizeof(s_pass), "%s", password);
    s_creds_calls++;
}

static void reset(void)
{
    memset(s_lines, 0, sizeof(s_lines));
    s_line_count = 0;
    memset(s_pass, 0, sizeof(s_pass));
    s_creds_calls = 0;
    console_line_init(&s_line);
    improv_wifi_set_writer(discard_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
}

/* command_task()'s byte path, with Improv armed or not. */
static void feed(const uint8_t *bytes, size_t len, bool improv_armed)
{
    for (size_t i = 0; i < len; i++) {
        const bool owned = improv_armed && improv_wifi_process_byte(bytes[i]);
        const char *line = console_line_feed(&s_line, (char)bytes[i], owned);
        if (line != NULL) {
            TEST_ASSERT(s_line_count < 16);
            snprintf(s_lines[s_line_count++], CONSOLE_LINE_MAX, "%s", line);
        }
    }
}

static void feed_text(const char *text)
{
    feed((const uint8_t *)text, strlen(text), true);
}

/* A Send Wi-Fi settings packet exactly as sdk-serial-js writes it: header,
 * version, type, length, [0x01, data length, ssid length, ssid, password
 * length, password], checksum over every byte before it, newline. Returns
 * the packet length; *checksum receives the checksum byte. */
static size_t build_credentials_packet(uint8_t *pkt, const char *ssid, const char *pass,
                                       uint8_t *checksum)
{
    const size_t ssid_len = strlen(ssid);
    const size_t pass_len = strlen(pass);
    size_t n = 0;
    memcpy(pkt, "IMPROV", 6);
    n = 6;
    pkt[n++] = 0x01;
    pkt[n++] = 0x03; /* RPC command */
    pkt[n++] = (uint8_t)(2 + 1 + ssid_len + 1 + pass_len);
    pkt[n++] = 0x01; /* Send Wi-Fi settings */
    pkt[n++] = (uint8_t)(1 + ssid_len + 1 + pass_len);
    pkt[n++] = (uint8_t)ssid_len;
    memcpy(pkt + n, ssid, ssid_len);
    n += ssid_len;
    pkt[n++] = (uint8_t)pass_len;
    memcpy(pkt + n, pass, pass_len);
    n += pass_len;
    uint8_t sum = 0;
    for (size_t i = 0; i < n; i++) {
        sum = (uint8_t)(sum + pkt[i]);
    }
    pkt[n++] = sum;
    pkt[n++] = '\n';
    *checksum = sum;
    return n;
}

static bool any_line_contains(const char *needle)
{
    for (int i = 0; i < s_line_count; i++) {
        if (strstr(s_lines[i], needle) != NULL) {
            return true;
        }
    }
    return false;
}

/* --- Tests ---------------------------------------------------------------- */

/* Typed commands still assemble into lines, with either terminator. */
static void test_typed_commands_become_lines(void)
{
    feed_text("status\r\nF\n");
    TEST_ASSERT(s_line_count == 2);
    TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
    TEST_ASSERT(strcmp(s_lines[1], "F") == 0);
}

/* The leak the review found: a credentials packet became a console line, and
 * the line was logged with the password in it. A 10-character password puts
 * 0x0A (its length byte) inside the packet, which used to split it into two
 * lines, the second holding the password. */
static void test_credentials_packet_never_becomes_a_line(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);
    TEST_ASSERT(strlen("s3cr3tpass") == 0x0A);

    feed(pkt, n, true);

    TEST_ASSERT(s_creds_calls == 1); /* the packet was valid and delivered */
    TEST_ASSERT(strcmp(s_pass, "s3cr3tpass") == 0);
    TEST_ASSERT(s_line_count == 0);
    TEST_ASSERT(!any_line_contains("s3cr3tpass"));
    TEST_ASSERT(!any_line_contains("IMPROV"));
}

/* No packet byte is dispatched either. Here the checksum is 'F', which as a
 * one-byte line is "drive forward". */
static void test_a_checksum_byte_is_not_a_movement_command(void)
{
    /* Three printable characters, 90 values each, sum to 268 consecutive
     * values, so every checksum is reachable. */
    char pass[] = "passwoXYZ";
    uint8_t pkt[128];
    uint8_t checksum = 0;
    size_t n = 0;
    for (int c = 0; c < 90 * 90 * 90 && checksum != 'F'; c++) {
        pass[6] = (char)('!' + c / (90 * 90));
        pass[7] = (char)('!' + (c / 90) % 90);
        pass[8] = (char)('!' + c % 90);
        n = build_credentials_packet(pkt, "HomeNet", pass, &checksum);
    }
    TEST_ASSERT(checksum == 'F');

    feed(pkt, n, true);

    TEST_ASSERT(s_creds_calls == 1);
    TEST_ASSERT(s_line_count == 0);
}

/* Commands typed after a packet are unaffected, and text typed before the
 * header on the same line goes with the packet rather than being run. */
static void test_commands_around_a_packet(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    feed_text("stat");
    feed(pkt, n, true);
    feed_text("status\n");

    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
}

/* Without Improv armed, bytes are plain console input, as before. */
static void test_unarmed_bytes_are_plain_input(void)
{
    static const uint8_t text[] = "IMPROV\n";
    feed(text, sizeof(text) - 1, false);
    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "IMPROV") == 0);
}

int main(void)
{
    printf("console_line tests\n");
    RUN_TEST(test_typed_commands_become_lines);
    RUN_TEST(test_credentials_packet_never_becomes_a_line);
    RUN_TEST(test_a_checksum_byte_is_not_a_movement_command);
    RUN_TEST(test_commands_around_a_packet);
    RUN_TEST(test_unarmed_bytes_are_plain_input);
    printf("%d/%d passed\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
