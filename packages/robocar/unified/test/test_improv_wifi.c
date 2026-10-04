/**
 * @file test_improv_wifi.c
 * @brief Host tests for the Improv Serial reply transport (issue #644).
 *
 * The shared improv-wifi component used to send every reply with
 * uart_write_bytes(UART_NUM_0, ...). On robocar-unified the console is the
 * USB-Serial-JTAG, so the requests arrived over USB while the replies went to
 * an uninitialised UART0 that nothing is wired to — the browser never heard
 * back. The component now writes through an injectable writer, defaulting to
 * UART0 so the projects whose console *is* UART0 (robocar-main, brainbox)
 * keep working unchanged.
 *
 * improv_wifi.c is compiled unmodified from packages/components/improv-wifi.
 * improv_wifi_uart0_write() — the ESP-IDF default, in improv_wifi_uart.c — is
 * replaced here by a recorder, so a test can tell which transport a reply
 * actually took.
 */

#include "improv_wifi.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static int test_count = 0;
static int test_pass = 0;

#define TEST_ASSERT(cond)                                                            \
    do {                                                                             \
        if (!(cond)) {                                                               \
            printf("FAIL: %s:%d assertion failed: %s\n", __FILE__, __LINE__, #cond); \
            assert(cond);                                                            \
        }                                                                            \
    } while (0)

#define RUN_TEST(fn)            \
    do {                        \
        test_count++;           \
        printf("  %-62s", #fn); \
        reset_recorders();      \
        fn();                   \
        test_pass++;            \
        printf("PASS\n");       \
    } while (0)

/* --- Two recorders: the UART0 default, and an injected writer ------------- */

typedef struct {
    uint8_t bytes[1024];
    size_t len;
    int calls;
} recorder_t;

static recorder_t s_uart;
static recorder_t s_custom;

static void record(recorder_t *r, const uint8_t *data, size_t len)
{
    TEST_ASSERT(r->len + len <= sizeof(r->bytes));
    memcpy(r->bytes + r->len, data, len);
    r->len += len;
    r->calls++;
}

/* Stands in for the ESP-IDF definition in improv_wifi_uart.c. */
void improv_wifi_uart0_write(const uint8_t *data, size_t len)
{
    record(&s_uart, data, len);
}

static void custom_writer(const uint8_t *data, size_t len)
{
    record(&s_custom, data, len);
}

static void reset_recorders(void)
{
    memset(&s_uart, 0, sizeof(s_uart));
    memset(&s_custom, 0, sizeof(s_custom));
    improv_wifi_set_writer(NULL);
}

/* --- Credentials callback ------------------------------------------------- */

static char s_ssid[IMPROV_WIFI_MAX_SSID_LEN + 1];
static char s_pass[IMPROV_WIFI_MAX_PASS_LEN + 1];
static int s_creds_calls;

static void on_creds(const char *ssid, const char *password)
{
    snprintf(s_ssid, sizeof(s_ssid), "%s", ssid);
    snprintf(s_pass, sizeof(s_pass), "%s", password);
    s_creds_calls++;
}

/* Build and feed one inbound Improv packet, exactly as a host would send it. */
static void feed_packet(uint8_t type, const uint8_t *data, uint8_t len)
{
    static const uint8_t hdr[] = {'I', 'M', 'P', 'R', 'O', 'V', 0x01};
    uint8_t sum = (uint8_t)(0x01 + type + len);
    for (size_t i = 0; i < sizeof(hdr); i++) {
        improv_wifi_process_byte(hdr[i]);
    }
    improv_wifi_process_byte(type);
    improv_wifi_process_byte(len);
    for (uint8_t i = 0; i < len; i++) {
        improv_wifi_process_byte(data[i]);
        sum = (uint8_t)(sum + data[i]);
    }
    improv_wifi_process_byte(sum);
}

/* --- Tests ---------------------------------------------------------------- */

/* robocar-main and brainbox install the UART0 driver and never set a writer;
 * their replies must still go to UART0. */
static void test_default_transport_is_uart0(void)
{
    improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);
    TEST_ASSERT(s_uart.calls == 1);
    TEST_ASSERT(s_custom.calls == 0);
}

/* The exact bytes the browser parses: header, version, type, length, data,
 * checksum = (1 + 1 + 1 + 2) % 256 = 5. One write call per packet, so a log
 * line cannot be interleaved into the middle of it by a second call. */
static void test_injected_writer_gets_the_whole_packet_in_one_call(void)
{
    improv_wifi_set_writer(custom_writer);
    improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);

    static const uint8_t expected[] = {'I', 'M', 'P', 'R', 'O', 'V', 0x01, 0x01, 0x01, 0x02, 0x05};
    TEST_ASSERT(s_custom.calls == 1);
    TEST_ASSERT(s_custom.len == sizeof(expected));
    TEST_ASSERT(memcmp(s_custom.bytes, expected, sizeof(expected)) == 0);
    TEST_ASSERT(s_uart.calls == 0);
}

/* robocar-unified sets the writer and only then calls improv_wifi_init().
 * If init cleared the writer, every reply would silently go back to UART0 —
 * the bug this issue is about, reintroduced by call order. */
static void test_init_keeps_a_writer_set_before_it(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
    improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);
    TEST_ASSERT(s_custom.calls == 1);
    TEST_ASSERT(s_uart.calls == 0);
}

/* The reply to a request goes back over the channel the request came in on:
 * a REQUEST_INFO RPC is answered with an RPC_RESULT through the writer. */
static void test_rpc_reply_takes_the_injected_writer(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    static const uint8_t request_info[] = {0x03, 0x00};
    feed_packet(0x03, request_info, sizeof(request_info));

    TEST_ASSERT(s_uart.calls == 0);
    TEST_ASSERT(s_custom.calls == 1);
    TEST_ASSERT(s_custom.len > 10);
    TEST_ASSERT(memcmp(s_custom.bytes, "IMPROV", 6) == 0);
    TEST_ASSERT(s_custom.bytes[7] == 0x04); /* TYPE_RPC_RESULT */
    TEST_ASSERT(s_custom.bytes[9] == 0x03); /* answers CMD_REQUEST_INFO */

    /* Checksum over version..data matches the trailing byte. */
    uint8_t sum = 0;
    for (size_t i = 6; i < s_custom.len - 1; i++) {
        sum = (uint8_t)(sum + s_custom.bytes[i]);
    }
    TEST_ASSERT(sum == s_custom.bytes[s_custom.len - 1]);
}

/* Credentials whose framing contains 0x0D (a 13-byte SSID: its length byte is
 * a carriage return) parse intact when the transport delivers bytes raw. The
 * firmware therefore switches the console's RX line-ending translation off —
 * under the default CR->LF mapping this length byte would arrive as 0x0A. */
static void test_a_13_byte_ssid_round_trips(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
    s_creds_calls = 0;

    const char *ssid = "thirteen-char"; /* 13 bytes == 0x0D */
    const char *pass = "pw";
    uint8_t rpc[64];
    uint8_t n = 0;
    rpc[n++] = 0x01; /* CMD_SEND_WIFI_CREDS */
    rpc[n++] = (uint8_t)strlen(ssid);
    memcpy(rpc + n, ssid, strlen(ssid));
    n = (uint8_t)(n + strlen(ssid));
    rpc[n++] = (uint8_t)strlen(pass);
    memcpy(rpc + n, pass, strlen(pass));
    n = (uint8_t)(n + strlen(pass));
    TEST_ASSERT(rpc[1] == 0x0D);

    feed_packet(0x03, rpc, n);

    TEST_ASSERT(s_creds_calls == 1);
    TEST_ASSERT(strcmp(s_ssid, ssid) == 0);
    TEST_ASSERT(strcmp(s_pass, pass) == 0);
    TEST_ASSERT(s_uart.calls == 0);
}

/* Passing NULL restores the UART0 default. */
static void test_null_writer_restores_uart0(void)
{
    improv_wifi_set_writer(custom_writer);
    improv_wifi_set_writer(NULL);
    improv_wifi_send_error(IMPROV_ERROR_UNKNOWN_CMD);
    TEST_ASSERT(s_uart.calls == 1);
    TEST_ASSERT(s_custom.calls == 0);
}

int main(void)
{
    printf("improv_wifi transport tests\n");
    RUN_TEST(test_default_transport_is_uart0);
    RUN_TEST(test_injected_writer_gets_the_whole_packet_in_one_call);
    RUN_TEST(test_init_keeps_a_writer_set_before_it);
    RUN_TEST(test_rpc_reply_takes_the_injected_writer);
    RUN_TEST(test_a_13_byte_ssid_round_trips);
    RUN_TEST(test_null_writer_restores_uart0);
    printf("%d/%d passed\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
