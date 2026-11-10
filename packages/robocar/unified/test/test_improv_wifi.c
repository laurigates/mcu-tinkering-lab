/**
 * @file test_improv_wifi.c
 * @brief Host tests for the shared improv-wifi component (issues #644, #1056).
 *
 * Transport (#644): the component used to send every reply with
 * uart_write_bytes(UART_NUM_0, ...). On robocar-unified the console is the
 * USB-Serial-JTAG, so the requests arrived over USB while the replies went to
 * an uninitialised UART0 that nothing is wired to. The component now writes
 * through an injectable writer, defaulting to UART0 so the projects whose
 * console *is* UART0 (brainbox) keep working unchanged.
 *
 * Protocol (#1056): the bytes are checked against the Improv Serial spec
 * (https://www.improv-wifi.com/serial/) and against the client ESP Web Tools
 * runs, improv-wifi/sdk-serial-js (src/serial.ts):
 *   - an RPC command's data is [command, data length, payload...], and an RPC
 *     result's is [command, data length, (string length, string)...];
 *   - the checksum is the low byte of the sum of every byte before it, the
 *     "IMPROV" header included (sdk-serial-js writePacketToStream() and
 *     _handleIncomingPacket(); ESPHome's improv_serial sums the same bytes);
 *   - every packet ends with a newline, as both of those do;
 *   - RPC 0x02 is Request current state, 0x03 Request device info;
 *   - error 0x03 is Unable to connect.
 *
 * improv_wifi.c is compiled unmodified from packages/components/improv-wifi.
 * The two ESP-IDF-only hooks it calls are replaced here:
 * improv_wifi_uart0_write() (improv_wifi_uart.c) by a recorder, so a test can
 * tell which transport a reply took, and improv_wifi_device_info()
 * (improv_wifi_device_info.c) by fixed strings.
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
            fflush(stdout);                                                          \
            assert(cond);                                                            \
        }                                                                            \
    } while (0)

#define RUN_TEST(fn)            \
    do {                        \
        test_count++;           \
        printf("  %-62s", #fn); \
        fflush(stdout);         \
        reset_recorders();      \
        fn();                   \
        test_pass++;            \
        printf("PASS\n");       \
    } while (0)

/* --- Two recorders: the UART0 default, and an injected writer ------------- */

typedef struct {
    uint8_t bytes[2048];
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

/* Stands in for the ESP-IDF definition in improv_wifi_device_info.c. */
void improv_wifi_device_info(improv_device_info_t *out)
{
    out->firmware_name = "RoboCar Unified";
    out->firmware_version = "1.2.3";
    out->chip = "ESP32-S3";
    out->device_name = "robocar-unified";
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

/* --- Packet helpers ------------------------------------------------------- */

#define TYPE_CURRENT_STATE 0x01
#define TYPE_ERROR_STATE 0x02
#define TYPE_RPC_COMMAND 0x03
#define TYPE_RPC_RESULT 0x04

/* Sum of 'I','M','P','R','O','V' (477) modulo 256. */
#define HEADER_SUM 0xDD

/* Build and feed one inbound Improv packet exactly as sdk-serial-js's
 * writePacketToStream() does: header, version, type, length, data, a checksum
 * over all of those, and a trailing newline. */
static void feed_packet(uint8_t type, const uint8_t *data, uint8_t len)
{
    uint8_t pkt[6 + 3 + 255 + 2];
    size_t n = 0;
    memcpy(pkt, "IMPROV", 6);
    n = 6;
    pkt[n++] = 0x01;
    pkt[n++] = type;
    pkt[n++] = len;
    memcpy(pkt + n, data, len);
    n += len;
    uint8_t sum = 0;
    for (size_t i = 0; i < n; i++) {
        sum = (uint8_t)(sum + pkt[i]);
    }
    pkt[n++] = sum;
    pkt[n++] = '\n';
    for (size_t i = 0; i < n; i++) {
        improv_wifi_process_byte(pkt[i]);
    }
}

/* Feed an RPC command: [command, payload length, payload...]. */
static void feed_rpc(uint8_t command, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t data[255];
    data[0] = command;
    data[1] = payload_len;
    if (payload_len > 0) {
        memcpy(data + 2, payload, payload_len);
    }
    feed_packet(TYPE_RPC_COMMAND, data, (uint8_t)(payload_len + 2));
}

typedef struct {
    uint8_t type;
    uint8_t len;
    const uint8_t *data;
} packet_t;

/* Split a recording into packets the way sdk-serial-js reads them, asserting
 * the framing of each: header, version 1, a checksum over every preceding
 * byte of the packet, then a newline. Returns the packet count. */
static int parse_packets(const recorder_t *r, packet_t *out, int max)
{
    size_t i = 0;
    int count = 0;
    while (i < r->len) {
        TEST_ASSERT(count < max);
        TEST_ASSERT(r->len - i >= 11);
        TEST_ASSERT(memcmp(r->bytes + i, "IMPROV", 6) == 0);
        TEST_ASSERT(r->bytes[i + 6] == 0x01);
        const uint8_t len = r->bytes[i + 8];
        TEST_ASSERT(r->len - i >= (size_t)9 + len + 2);
        uint8_t sum = 0;
        for (size_t k = i; k < i + 9 + len; k++) {
            sum = (uint8_t)(sum + r->bytes[k]);
        }
        TEST_ASSERT(r->bytes[i + 9 + len] == sum);
        TEST_ASSERT(r->bytes[i + 9 + len + 1] == '\n');
        out[count].type = r->bytes[i + 7];
        out[count].len = len;
        out[count].data = r->bytes + i + 9;
        count++;
        i += (size_t)9 + len + 2;
    }
    return count;
}

/* Decode an RPC result the way sdk-serial-js does, asserting that the data
 * length byte matches the strings that follow. Returns the string count. */
static int rpc_strings(const packet_t *p, uint8_t expect_command, char out[][96], int max)
{
    TEST_ASSERT(p->type == TYPE_RPC_RESULT);
    TEST_ASSERT(p->len >= 2);
    TEST_ASSERT(p->data[0] == expect_command);
    TEST_ASSERT(p->data[1] == p->len - 2);
    int count = 0;
    size_t idx = 2;
    while (idx < p->len) {
        TEST_ASSERT(count < max);
        const uint8_t slen = p->data[idx];
        TEST_ASSERT(idx + 1 + slen <= p->len);
        TEST_ASSERT(slen < 96);
        memcpy(out[count], p->data + idx + 1, slen);
        out[count][slen] = '\0';
        count++;
        idx += (size_t)1 + slen;
    }
    return count;
}

/* --- Transport ------------------------------------------------------------ */

/* brainbox installs the UART0 driver and never sets a writer; its replies
 * must still go to UART0. */
static void test_default_transport_is_uart0(void)
{
    improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);
    TEST_ASSERT(s_uart.calls == 1);
    TEST_ASSERT(s_custom.calls == 0);
}

/* The exact bytes the browser parses: header, version, type, length, data,
 * checksum = ('IMPROV' + 1 + 1 + 1 + 2) % 256 = 0xE2, then a newline. One
 * write call per packet, so a log line cannot land in the middle of it. */
static void test_injected_writer_gets_the_whole_packet_in_one_call(void)
{
    improv_wifi_set_writer(custom_writer);
    improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);

    static const uint8_t expected[] = {'I',  'M',  'P',  'R',  'O',  'V',
                                       0x01, 0x01, 0x01, 0x02, 0xE2, '\n'};
    TEST_ASSERT(HEADER_SUM + 0x01 + 0x01 + 0x01 + 0x02 == 0xE2);
    TEST_ASSERT(s_custom.calls == 1);
    TEST_ASSERT(s_custom.len == sizeof(expected));
    TEST_ASSERT(memcmp(s_custom.bytes, expected, sizeof(expected)) == 0);
    TEST_ASSERT(s_uart.calls == 0);
}

/* robocar-unified sets the writer and only then calls improv_wifi_init().
 * If init cleared the writer, every reply would silently go back to UART0. */
static void test_init_keeps_a_writer_set_before_it(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
    improv_wifi_send_state(IMPROV_STATE_AUTHORIZED);
    TEST_ASSERT(s_custom.calls == 1);
    TEST_ASSERT(s_uart.calls == 0);
}

/* The reply to a request goes back over the channel the request came in on. */
static void test_rpc_reply_takes_the_injected_writer(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    feed_rpc(0x03, NULL, 0);

    TEST_ASSERT(s_uart.calls == 0);
    TEST_ASSERT(s_custom.calls >= 1);
    packet_t pkts[4];
    TEST_ASSERT(parse_packets(&s_custom, pkts, 4) >= 1);
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

/* --- Protocol (#1056) ----------------------------------------------------- */

/* The spec says a device answers every RPC command with an error state of
 * 0x00 first; sdk-serial-js rejects the pending command on any non-zero one. */
static void test_rpc_clears_the_error_state_first(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    feed_rpc(0x02, NULL, 0);

    packet_t pkts[4];
    const int n = parse_packets(&s_custom, pkts, 4);
    TEST_ASSERT(n >= 1);
    TEST_ASSERT(pkts[0].type == TYPE_ERROR_STATE);
    TEST_ASSERT(pkts[0].len == 1);
    TEST_ASSERT(pkts[0].data[0] == 0x00);
}

/* RPC 0x02 is Request current state. ESP Web Tools sends it to detect Improv
 * at all (sdk-serial-js initialize()); a device that does not answer with a
 * Current State packet is "not detected". Right after init the device is
 * offering provisioning, so the state is Ready (Authorized), 0x02. */
static void test_request_current_state_answers_with_the_state(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    feed_rpc(0x02, NULL, 0);

    packet_t pkts[4];
    const int n = parse_packets(&s_custom, pkts, 4);
    TEST_ASSERT(n == 2); /* error 0x00, current state; no RPC result */
    TEST_ASSERT(pkts[1].type == TYPE_CURRENT_STATE);
    TEST_ASSERT(pkts[1].len == 1);
    TEST_ASSERT(pkts[1].data[0] == IMPROV_STATE_AUTHORIZED);
}

/* The state reported is the last one the application sent. */
static void test_request_current_state_reports_the_last_state_sent(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
    improv_wifi_send_state(IMPROV_STATE_PROVISIONING);
    memset(&s_custom, 0, sizeof(s_custom));

    feed_rpc(0x02, NULL, 0);

    packet_t pkts[4];
    const int n = parse_packets(&s_custom, pkts, 4);
    TEST_ASSERT(n == 2);
    TEST_ASSERT(pkts[1].type == TYPE_CURRENT_STATE);
    TEST_ASSERT(pkts[1].data[0] == IMPROV_STATE_PROVISIONING);
}

/* When provisioned, the spec adds "the same response you would get if device
 * provisioning was successful". sdk-serial-js waits for an RPC result echoing
 * command 0x02 and takes its first string as the device URL. */
static void test_request_current_state_when_provisioned_replays_result(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
    improv_wifi_send_state(IMPROV_STATE_PROVISIONED);
    improv_wifi_send_provisioned_result("http://robocar-unified.local");
    memset(&s_custom, 0, sizeof(s_custom));

    feed_rpc(0x02, NULL, 0);

    packet_t pkts[4];
    const int n = parse_packets(&s_custom, pkts, 4);
    TEST_ASSERT(n == 3);
    TEST_ASSERT(pkts[1].type == TYPE_CURRENT_STATE);
    TEST_ASSERT(pkts[1].data[0] == IMPROV_STATE_PROVISIONED);
    char s[4][96];
    TEST_ASSERT(rpc_strings(&pkts[2], 0x02, s, 4) == 1);
    TEST_ASSERT(strcmp(s[0], "http://robocar-unified.local") == 0);
}

/* With no redirect URL the result carries no strings. An empty string would
 * not do: ESP Web Tools shows "Visit Device" whenever the URL is not
 * undefined, so "" would render a link to nowhere. */
static void test_provisioned_result_without_a_url_has_no_strings(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    improv_wifi_send_provisioned_result(NULL);
    packet_t pkts[4];
    TEST_ASSERT(parse_packets(&s_custom, pkts, 4) == 1);
    char s[4][96];
    TEST_ASSERT(rpc_strings(&pkts[0], 0x01, s, 4) == 0);

    /* And the 0x02 replay of it, once provisioned. */
    improv_wifi_send_state(IMPROV_STATE_PROVISIONED);
    memset(&s_custom, 0, sizeof(s_custom));
    feed_rpc(0x02, NULL, 0);
    TEST_ASSERT(parse_packets(&s_custom, pkts, 4) == 3);
    TEST_ASSERT(rpc_strings(&pkts[2], 0x02, s, 4) == 0);
}

/* The successful-provisioning result itself echoes command 0x01. */
static void test_provisioned_result_echoes_command_0x01(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    improv_wifi_send_provisioned_result("http://10.0.0.7");
    packet_t pkts[4];
    TEST_ASSERT(parse_packets(&s_custom, pkts, 4) == 1);
    char s[4][96];
    TEST_ASSERT(rpc_strings(&pkts[0], 0x01, s, 4) == 1);
    TEST_ASSERT(strcmp(s[0], "http://10.0.0.7") == 0);
}

/* "Unable to connect" is error 0x03 in the spec and in sdk-serial-js. It was
 * 0xFE, which the SDK reads as its own local TIMEOUT. */
static void test_unable_to_connect_is_error_0x03(void)
{
    improv_wifi_set_writer(custom_writer);
    improv_wifi_send_error(IMPROV_ERROR_UNABLE_CONNECT);

    packet_t pkts[2];
    TEST_ASSERT(parse_packets(&s_custom, pkts, 2) == 1);
    TEST_ASSERT(pkts[0].type == TYPE_ERROR_STATE);
    TEST_ASSERT(pkts[0].len == 1);
    TEST_ASSERT(pkts[0].data[0] == 0x03);
}

/* RPC 0x03 returns the four strings in the spec's order: firmware name,
 * firmware version, chip, device name. ESP Web Tools compares the firmware
 * name with the manifest's name to choose "Update" over "Install" (which
 * erases flash, NVS credentials included) and the version with the
 * manifest's version. The strings come from improv_wifi_device_info(). */
static void test_request_device_info_returns_the_platform_strings(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    feed_rpc(0x03, NULL, 0);

    packet_t pkts[4];
    TEST_ASSERT(parse_packets(&s_custom, pkts, 4) == 2);
    TEST_ASSERT(pkts[0].type == TYPE_ERROR_STATE && pkts[0].data[0] == 0x00);
    char s[6][96];
    TEST_ASSERT(rpc_strings(&pkts[1], 0x03, s, 6) == 4);
    TEST_ASSERT(strcmp(s[0], "RoboCar Unified") == 0);
    TEST_ASSERT(strcmp(s[1], "1.2.3") == 0);
    TEST_ASSERT(strcmp(s[2], "ESP32-S3") == 0);
    TEST_ASSERT(strcmp(s[3], "robocar-unified") == 0);
}

/* The spec's own example for RPC 0x01:
 *   01 1E 0C {MyWirelessAP} 10 {mysecurepassword}
 * byte 2 is the data length, then the SSID and password, each length-prefixed. */
static void test_send_wifi_settings_parses_the_spec_example(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
    s_creds_calls = 0;

    uint8_t rpc[64];
    uint8_t n = 0;
    rpc[n++] = 0x01;
    rpc[n++] = 0x1E;
    rpc[n++] = 0x0C;
    memcpy(rpc + n, "MyWirelessAP", 12);
    n += 12;
    rpc[n++] = 0x10;
    memcpy(rpc + n, "mysecurepassword", 16);
    n += 16;
    TEST_ASSERT(rpc[1] == n - 2);

    feed_packet(TYPE_RPC_COMMAND, rpc, n);

    TEST_ASSERT(s_creds_calls == 1);
    TEST_ASSERT(strcmp(s_ssid, "MyWirelessAP") == 0);
    TEST_ASSERT(strcmp(s_pass, "mysecurepassword") == 0);

    packet_t pkts[4];
    const int np = parse_packets(&s_custom, pkts, 4);
    TEST_ASSERT(np >= 2);
    TEST_ASSERT(pkts[0].type == TYPE_ERROR_STATE && pkts[0].data[0] == 0x00);
    TEST_ASSERT(pkts[1].type == TYPE_CURRENT_STATE && pkts[1].data[0] == IMPROV_STATE_PROVISIONING);
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
    uint8_t payload[64];
    uint8_t n = 0;
    payload[n++] = (uint8_t)strlen(ssid);
    memcpy(payload + n, ssid, strlen(ssid));
    n = (uint8_t)(n + strlen(ssid));
    payload[n++] = (uint8_t)strlen(pass);
    memcpy(payload + n, pass, strlen(pass));
    n = (uint8_t)(n + strlen(pass));
    TEST_ASSERT(payload[0] == 0x0D);

    feed_rpc(0x01, payload, n);

    TEST_ASSERT(s_creds_calls == 1);
    TEST_ASSERT(strcmp(s_ssid, ssid) == 0);
    TEST_ASSERT(strcmp(s_pass, pass) == 0);
    TEST_ASSERT(s_uart.calls == 0);
}

/* A data length byte that disagrees with the packet is an invalid RPC, and
 * the credentials callback never runs, even when the payload after it would
 * parse: here SSID "a" and password "b", 4 bytes, under a length byte of 5. */
static void test_rpc_with_a_wrong_data_length_is_invalid(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
    s_creds_calls = 0;

    static const uint8_t rpc[] = {0x01, 0x05, 0x01, 'a', 0x01, 'b'};
    feed_packet(TYPE_RPC_COMMAND, rpc, sizeof(rpc));

    TEST_ASSERT(s_creds_calls == 0);
    packet_t pkts[4];
    const int n = parse_packets(&s_custom, pkts, 4);
    TEST_ASSERT(n >= 1);
    TEST_ASSERT(pkts[n - 1].type == TYPE_ERROR_STATE);
    TEST_ASSERT(pkts[n - 1].data[0] == IMPROV_ERROR_INVALID_RPC);
}

/* A checksum that leaves out the header (what the component used to compute)
 * is rejected, so the parser and sdk-serial-js agree on the algorithm. */
static void test_checksum_without_the_header_is_rejected(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    static const uint8_t pkt[] = {
        'I',  'M',  'P',  'R',
        'O',  'V',  0x01, TYPE_RPC_COMMAND,
        0x02, 0x02, 0x00, (uint8_t)(0x01 + TYPE_RPC_COMMAND + 0x02 + 0x02)};
    for (size_t i = 0; i < sizeof(pkt); i++) {
        improv_wifi_process_byte(pkt[i]);
    }

    packet_t pkts[2];
    TEST_ASSERT(parse_packets(&s_custom, pkts, 2) == 1);
    TEST_ASSERT(pkts[0].type == TYPE_ERROR_STATE);
    TEST_ASSERT(pkts[0].data[0] == IMPROV_ERROR_INVALID_RPC);
}

/* An RPC the component does not implement answers error 0x02, after the
 * 0x00 the spec requires. 0x04 (scan) is one; clients treat it as optional. */
static void test_unknown_rpc_is_error_0x02(void)
{
    improv_wifi_set_writer(custom_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);

    feed_rpc(0x04, NULL, 0);

    packet_t pkts[4];
    TEST_ASSERT(parse_packets(&s_custom, pkts, 4) == 2);
    TEST_ASSERT(pkts[0].type == TYPE_ERROR_STATE && pkts[0].data[0] == 0x00);
    TEST_ASSERT(pkts[1].type == TYPE_ERROR_STATE && pkts[1].data[0] == 0x02);
}

int main(void)
{
    printf("improv_wifi tests\n");
    RUN_TEST(test_default_transport_is_uart0);
    RUN_TEST(test_injected_writer_gets_the_whole_packet_in_one_call);
    RUN_TEST(test_init_keeps_a_writer_set_before_it);
    RUN_TEST(test_rpc_reply_takes_the_injected_writer);
    RUN_TEST(test_null_writer_restores_uart0);
    RUN_TEST(test_rpc_clears_the_error_state_first);
    RUN_TEST(test_request_current_state_answers_with_the_state);
    RUN_TEST(test_request_current_state_reports_the_last_state_sent);
    RUN_TEST(test_request_current_state_when_provisioned_replays_result);
    RUN_TEST(test_provisioned_result_without_a_url_has_no_strings);
    RUN_TEST(test_provisioned_result_echoes_command_0x01);
    RUN_TEST(test_unable_to_connect_is_error_0x03);
    RUN_TEST(test_request_device_info_returns_the_platform_strings);
    RUN_TEST(test_send_wifi_settings_parses_the_spec_example);
    RUN_TEST(test_a_13_byte_ssid_round_trips);
    RUN_TEST(test_rpc_with_a_wrong_data_length_is_invalid);
    RUN_TEST(test_checksum_without_the_header_is_rejected);
    RUN_TEST(test_unknown_rpc_is_error_0x02);
    printf("%d/%d passed\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
