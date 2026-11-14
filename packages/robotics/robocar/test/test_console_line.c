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
 * assert that no packet byte ever reaches a completed line, including when
 * Improv is armed or disarmed in the middle of a packet and when a packet is
 * cut off or stalled (issue #1087).
 *
 * Bytes are read one millisecond apart, as a packet written in one go arrives;
 * pause() stands for a quiet gap on the port.
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
    out->firmware_name = "RoboCar";
    out->firmware_version = "1.2.3";
    out->chip = "ESP32-S3";
    out->device_name = "robocar";
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
/* The firmware's s_improv_session: bumped by every arm and every disarm, odd
 * while armed. */
static uint32_t s_session;
/* command_task's read clock, in milliseconds. */
static uint32_t s_now_ms;

/* A disarm and a re-arm take two network supervisor steps, which run
 * NET_SUPERVISOR_PERIOD_MS (500 ms) apart, so both can fall between two
 * console bytes only when the second byte is read at least this much later. */
#define EDGE_PAIR_GAP_MS 500u
/* Long enough for anything: a person typing the next command. */
#define LATER_MS 2000u

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
    s_session = 0;
    s_now_ms = 0;
    console_line_init(&s_line);
    improv_wifi_set_writer(discard_writer);
    TEST_ASSERT(improv_wifi_init(on_creds) == 0);
}

/* The network supervisor arming or disarming Improv between two bytes. */
static void set_armed(bool armed)
{
    if (console_line_improv_armed(s_session) != armed) {
        s_session++;
    }
}

/* Nothing on the port for @p ms. */
static void pause(uint32_t ms)
{
    s_now_ms += ms;
}

/* command_task()'s byte path, with Improv armed or not. */
static void feed(const uint8_t *bytes, size_t len, bool improv_armed)
{
    set_armed(improv_armed);
    for (size_t i = 0; i < len; i++) {
        s_now_ms += 1;
        const char *line = console_line_feed(&s_line, (char)bytes[i], s_session, s_now_ms);
        if (line != NULL) {
            TEST_ASSERT(s_line_count < 16);
            // cppcheck-suppress arrayIndexOutOfBoundsCond ; TEST_ASSERT aborts at 16
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
    pause(LATER_MS);
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

/* Issue #1087, the stale parser: the link comes up in the middle of a packet,
 * so Improv is disarmed with the parser still counting data bytes, and armed
 * again later. The rest of that packet never comes; the next packet does,
 * after the gap the two edges take. Without a reset the parser spends that
 * stale count on the next packet, takes one of its bytes as the checksum,
 * fails, and hands the rest of it, password included, to the console line. */
static void test_a_packet_cut_off_by_a_link_edge_does_not_leak_the_next_one(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    feed(pkt, 14, true); /* header through two SSID bytes */
    set_armed(false);    /* link up: disarmed, no byte read in between */
    set_armed(true);     /* link down again: re-armed */
    pause(EDGE_PAIR_GAP_MS);
    feed(pkt, n, true);

    TEST_ASSERT(s_creds_calls == 1);
    TEST_ASSERT(strcmp(s_pass, "s3cr3tpass") == 0);
    TEST_ASSERT(s_line_count == 0);
    TEST_ASSERT(!any_line_contains("pass"));
}

/* The same edge seen from the other side: the rest of the packet arrives after
 * the disarm. Its bytes cannot be told from text without the parser, and a
 * 10-character password puts a 0x0A length byte in front of the password, so
 * the parser keeps reading until the packet in flight ends and is reset then.
 * Nothing of it becomes a line; the next command does. */
static void test_a_packet_in_flight_at_disarm_is_finished_not_logged(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    feed(pkt, 14, true);
    feed(pkt + 14, n - 14, false); /* the rest, password and newline, after the disarm */
    pause(LATER_MS);
    feed((const uint8_t *)"status\n", 7, false);

    TEST_ASSERT(s_creds_calls == 1); /* the packet the user sent still lands */
    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
    TEST_ASSERT(!any_line_contains("pass"));
}

/* Once that packet has ended, the parser is not fed any more while disarmed:
 * a header typed then is text. */
static void test_disarm_takes_effect_when_the_packet_in_flight_ends(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    feed(pkt, 14, true);
    feed(pkt + 14, n - 14, false);
    pause(LATER_MS);
    feed((const uint8_t *)"IMPROV\n", 7, false);

    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "IMPROV") == 0);
}

/* A packet whose RPC length byte is too small, the shape of the bug #1075
 * fixed in the client direction: the parser takes a password byte as the
 * checksum, rejects the packet and claims nothing after it. The rest of the
 * password, up to the packet's newline, must not become a line; the next
 * command still runs. */
static void test_a_packet_with_a_short_length_byte_leaks_nothing(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);
    pkt[8] = (uint8_t)(pkt[8] - 8); /* the packet length, eight short */
    uint8_t sum = 0;
    for (size_t i = 0; i < n - 2; i++) {
        sum = (uint8_t)(sum + pkt[i]);
    }
    pkt[n - 2] = sum; /* a client checksums what it sends */

    feed(pkt, n, true);
    pause(LATER_MS);
    feed_text("status\n");

    TEST_ASSERT(s_creds_calls == 0);
    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
    TEST_ASSERT(!any_line_contains("tpass"));
}

/* A packet rejected after it lost framing ahead of a 0x0A: twelve short, the
 * parser takes the SSID's 't' as the checksum, and the password's length byte
 * (0x0A) follows. Dropping only up to that newline let the password through as
 * the next line, the limit wifi-and-ota.md used to document; the whole burst
 * is dropped now. */
static void test_a_rejected_packet_with_a_newline_in_its_tail_leaks_nothing(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);
    pkt[8] = (uint8_t)(pkt[8] - 12);
    TEST_ASSERT(pkt[9 + pkt[8] + 1] == 0x0A); /* right after the byte taken as checksum */

    feed(pkt, n, true);
    pause(LATER_MS);
    feed_text("status\n");

    TEST_ASSERT(s_creds_calls == 0);
    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
    TEST_ASSERT(!any_line_contains("tpass"));
}

/* Dropping the rest of a burst still lets a packet in it through: a client
 * may send two in one write. */
static void test_two_packets_in_one_burst_both_parse(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    feed(pkt, n, true);
    feed(pkt, n, true);

    TEST_ASSERT(s_creds_calls == 2);
    TEST_ASSERT(s_line_count == 0);
}

/* The cut of the stale-parser test, with the next packet arriving inside the
 * same burst, so it cannot be told from the cut packet's tail. The parser
 * mis-frames and rejects it, and the rest of the burst is dropped: the
 * credentials are not delivered, and nothing of them is logged. */
static void test_a_new_packet_inside_the_burst_after_a_cut_leaks_nothing(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    feed(pkt, 14, true);
    set_armed(false);
    set_armed(true);
    feed(pkt, n, true);
    pause(LATER_MS);
    feed_text("status\n");

    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
    TEST_ASSERT(!any_line_contains("pass"));
    TEST_ASSERT(!any_line_contains("HomeNet"));
}

/* Whether a packet of @p n bytes stalled after @p cut of them still lands: a
 * header cut does (its matched bytes survive the gap), and so does a cut after
 * the checksum, leaving only the newline; a cut in between is dropped. */
static bool delivered_across_a_stall(size_t cut, size_t n)
{
    return cut < 6 || cut >= n - 1;
}

/* No line holds a byte of the packet cut after @p cut bytes, and the console
 * still runs the next command. */
static void assert_cut_leaks_nothing(size_t cut)
{
    for (int i = 0; i < s_line_count; i++) {
        printf("\n    edge after byte %zu, line %d: \"%s\"", cut, i, s_lines[i]);
    }
    TEST_ASSERT(!any_line_contains("s3cr3tpass"));
    TEST_ASSERT(!any_line_contains("HomeNet"));
    TEST_ASSERT(!any_line_contains("PROV"));
    TEST_ASSERT(s_line_count == 0);
    pause(LATER_MS);
    feed_text("status\n");
    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
}

/* A disarm after every byte of a packet, the rest following in the same burst
 * (the link came up while ESP Web Tools was writing). Before the header is
 * complete the parser has claimed nothing, yet the packet is in flight: its
 * first bytes are in the line buffer, and resetting the parser there made the
 * rest of the packet text, two logged lines holding the SSID and the password.
 * The packet in flight is finished instead, from any cut. */
static void test_a_disarm_anywhere_in_a_packet_leaks_nothing(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    for (size_t cut = 1; cut < n; cut++) {
        reset();
        feed(pkt, cut, true);
        feed(pkt + cut, n - cut, false);
        TEST_ASSERT(s_creds_calls == 1); /* finished, and delivered */
        assert_cut_leaks_nothing(cut);
    }
}

/* The same disarm with the rest of the packet stalled past the burst gap: the
 * parser is reset and that burst dropped. A header cut keeps its matched bytes
 * across the gap, so the rest of the header completes it and the packet still
 * lands. */
static void test_a_disarm_with_the_rest_stalled_leaks_nothing(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    for (size_t cut = 1; cut < n; cut++) {
        reset();
        feed(pkt, cut, true);
        pause(LATER_MS);
        feed(pkt + cut, n - cut, false);
        TEST_ASSERT(s_creds_calls == (delivered_across_a_stall(cut, n) ? 1 : 0));
        assert_cut_leaks_nothing(cut);
    }
}

/* A disarm and a re-arm between two bytes of one packet, with the rest of it
 * arriving at least EDGE_PAIR_GAP_MS later (the two edges are two supervisor
 * steps apart). Assuming the rest never came, and dropping only up to the next
 * newline, logged the password from a cut inside the data and the whole packet
 * from a cut inside the header. */
static void test_a_disarm_and_re_arm_anywhere_in_a_packet_leaks_nothing(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    for (size_t cut = 1; cut < n; cut++) {
        reset();
        feed(pkt, cut, true);
        set_armed(false);
        set_armed(true);
        pause(EDGE_PAIR_GAP_MS);
        feed(pkt + cut, n - cut, true);
        TEST_ASSERT(s_creds_calls == (delivered_across_a_stall(cut, n) ? 1 : 0));
        assert_cut_leaks_nothing(cut);
    }
}

/* The same double edge with no gap, which the supervisor cannot produce but the
 * console does not rely on: the packet is simply finished. */
static void test_a_disarm_and_re_arm_inside_the_burst_finishes_the_packet(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    for (size_t cut = 1; cut < n; cut++) {
        reset();
        feed(pkt, cut, true);
        set_armed(false);
        set_armed(true);
        feed(pkt + cut, n - cut, true);
        TEST_ASSERT(s_creds_calls == 1);
        assert_cut_leaks_nothing(cut);
    }
}

/* Arming and disarming do not cost a half-typed command its first part. */
static void test_a_link_edge_keeps_a_half_typed_command(void)
{
    feed((const uint8_t *)"sta", 3, false);
    feed_text("tus\n");
    TEST_ASSERT(s_line_count == 1);
    TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
}

/* The session count's parity is the armed state, so one variable carries both
 * the state and every edge between two reads of it. */
static void test_odd_sessions_are_armed(void)
{
    TEST_ASSERT(!console_line_improv_armed(0));
    TEST_ASSERT(console_line_improv_armed(1));
    TEST_ASSERT(!console_line_improv_armed(2));
    TEST_ASSERT(console_line_improv_armed(UINT32_MAX));
}

/* feed() reads its first byte 1 ms after the pause, so pause(ms) leaves a gap of
 * ms + 1 between the last byte before it and the first one after. */

/* Issue #1103: a disarm cuts a packet inside its frame and the rest never
 * comes (the client gave up). After more than CONSOLE_LINE_PACKET_STALE_MS of
 * silence that packet is dead, so the next command, typed at the 1 ms a byte
 * a paste or a fast typist gives, runs instead of being dropped as its tail. */
static void test_a_command_after_a_stale_cut_packet_runs(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    for (size_t cut = 6; cut < n; cut++) { /* the header complete, the parser in the frame */
        reset();
        feed(pkt, cut, true);
        set_armed(false);
        pause(CONSOLE_LINE_PACKET_STALE_MS); /* a gap of the limit plus 1 ms */
        feed((const uint8_t *)"status\n", 7, false);
        /* A cut after the checksum leaves only the newline, so that one landed. */
        TEST_ASSERT(s_creds_calls == (delivered_across_a_stall(cut, n) ? 1 : 0));
        TEST_ASSERT(s_line_count == 1);
        TEST_ASSERT(strcmp(s_lines[0], "status") == 0);
    }
}

/* Up to the limit the burst after the gap is still the stalled tail and is
 * dropped: the stalled-rest test above, with the gap at exactly
 * CONSOLE_LINE_PACKET_STALE_MS. */
static void test_a_tail_within_the_stale_limit_is_still_dropped(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);

    for (size_t cut = 1; cut < n; cut++) {
        reset();
        feed(pkt, cut, true);
        pause(CONSOLE_LINE_PACKET_STALE_MS - 1); /* a gap of exactly the limit */
        feed(pkt + cut, n - cut, false);
        TEST_ASSERT(s_creds_calls == (delivered_across_a_stall(cut, n) ? 1 : 0));
        assert_cut_leaks_nothing(cut);
    }
}

/* Both rules across the uint32 millisecond wrap (day 49), with the cut packet's
 * last byte read before it and the next burst after it. Two placements, so that
 * neither now > last + limit nor now - limit > last, each wrong somewhere near
 * the wrap, can pass for the subtraction the burst gap uses. */
static void test_the_stale_limit_survives_the_millisecond_wrap(void)
{
    uint8_t pkt[128];
    uint8_t checksum = 0;
    const size_t n = build_credentials_packet(pkt, "HomeNet", "s3cr3tpass", &checksum);
    static const uint32_t before_wrap_ms[] = {6000u, 100u};

    for (size_t i = 0; i < sizeof(before_wrap_ms) / sizeof(before_wrap_ms[0]); i++) {
        /* Stale: the command runs. */
        reset();
        s_now_ms = UINT32_MAX - before_wrap_ms[i] - 14u;
        feed(pkt, 14, true);
        set_armed(false);
        pause(CONSOLE_LINE_PACKET_STALE_MS + 2000u);
        TEST_ASSERT(s_now_ms < CONSOLE_LINE_PACKET_STALE_MS + 2000u); /* wrapped */
        feed((const uint8_t *)"status\n", 7, false);
        TEST_ASSERT(s_line_count == 1);
        TEST_ASSERT(strcmp(s_lines[0], "status") == 0);

        /* Within the limit: the tail is dropped. */
        reset();
        s_now_ms = UINT32_MAX - before_wrap_ms[i] - 14u;
        feed(pkt, 14, true);
        pause(CONSOLE_LINE_PACKET_STALE_MS - 1);
        feed(pkt + 14, n - 14, false);
        TEST_ASSERT(s_creds_calls == 0);
        assert_cut_leaks_nothing(14);
    }
}

int main(void)
{
    printf("console_line tests\n");
    RUN_TEST(test_typed_commands_become_lines);
    RUN_TEST(test_credentials_packet_never_becomes_a_line);
    RUN_TEST(test_a_checksum_byte_is_not_a_movement_command);
    RUN_TEST(test_commands_around_a_packet);
    RUN_TEST(test_unarmed_bytes_are_plain_input);
    RUN_TEST(test_a_packet_cut_off_by_a_link_edge_does_not_leak_the_next_one);
    RUN_TEST(test_a_packet_in_flight_at_disarm_is_finished_not_logged);
    RUN_TEST(test_disarm_takes_effect_when_the_packet_in_flight_ends);
    RUN_TEST(test_a_packet_with_a_short_length_byte_leaks_nothing);
    RUN_TEST(test_a_rejected_packet_with_a_newline_in_its_tail_leaks_nothing);
    RUN_TEST(test_two_packets_in_one_burst_both_parse);
    RUN_TEST(test_a_new_packet_inside_the_burst_after_a_cut_leaks_nothing);
    RUN_TEST(test_a_disarm_anywhere_in_a_packet_leaks_nothing);
    RUN_TEST(test_a_disarm_with_the_rest_stalled_leaks_nothing);
    RUN_TEST(test_a_disarm_and_re_arm_anywhere_in_a_packet_leaks_nothing);
    RUN_TEST(test_a_disarm_and_re_arm_inside_the_burst_finishes_the_packet);
    RUN_TEST(test_a_link_edge_keeps_a_half_typed_command);
    RUN_TEST(test_odd_sessions_are_armed);
    RUN_TEST(test_a_command_after_a_stale_cut_packet_runs);
    RUN_TEST(test_a_tail_within_the_stale_limit_is_still_dropped);
    RUN_TEST(test_the_stale_limit_survives_the_millisecond_wrap);
    printf("%d/%d passed\n", test_pass, test_count);
    return test_pass == test_count ? 0 : 1;
}
