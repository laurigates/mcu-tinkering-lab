/**
 * @file test_credentials_loader.c
 *
 * Pins the MQTT broker-credential loader (issue #639), the Gemini key
 * loader (issue #845), the broker URI and the WiFi NVS reason (issue #1312). Its output decides
 * whether the MQTT command topic may change anything
 * (mqtt_command_access_for_credentials(), pinned in test_mqtt_command.c), so
 * what reaches that decision matters as much as the decision:
 *
 *  - a complete NVS pair wins over MQTT_USERNAME/MQTT_PASSWORD in credentials.h;
 *  - a half-written NVS pair falls back to credentials.h — never a username
 *    with no password;
 *  - credentials_nvs_save_mqtt() refuses an empty or over-long value instead of
 *    storing it.
 *
 * credentials_loader.c is compiled unmodified against the in-memory NVS in
 * nvs_shim.c, with credentials_fixture.h standing in for credentials.h. This file
 * is built twice: once with broker credentials in that fixture, and once
 * (CREDENTIALS_FIXTURE_NO_MQTT) shaped like the CMake stub, where every
 * fallback must come out as "none".
 *
 * The loader reads NVS once per boot and latches, on purpose: the MQTT client
 * is created once, so the mode it reports must be the mode it started in. So a
 * test never calls the getters in this process. It prepares NVS, then reads the
 * credentials inside in_fresh_boot(), which forks: the child inherits the store
 * but not a latch, because this process never set one. That keeps the module a
 * black box without a reset hook added just for tests.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include "credentials_loader.h"
#include "nvs_shim.h"

/* Namespace and keys as credentials_loader.c defines them. Duplicated on
 * purpose: a rename there would orphan every board's stored credentials, and
 * this is the place that notices. */
#define MQTT_NS "mqtt_auth"
#define MQTT_USER_KEY "user"
#define MQTT_PASS_KEY "pass"
#define WIFI_NS "wifi_config"
#define GEMINI_NS "gemini_auth"
#define GEMINI_KEY "key"
#define BROKER_NS "mqtt_broker"
#define BROKER_KEY "uri"
#define BUILD_BROKER "mqtt://192.168.0.100:1883"

#ifdef CREDENTIALS_FIXTURE_NO_MQTT
#define FALLBACK_USER NULL
#define FALLBACK_PASS NULL
#define FALLBACK_SOURCE "none"
#define FALLBACK_GEMINI NULL
#define FALLBACK_GEMINI_SOURCE "none"
#else
#define FALLBACK_USER "file-user"
#define FALLBACK_PASS "file-pass"
#define FALLBACK_SOURCE "credentials.h"
#define FALLBACK_GEMINI "file-gemini-key"
#define FALLBACK_GEMINI_SOURCE "credentials.h"
#endif

static int s_failures;

#define CHECK(cond, ...)                                      \
    do {                                                      \
        if (!(cond)) {                                        \
            fprintf(stderr, "  %s:%d: ", __FILE__, __LINE__); \
            fprintf(stderr, __VA_ARGS__);                     \
            fprintf(stderr, "\n");                            \
            s_failures++;                                     \
        }                                                     \
    } while (0)

static bool str_eq(const char *a, const char *b)
{
    return (a == NULL && b == NULL) || (a != NULL && b != NULL && strcmp(a, b) == 0);
}

static const char *show(const char *s)
{
    return s ? s : "(null)";
}

#define CHECK_STR(actual, expected)                                                         \
    do {                                                                                    \
        const char *_a = (actual);                                                          \
        const char *_e = (expected);                                                        \
        CHECK(str_eq(_a, _e), "%s = \"%s\", expected \"%s\"", #actual, show(_a), show(_e)); \
    } while (0)

/* ---- one boot ------------------------------------------------------------ */

/* What the child of in_fresh_boot() is expected to read. */
static const char *s_want_user;
static const char *s_want_pass;
static const char *s_want_source;

static void check_boot_reads_expected(void)
{
    CHECK_STR(get_mqtt_username(), s_want_user);
    CHECK_STR(get_mqtt_password(), s_want_pass);
    CHECK_STR(get_mqtt_credentials_source(), s_want_source);
    /* Every read path must close what it opened. */
    CHECK(nvs_shim_open_handles() == 0, "%d NVS handle(s) left open", nvs_shim_open_handles());
}

/* Run @p body as a fresh boot: a forked child with the current NVS contents and
 * no loader latch. Its CHECK failures become this process's failures. */
static void in_fresh_boot(void (*body)(void))
{
    fflush(stdout);
    fflush(stderr);
    const pid_t pid = fork();
    if (pid < 0) {
        perror("fork");
        exit(EXIT_FAILURE);
    }
    if (pid == 0) {
        s_failures = 0;
        body();
        fflush(stderr);
        _exit(s_failures == 0 ? 0 : 1);
    }
    int status = 0;
    if (waitpid(pid, &status, 0) != pid || !WIFEXITED(status)) {
        fprintf(stderr, "  boot child did not exit cleanly (status %d)\n", status);
        s_failures++;
        return;
    }
    if (WEXITSTATUS(status) != 0) {
        s_failures++;
    }
}

static void expect_boot(const char *user, const char *pass, const char *source)
{
    s_want_user = user;
    s_want_pass = pass;
    s_want_source = source;
    in_fresh_boot(check_boot_reads_expected);
}

static void expect_fallback(void)
{
    expect_boot(FALLBACK_USER, FALLBACK_PASS, FALLBACK_SOURCE);
}

/* ---- loading ------------------------------------------------------------- */

static void test_empty_nvs_falls_back_to_credentials_h(void)
{
    expect_fallback();
}

static void test_a_complete_nvs_pair_wins_over_credentials_h(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "nvs-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    expect_boot("nvs-user", "nvs-pass", "nvs");
}

static void test_a_username_with_no_password_key_falls_back(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "nvs-user");
    expect_fallback();
}

static void test_a_password_with_no_username_key_falls_back(void)
{
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    expect_fallback();
}

/* Both keys present, one of them "". nvs_get_str() succeeds on both, so only
 * the both-non-empty condition stands between this and a lone username — the
 * shape a hand-written or interrupted entry leaves behind. */
static void test_an_empty_nvs_password_falls_back(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "nvs-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "");
    expect_fallback();
}

static void test_an_empty_nvs_username_falls_back(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    expect_fallback();
}

/* A value too long for the loader's buffer is refused by nvs_get_str(), not
 * truncated into a wrong credential that looks configured. */
static void test_an_over_long_nvs_value_falls_back(void)
{
    char user[MAX_MQTT_USERNAME_LENGTH + 1];
    memset(user, 'u', sizeof(user) - 1);
    user[sizeof(user) - 1] = '\0'; /* 33 characters: one more than fits */
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, user);
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    expect_fallback();
}

static void test_an_nvs_that_will_not_open_falls_back(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "nvs-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    nvs_shim_fail_open(ESP_FAIL);
    expect_fallback();
}

/* WiFi credentials live in their own namespace; they are not broker credentials. */
static void test_wifi_credentials_are_not_read_as_broker_credentials(void)
{
    nvs_shim_put(WIFI_NS, "ssid", "home");
    nvs_shim_put(WIFI_NS, "password", "wifi-secret");
    nvs_shim_put(WIFI_NS, MQTT_USER_KEY, "nvs-user");
    nvs_shim_put(WIFI_NS, MQTT_PASS_KEY, "nvs-pass");
    expect_fallback();
}

static void read_then_change_nvs_then_read_again(void)
{
    CHECK_STR(get_mqtt_username(), "nvs-user");
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "changed-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "changed-pass");
    CHECK_STR(get_mqtt_username(), "nvs-user");
    CHECK_STR(get_mqtt_password(), "nvs-pass");
    CHECK_STR(get_mqtt_credentials_source(), "nvs");
}

/* The client is created once per boot; the mode reported must stay the mode it
 * was started in, even after `mqtt auth` writes new credentials. */
static void test_credentials_are_read_once_per_boot(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "nvs-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    in_fresh_boot(read_then_change_nvs_then_read_again);
}

/* ---- saving and clearing ------------------------------------------------- */

static void check_save_refused(const char *user, const char *pass, const char *what)
{
    const int writes = nvs_shim_write_count();
    CHECK(!credentials_nvs_save_mqtt(user, pass), "save accepted %s", what);
    CHECK(nvs_shim_write_count() == writes, "refused save of %s still wrote NVS", what);
    CHECK(nvs_shim_open_handles() == 0, "refused save of %s left an NVS handle open", what);
}

static void test_save_refuses_a_missing_or_empty_value(void)
{
    check_save_refused(NULL, "nvs-pass", "a NULL username");
    check_save_refused("nvs-user", NULL, "a NULL password");
    check_save_refused("", "nvs-pass", "an empty username");
    check_save_refused("nvs-user", "", "an empty password");
    expect_fallback();
}

static void test_save_refuses_a_value_that_does_not_fit(void)
{
    char user_33[MAX_MQTT_USERNAME_LENGTH + 1];
    memset(user_33, 'u', sizeof(user_33) - 1);
    user_33[sizeof(user_33) - 1] = '\0'; /* 33 characters: one more than fits */

    char pass_65[MAX_MQTT_PASSWORD_LENGTH + 1];
    memset(pass_65, 'p', sizeof(pass_65) - 1);
    pass_65[sizeof(pass_65) - 1] = '\0'; /* 65 characters: one more than fits */

    check_save_refused(user_33, "nvs-pass", "a 33-character username");
    check_save_refused("nvs-user", pass_65, "a 65-character password");
    expect_fallback();
}

/* A refused save must not disturb a pair that is already stored. */
static void test_a_refused_save_keeps_the_stored_pair(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "nvs-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    check_save_refused("new-user", "", "an empty password over a stored pair");
    expect_boot("nvs-user", "nvs-pass", "nvs");
}

static void test_save_accepts_the_longest_values_that_fit(void)
{
    char user_32[MAX_MQTT_USERNAME_LENGTH];
    memset(user_32, 'u', sizeof(user_32) - 1);
    user_32[sizeof(user_32) - 1] = '\0';
    char pass_64[MAX_MQTT_PASSWORD_LENGTH];
    memset(pass_64, 'p', sizeof(pass_64) - 1);
    pass_64[sizeof(pass_64) - 1] = '\0';

    CHECK(credentials_nvs_save_mqtt(user_32, pass_64), "save refused 32/64-character values");
    CHECK(nvs_shim_open_handles() == 0, "save left an NVS handle open");
    expect_boot(user_32, pass_64, "nvs");
}

static void test_saved_credentials_apply_at_the_next_boot(void)
{
    CHECK(credentials_nvs_save_mqtt("saved-user", "saved-pass"), "save refused valid values");
    CHECK_STR(nvs_shim_get(MQTT_NS, MQTT_USER_KEY), "saved-user");
    CHECK_STR(nvs_shim_get(MQTT_NS, MQTT_PASS_KEY), "saved-pass");
    expect_boot("saved-user", "saved-pass", "nvs");
}

/* `mqtt auth` on a board that already has broker credentials: the new pair
 * replaces both stored values rather than being refused or half-applied. */
static void test_save_replaces_a_stored_pair(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "old-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "old-pass");
    CHECK(credentials_nvs_save_mqtt("new-user", "new-pass"), "save refused to replace a pair");
    expect_boot("new-user", "new-pass", "nvs");
}

static void test_save_fails_when_nvs_will_not_open(void)
{
    nvs_shim_fail_open(ESP_FAIL);
    CHECK(!credentials_nvs_save_mqtt("saved-user", "saved-pass"),
          "save reported success with NVS unavailable");
}

static void test_clear_falls_back_to_credentials_h_at_the_next_boot(void)
{
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "nvs-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    nvs_shim_put(WIFI_NS, "ssid", "home");

    CHECK(credentials_nvs_clear_mqtt(), "clear failed");
    CHECK(nvs_shim_open_handles() == 0, "clear left an NVS handle open");
    CHECK_STR(nvs_shim_get(MQTT_NS, MQTT_USER_KEY), NULL);
    CHECK_STR(nvs_shim_get(MQTT_NS, MQTT_PASS_KEY), NULL);
    /* Clearing broker credentials must not cost the board its WiFi. */
    CHECK_STR(nvs_shim_get(WIFI_NS, "ssid"), "home");
    expect_fallback();
}

static void test_clear_succeeds_with_nothing_stored(void)
{
    CHECK(credentials_nvs_clear_mqtt(), "clear failed on an empty store");
    expect_fallback();
}

static void test_clear_fails_when_nvs_will_not_open(void)
{
    nvs_shim_fail_open(ESP_FAIL);
    CHECK(!credentials_nvs_clear_mqtt(), "clear reported success with NVS unavailable");
}

/* ---- Gemini API key (issue #845) ----------------------------------------- */

static const char *s_want_key;
static const char *s_want_key_source;

static void check_boot_reads_gemini_key(void)
{
    CHECK_STR(get_gemini_api_key(), s_want_key);
    CHECK_STR(get_gemini_api_key_source(), s_want_key_source);
    CHECK(nvs_shim_open_handles() == 0, "%d NVS handle(s) left open", nvs_shim_open_handles());
}

static void expect_gemini_boot(const char *key, const char *source)
{
    s_want_key = key;
    s_want_key_source = source;
    in_fresh_boot(check_boot_reads_gemini_key);
}

static void test_gemini_key_falls_back_to_credentials_h(void)
{
    expect_gemini_boot(FALLBACK_GEMINI, FALLBACK_GEMINI_SOURCE);
}

/* The bug: once Improv had written WiFi credentials to NVS, the key was read
 * from getenv() alone, so a key compiled into credentials.h was dropped. */
static void test_improv_wifi_does_not_drop_the_gemini_key(void)
{
    nvs_shim_put(WIFI_NS, "ssid", "home");
    nvs_shim_put(WIFI_NS, "password", "secret");
    expect_gemini_boot(FALLBACK_GEMINI, FALLBACK_GEMINI_SOURCE);
}

static void test_nvs_gemini_key_wins(void)
{
    nvs_shim_put(GEMINI_NS, GEMINI_KEY, "nvs-gemini-key");
    nvs_shim_put(WIFI_NS, "ssid", "home");
    nvs_shim_put(WIFI_NS, "password", "secret");
    expect_gemini_boot("nvs-gemini-key", "nvs");
}

static void test_a_malformed_nvs_gemini_key_falls_back(void)
{
    nvs_shim_put(GEMINI_NS, GEMINI_KEY, "has a space");
    expect_gemini_boot(FALLBACK_GEMINI, FALLBACK_GEMINI_SOURCE);
    nvs_shim_put(GEMINI_NS, GEMINI_KEY, "");
    expect_gemini_boot(FALLBACK_GEMINI, FALLBACK_GEMINI_SOURCE);
}

static void check_env_key(void)
{
    setenv("GEMINI_API_KEY", "env-gemini-key", 1);
    check_boot_reads_gemini_key();
}

static void test_environment_key_is_the_last_source(void)
{
#ifdef CREDENTIALS_FIXTURE_NO_MQTT
    s_want_key = "env-gemini-key";
    s_want_key_source = "env";
#else
    s_want_key = "file-gemini-key"; /* credentials.h outranks the environment */
    s_want_key_source = "credentials.h";
#endif
    in_fresh_boot(check_env_key);
}

/* `gemini auth <key>` applies at once, in the running boot, and survives a
 * reboot; Improv provisioning afterwards does not touch it. */
static void check_save_applies_now(void)
{
    (void)get_gemini_api_key(); /* loaded before the save, as on a running board */
    CHECK(credentials_nvs_save_gemini("saved-gemini-key"), "save refused a valid key");
    CHECK_STR(get_gemini_api_key(), "saved-gemini-key");
    CHECK_STR(get_gemini_api_key_source(), "nvs");
    CHECK(credentials_nvs_save_wifi("home", "secret"), "WiFi save failed");
    CHECK(credentials_reload(), "WiFi reload failed");
    CHECK_STR(get_gemini_api_key(), "saved-gemini-key");
}

static void test_saved_gemini_key_applies_now_and_survives(void)
{
    in_fresh_boot(check_save_applies_now);
    /* The child's writes do not reach this process's store, so save here too
     * and boot again. */
    CHECK(credentials_nvs_save_gemini("saved-gemini-key"), "save refused a valid key");
    CHECK_STR(nvs_shim_get(GEMINI_NS, GEMINI_KEY), "saved-gemini-key");
    expect_gemini_boot("saved-gemini-key", "nvs");
}

/* A key old enough to be read before a change must still be intact after it:
 * a request on another task may be sending it. */
static void check_old_pointer_survives_one_change(void)
{
    CHECK(credentials_nvs_save_gemini("first-gemini-key"), "save failed");
    const char *before = get_gemini_api_key();
    CHECK(credentials_nvs_save_gemini("second-gemini-key"), "save failed");
    CHECK_STR(before, "first-gemini-key");
    CHECK_STR(get_gemini_api_key(), "second-gemini-key");
}

static void test_a_key_change_never_rewrites_a_key_in_use(void)
{
    in_fresh_boot(check_old_pointer_survives_one_change);
}

static void test_gemini_save_refuses_bad_keys(void)
{
    char too_long[MAX_API_KEY_LENGTH + 1];
    memset(too_long, 'k', sizeof(too_long) - 1);
    too_long[sizeof(too_long) - 1] = '\0';
    CHECK(!credentials_nvs_save_gemini(NULL), "saved NULL");
    CHECK(!credentials_nvs_save_gemini(""), "saved an empty key");
    CHECK(!credentials_nvs_save_gemini("two words"), "saved a key with a space");
    CHECK(!credentials_nvs_save_gemini("abc\r\nX-Evil: 1"), "saved a key with CR/LF");
    CHECK(!credentials_nvs_save_gemini(too_long), "saved an over-long key");
    too_long[MAX_API_KEY_LENGTH - 1] = '\0';
    CHECK(credentials_nvs_save_gemini(too_long), "refused the longest key that fits");
}

static void check_clear_applies_now(void)
{
    CHECK(credentials_nvs_save_gemini("saved-gemini-key"), "save failed");
    CHECK(credentials_nvs_clear_gemini(), "clear failed");
    CHECK_STR(get_gemini_api_key(), FALLBACK_GEMINI);
    CHECK_STR(get_gemini_api_key_source(), FALLBACK_GEMINI_SOURCE);
}

static void test_gemini_clear_falls_back_now(void)
{
    nvs_shim_put(WIFI_NS, "ssid", "home");
    in_fresh_boot(check_clear_applies_now);
    nvs_shim_put(GEMINI_NS, GEMINI_KEY, "nvs-gemini-key");
    CHECK(credentials_nvs_clear_gemini(), "clear failed");
    CHECK_STR(nvs_shim_get(GEMINI_NS, GEMINI_KEY), NULL);
    CHECK_STR(nvs_shim_get(WIFI_NS, "ssid"), "home"); /* WiFi untouched */
}

/* ---- MQTT broker URI (issue #1312) --------------------------------------- */

/* A release is built with the CMake stub, so before this the broker was the
 * compiled default whatever the network: a released or OTA image could not
 * reach a broker on another host or port. `mqtt broker <uri>` stores one in
 * NVS, read before the default, in its own namespace so `mqtt auth clear`
 * (which erases the mqtt_auth namespace) leaves it alone. */

static const char *s_want_broker;
static const char *s_want_broker_source;

static void check_boot_reads_broker(void)
{
    CHECK_STR(get_mqtt_broker_uri(BUILD_BROKER), s_want_broker);
    CHECK_STR(get_mqtt_broker_uri_source(), s_want_broker_source);
    CHECK(nvs_shim_open_handles() == 0, "%d NVS handle(s) left open", nvs_shim_open_handles());
}

static void expect_broker_boot(const char *uri, const char *source)
{
    s_want_broker = uri;
    s_want_broker_source = source;
    in_fresh_boot(check_boot_reads_broker);
}

static void test_broker_falls_back_to_the_build_default(void)
{
    expect_broker_boot(BUILD_BROKER, "build");
}

static void test_nvs_broker_wins(void)
{
    nvs_shim_put(BROKER_NS, BROKER_KEY, "mqtt://192.168.0.106:11883");
    expect_broker_boot("mqtt://192.168.0.106:11883", "nvs");
}

/* A stored value the save path would refuse is ignored, not used. */
static void test_a_malformed_nvs_broker_falls_back(void)
{
    nvs_shim_put(BROKER_NS, BROKER_KEY, "192.168.0.106:1883");
    expect_broker_boot(BUILD_BROKER, "build");
    nvs_shim_put(BROKER_NS, BROKER_KEY, "");
    expect_broker_boot(BUILD_BROKER, "build");
    nvs_shim_put(BROKER_NS, BROKER_KEY, "mqtt://a b");
    expect_broker_boot(BUILD_BROKER, "build");
}

static void test_a_broker_nvs_that_will_not_open_falls_back(void)
{
    nvs_shim_put(BROKER_NS, BROKER_KEY, "mqtt://10.0.0.2:1883");
    nvs_shim_fail_open(ESP_FAIL);
    expect_broker_boot(BUILD_BROKER, "build");
}

/* The MQTT client is created once per boot with the URI it was given; what
 * `mqtt` reports must stay that URI after `mqtt broker` stores a new one. */
static void read_broker_then_save_then_read_again(void)
{
    CHECK_STR(get_mqtt_broker_uri(BUILD_BROKER), "mqtt://10.0.0.2:1883");
    CHECK(credentials_nvs_save_mqtt_broker("mqtt://10.0.0.3:1883"), "save refused a valid URI");
    CHECK_STR(get_mqtt_broker_uri(BUILD_BROKER), "mqtt://10.0.0.2:1883");
    CHECK_STR(get_mqtt_broker_uri_source(), "nvs");
}

static void test_broker_is_read_once_per_boot(void)
{
    nvs_shim_put(BROKER_NS, BROKER_KEY, "mqtt://10.0.0.2:1883");
    in_fresh_boot(read_broker_then_save_then_read_again);
}

static void test_broker_uri_validation(void)
{
    char longest[MAX_MQTT_BROKER_URI_LENGTH];
    const size_t scheme = strlen("mqtt://");
    memcpy(longest, "mqtt://", scheme);
    memset(longest + scheme, 'h', sizeof(longest) - 1 - scheme);
    longest[sizeof(longest) - 1] = '\0';
    char too_long[MAX_MQTT_BROKER_URI_LENGTH + 1];
    memcpy(too_long, longest, sizeof(longest) - 1);
    too_long[sizeof(too_long) - 2] = 'h';
    too_long[sizeof(too_long) - 1] = '\0';

    CHECK(credentials_mqtt_broker_uri_valid("mqtt://192.168.0.106:11883"), "refused mqtt://");
    CHECK(credentials_mqtt_broker_uri_valid("mqtts://broker.lan:8883"), "refused mqtts://");
    CHECK(credentials_mqtt_broker_uri_valid("ws://broker.lan/mqtt"), "refused ws://");
    CHECK(credentials_mqtt_broker_uri_valid("wss://broker.lan/mqtt"), "refused wss://");
    CHECK(credentials_mqtt_broker_uri_valid(longest), "refused the longest URI that fits");
    CHECK(!credentials_mqtt_broker_uri_valid(NULL), "accepted NULL");
    CHECK(!credentials_mqtt_broker_uri_valid(""), "accepted an empty URI");
    CHECK(!credentials_mqtt_broker_uri_valid("mqtt://"), "accepted a scheme with no host");
    CHECK(!credentials_mqtt_broker_uri_valid("192.168.0.106:1883"), "accepted no scheme");
    CHECK(!credentials_mqtt_broker_uri_valid("http://broker.lan"), "accepted http://");
    CHECK(!credentials_mqtt_broker_uri_valid("mqtt://a b"), "accepted a space");
    CHECK(!credentials_mqtt_broker_uri_valid("mqtt://a\r\n"), "accepted CR/LF");
    CHECK(!credentials_mqtt_broker_uri_valid(too_long), "accepted an over-long URI");
}

static void test_broker_save_refuses_an_invalid_uri(void)
{
    nvs_shim_put(BROKER_NS, BROKER_KEY, "mqtt://10.0.0.2:1883");
    const int writes = nvs_shim_write_count();
    CHECK(!credentials_nvs_save_mqtt_broker("not-a-uri"), "save accepted an invalid URI");
    CHECK(nvs_shim_write_count() == writes, "refused save still wrote NVS");
    CHECK(nvs_shim_open_handles() == 0, "refused save left an NVS handle open");
    expect_broker_boot("mqtt://10.0.0.2:1883", "nvs");
}

static void test_saved_broker_applies_at_the_next_boot(void)
{
    CHECK(credentials_nvs_save_mqtt_broker("mqtt://192.168.0.106:11883"), "save refused");
    CHECK(nvs_shim_open_handles() == 0, "save left an NVS handle open");
    CHECK_STR(nvs_shim_get(BROKER_NS, BROKER_KEY), "mqtt://192.168.0.106:11883");
    expect_broker_boot("mqtt://192.168.0.106:11883", "nvs");
}

static void test_broker_save_fails_when_nvs_will_not_open(void)
{
    nvs_shim_fail_open(ESP_FAIL);
    CHECK(!credentials_nvs_save_mqtt_broker("mqtt://10.0.0.2:1883"),
          "save reported success with NVS unavailable");
}

static void test_broker_clear_falls_back_and_keeps_the_rest(void)
{
    nvs_shim_put(BROKER_NS, BROKER_KEY, "mqtt://10.0.0.2:1883");
    nvs_shim_put(MQTT_NS, MQTT_USER_KEY, "nvs-user");
    nvs_shim_put(MQTT_NS, MQTT_PASS_KEY, "nvs-pass");
    CHECK(credentials_nvs_clear_mqtt_broker(), "clear failed");
    CHECK(nvs_shim_open_handles() == 0, "clear left an NVS handle open");
    CHECK_STR(nvs_shim_get(BROKER_NS, BROKER_KEY), NULL);
    CHECK_STR(nvs_shim_get(MQTT_NS, MQTT_USER_KEY), "nvs-user");
    expect_broker_boot(BUILD_BROKER, "build");
}

/* `mqtt auth clear` must not cost the board its broker. */
static void test_mqtt_auth_clear_keeps_the_broker(void)
{
    nvs_shim_put(BROKER_NS, BROKER_KEY, "mqtt://10.0.0.2:1883");
    CHECK(credentials_nvs_clear_mqtt(), "clear failed");
    expect_broker_boot("mqtt://10.0.0.2:1883", "nvs");
}

/* ---- why WiFi did not load from NVS (issue #1312) ------------------------ */

/* load_from_nvs() returned false with no log line, so a missing namespace, a
 * missing key and NVS not yet initialised looked the same. Each now has a
 * stable reason, logged at WARN when it changes. */
static const char *s_want_reason;

static void check_wifi_nvs_reason(void)
{
    (void)are_credentials_available();
    CHECK_STR(credentials_wifi_nvs_reason(), s_want_reason);
    CHECK(nvs_shim_open_handles() == 0, "%d NVS handle(s) left open", nvs_shim_open_handles());
}

static void expect_wifi_nvs_reason(const char *reason)
{
    s_want_reason = reason;
    in_fresh_boot(check_wifi_nvs_reason);
}

static void test_wifi_nvs_reasons(void)
{
    expect_wifi_nvs_reason("no_namespace");

    nvs_shim_put(WIFI_NS, "password", "secret");
    expect_wifi_nvs_reason("no_ssid");

    nvs_shim_reset();
    nvs_shim_put(WIFI_NS, "ssid", "home");
    expect_wifi_nvs_reason("no_password");

    nvs_shim_put(WIFI_NS, "ssid", "");
    nvs_shim_put(WIFI_NS, "password", "secret");
    expect_wifi_nvs_reason("empty_ssid");

    char ssid[MAX_SSID_LENGTH + 1];
    memset(ssid, 's', sizeof(ssid) - 1);
    ssid[sizeof(ssid) - 1] = '\0';
    nvs_shim_put(WIFI_NS, "ssid", ssid);
    expect_wifi_nvs_reason("too_long");

    nvs_shim_put(WIFI_NS, "ssid", "home");
    expect_wifi_nvs_reason("loaded");

    nvs_shim_fail_open(ESP_FAIL);
    expect_wifi_nvs_reason("open_failed");
}

int main(void)
{
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"empty_nvs_falls_back_to_credentials_h", test_empty_nvs_falls_back_to_credentials_h},
        {"a_complete_nvs_pair_wins_over_credentials_h",
         test_a_complete_nvs_pair_wins_over_credentials_h},
        {"a_username_with_no_password_key_falls_back",
         test_a_username_with_no_password_key_falls_back},
        {"a_password_with_no_username_key_falls_back",
         test_a_password_with_no_username_key_falls_back},
        {"an_empty_nvs_password_falls_back", test_an_empty_nvs_password_falls_back},
        {"an_empty_nvs_username_falls_back", test_an_empty_nvs_username_falls_back},
        {"an_over_long_nvs_value_falls_back", test_an_over_long_nvs_value_falls_back},
        {"an_nvs_that_will_not_open_falls_back", test_an_nvs_that_will_not_open_falls_back},
        {"wifi_credentials_are_not_read_as_broker_credentials",
         test_wifi_credentials_are_not_read_as_broker_credentials},
        {"credentials_are_read_once_per_boot", test_credentials_are_read_once_per_boot},
        {"save_refuses_a_missing_or_empty_value", test_save_refuses_a_missing_or_empty_value},
        {"save_refuses_a_value_that_does_not_fit", test_save_refuses_a_value_that_does_not_fit},
        {"a_refused_save_keeps_the_stored_pair", test_a_refused_save_keeps_the_stored_pair},
        {"save_accepts_the_longest_values_that_fit", test_save_accepts_the_longest_values_that_fit},
        {"saved_credentials_apply_at_the_next_boot", test_saved_credentials_apply_at_the_next_boot},
        {"save_replaces_a_stored_pair", test_save_replaces_a_stored_pair},
        {"save_fails_when_nvs_will_not_open", test_save_fails_when_nvs_will_not_open},
        {"clear_falls_back_to_credentials_h_at_the_next_boot",
         test_clear_falls_back_to_credentials_h_at_the_next_boot},
        {"clear_succeeds_with_nothing_stored", test_clear_succeeds_with_nothing_stored},
        {"clear_fails_when_nvs_will_not_open", test_clear_fails_when_nvs_will_not_open},
        {"gemini_key_falls_back_to_credentials_h", test_gemini_key_falls_back_to_credentials_h},
        {"improv_wifi_does_not_drop_the_gemini_key", test_improv_wifi_does_not_drop_the_gemini_key},
        {"nvs_gemini_key_wins", test_nvs_gemini_key_wins},
        {"a_malformed_nvs_gemini_key_falls_back", test_a_malformed_nvs_gemini_key_falls_back},
        {"environment_key_is_the_last_source", test_environment_key_is_the_last_source},
        {"saved_gemini_key_applies_now_and_survives",
         test_saved_gemini_key_applies_now_and_survives},
        {"a_key_change_never_rewrites_a_key_in_use", test_a_key_change_never_rewrites_a_key_in_use},
        {"gemini_save_refuses_bad_keys", test_gemini_save_refuses_bad_keys},
        {"gemini_clear_falls_back_now", test_gemini_clear_falls_back_now},
        {"broker_falls_back_to_the_build_default", test_broker_falls_back_to_the_build_default},
        {"nvs_broker_wins", test_nvs_broker_wins},
        {"a_malformed_nvs_broker_falls_back", test_a_malformed_nvs_broker_falls_back},
        {"a_broker_nvs_that_will_not_open_falls_back",
         test_a_broker_nvs_that_will_not_open_falls_back},
        {"broker_is_read_once_per_boot", test_broker_is_read_once_per_boot},
        {"broker_uri_validation", test_broker_uri_validation},
        {"broker_save_refuses_an_invalid_uri", test_broker_save_refuses_an_invalid_uri},
        {"saved_broker_applies_at_the_next_boot", test_saved_broker_applies_at_the_next_boot},
        {"broker_save_fails_when_nvs_will_not_open", test_broker_save_fails_when_nvs_will_not_open},
        {"broker_clear_falls_back_and_keeps_the_rest",
         test_broker_clear_falls_back_and_keeps_the_rest},
        {"mqtt_auth_clear_keeps_the_broker", test_mqtt_auth_clear_keeps_the_broker},
        {"wifi_nvs_reasons", test_wifi_nvs_reasons},
    };
#ifdef CREDENTIALS_FIXTURE_NO_MQTT
    printf("credentials.h fixture: no broker credentials (CMake stub shape)\n");
#else
    printf("credentials.h fixture: broker credentials compiled in\n");
#endif
    /* The loader's last Gemini key source is the environment, and a developer
     * may export GEMINI_API_KEY for other tools. Every case here expects it
     * unset except environment_key_is_the_last_source, which sets it in its
     * own boot child (issue #1088; ctest also runs this with it set). */
    if (unsetenv("GEMINI_API_KEY") != 0) {
        perror("unsetenv GEMINI_API_KEY");
        return EXIT_FAILURE;
    }
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        nvs_shim_reset();
        int before = s_failures;
        tests[i].fn();
        printf("%s %s\n", s_failures == before ? "PASS" : "FAIL", tests[i].name);
    }
    printf("%d failure(s)\n", s_failures);
    return s_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
