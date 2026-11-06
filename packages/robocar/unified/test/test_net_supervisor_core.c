/**
 * @file test_net_supervisor_core.c
 * @brief When MQTT, OTA and Improv run, from link edges (issue #846).
 *
 * The cases are the ones a bench stages only slowly or not at all: a board
 * that boots with the router off and gets it back an hour later, a link that
 * flaps, the uint32 millisecond wrap at day 49.
 */

#include "net_supervisor_core.h"

#include <stdio.h>
#include <stdlib.h>

static int test_count = 0;

#define ASSERT(cond)                                                         \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            abort();                                                         \
        }                                                                    \
    } while (0)

static void test_run(const char *name, void (*fn)(void))
{
    test_count++;
    printf("[%d] %s\n", test_count, name);
    fn();
}

static bool none(net_actions_t a)
{
    return !a.start_ota && !a.start_mqtt && !a.stop_mqtt && !a.arm_improv && !a.disarm_improv;
}

/* Acceptance: a board that boots with its router off reaches MQTT and OTA
 * once the router comes back, with no reboot. */
static void test_offline_boot_starts_services_when_the_link_returns(void)
{
    net_supervisor_t s;
    net_supervisor_init(&s, true, 0);

    /* An hour offline, stepped at 500 ms like the glue task. */
    int ota = 0, mqtt = 0;
    for (uint32_t t = 0; t < 3600000u; t += 500u) {
        const net_actions_t a = net_supervisor_step(&s, false, t);
        ota += a.start_ota;
        mqtt += a.start_mqtt;
    }
    ASSERT(ota == 0 && mqtt == 0);

    const net_actions_t up = net_supervisor_step(&s, true, 3600000u);
    ASSERT(up.start_ota);
    ASSERT(up.start_mqtt);
    ASSERT(up.disarm_improv); /* it armed while offline */
}

static void test_ota_starts_once_per_boot(void)
{
    net_supervisor_t s;
    net_supervisor_init(&s, true, 0);
    ASSERT(net_supervisor_step(&s, true, 10).start_ota);
    ASSERT(!net_supervisor_step(&s, false, 20).start_ota);
    ASSERT(!net_supervisor_step(&s, true, 30).start_ota);
}

static void test_mqtt_follows_every_edge(void)
{
    net_supervisor_t s;
    net_supervisor_init(&s, true, 0);
    ASSERT(net_supervisor_step(&s, true, 10).start_mqtt);
    ASSERT(none(net_supervisor_step(&s, true, 20)));
    const net_actions_t down = net_supervisor_step(&s, false, 30);
    ASSERT(down.stop_mqtt && !down.start_mqtt);
    ASSERT(none(net_supervisor_step(&s, false, 40)));
    const net_actions_t up = net_supervisor_step(&s, true, 50);
    ASSERT(up.start_mqtt && !up.stop_mqtt && !up.start_ota);
}

static void test_improv_waits_for_a_long_outage(void)
{
    net_supervisor_t s;
    net_supervisor_init(&s, true, 0);
    ASSERT(net_supervisor_step(&s, true, 0).start_mqtt);

    /* A router reboot shorter than the delay never opens provisioning. */
    net_supervisor_step(&s, false, 1000);
    ASSERT(!net_supervisor_step(&s, false, 1000 + NET_IMPROV_REARM_MS - 1).arm_improv);
    ASSERT(!net_supervisor_step(&s, true, 1000 + NET_IMPROV_REARM_MS - 1).disarm_improv);

    /* A longer one does, and reconnecting closes it. */
    net_supervisor_step(&s, false, 200000);
    ASSERT(net_supervisor_step(&s, false, 200000 + NET_IMPROV_REARM_MS).arm_improv);
    ASSERT(none(net_supervisor_step(&s, false, 200000 + NET_IMPROV_REARM_MS + 500)));
    ASSERT(net_supervisor_step(&s, true, 400000).disarm_improv);
}

static void test_no_credentials_arms_improv_at_once(void)
{
    net_supervisor_t s;
    net_supervisor_init(&s, false, 0);
    ASSERT(net_supervisor_step(&s, false, 0).arm_improv);
    ASSERT(net_supervisor_step(&s, true, 5000).disarm_improv);

    /* Provisioned now: the next outage waits out the delay. */
    net_supervisor_set_credentials(&s, true);
    net_supervisor_step(&s, false, 6000);
    ASSERT(!net_supervisor_step(&s, false, 7000).arm_improv);
}

static void test_boot_with_credentials_and_no_router_arms_after_the_delay(void)
{
    net_supervisor_t s;
    net_supervisor_init(&s, true, 100);
    ASSERT(!net_supervisor_step(&s, false, 100).arm_improv);
    ASSERT(net_supervisor_step(&s, false, 100 + NET_IMPROV_REARM_MS).arm_improv);
}

static void test_the_delay_survives_the_millisecond_wrap(void)
{
    const uint32_t near_wrap = UINT32_MAX - 1000u;
    net_supervisor_t s;
    net_supervisor_init(&s, true, near_wrap);
    net_supervisor_step(&s, true, near_wrap);
    net_supervisor_step(&s, false, near_wrap);
    /* 2 s later the clock has wrapped; a signed or naive compare would arm. */
    ASSERT(!net_supervisor_step(&s, false, near_wrap + 2000u).arm_improv);
    ASSERT(net_supervisor_step(&s, false, near_wrap + NET_IMPROV_REARM_MS).arm_improv);
}

static void test_uninitialised_does_nothing(void)
{
    net_supervisor_t s = {0};
    ASSERT(none(net_supervisor_step(&s, true, 0)));
}

int main(void)
{
    test_run("offline_boot_starts_services_when_the_link_returns",
             test_offline_boot_starts_services_when_the_link_returns);
    test_run("ota_starts_once_per_boot", test_ota_starts_once_per_boot);
    test_run("mqtt_follows_every_edge", test_mqtt_follows_every_edge);
    test_run("improv_waits_for_a_long_outage", test_improv_waits_for_a_long_outage);
    test_run("no_credentials_arms_improv_at_once", test_no_credentials_arms_improv_at_once);
    test_run("boot_with_credentials_and_no_router_arms_after_the_delay",
             test_boot_with_credentials_and_no_router_arms_after_the_delay);
    test_run("the_delay_survives_the_millisecond_wrap",
             test_the_delay_survives_the_millisecond_wrap);
    test_run("uninitialised_does_nothing", test_uninitialised_does_nothing);
    printf("%d tests passed\n", test_count);
    return 0;
}
