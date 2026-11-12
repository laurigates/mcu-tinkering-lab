/**
 * @file net_supervisor_core.h
 * @brief When the network services run, decided from link edges (issue #846).
 *
 * The network services used to start from one check at boot: MQTT and OTA ran
 * only if WiFi was up at that moment, Improv only if it was down. A board that
 * booted with its router off never got MQTT or OTA, and an OTA image whose
 * first boot was offline was never confirmed and rolled back at the next
 * reset, because ota_manager_init() also arms the rollback-confirm timer.
 *
 * This core turns link observations into actions:
 *
 *   - OTA (with the rollback-confirm timer) starts on the first link-up of a
 *     boot, and only once.
 *   - MQTT starts on every link-up and stops on every link-down. The stop and
 *     start are the hook a later "home network" flag uses (ADR-028 §6).
 *   - Improv arms when the link has been down for NET_IMPROV_REARM_MS, or at
 *     once when the board has no stored credentials, and disarms on link-up.
 *
 * Pure C with an injected clock, so the uint32 millisecond wrap and an
 * offline hour are host-tested. The ESP-IDF glue in main.c polls the link,
 * feeds net_supervisor_step() and carries out the returned actions.
 */

#ifndef NET_SUPERVISOR_CORE_H
#define NET_SUPERVISOR_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Link down this long before Improv re-arms. Long enough that a router
 *  reboot (about a minute) does not open provisioning on a working board. */
#define NET_IMPROV_REARM_MS 120000u

/** Actions for one step; each flag is an edge, set on the step it applies. */
typedef struct {
    bool start_ota;     /**< First link-up of the boot: start OTA + confirm timer. */
    bool start_mqtt;    /**< Link came up: start (or resume) MQTT. */
    bool stop_mqtt;     /**< Link went down: stop MQTT. */
    bool arm_improv;    /**< Start accepting Improv provisioning. */
    bool disarm_improv; /**< Stop accepting it. */
} net_actions_t;

typedef struct {
    bool initialised;
    bool link_up;
    bool ota_started;
    bool mqtt_running;
    bool improv_armed;
    bool has_credentials;
    uint32_t down_since_ms; /**< When the link was last seen going (or being) down. */
} net_supervisor_t;

/**
 * @brief Start a boot.
 * @param has_credentials true when stored WiFi credentials exist; without
 *        them Improv arms on the first step instead of after the delay.
 */
void net_supervisor_init(net_supervisor_t *s, bool has_credentials, uint32_t now_ms);

/** Credentials were provisioned (Improv saved them); the delay applies again. */
void net_supervisor_set_credentials(net_supervisor_t *s, bool has_credentials);

/**
 * @brief Feed one observation of the link.
 * @param link_up  wifi_is_connected(), i.e. associated and holding an IP.
 * @return the actions to carry out now.
 */
net_actions_t net_supervisor_step(net_supervisor_t *s, bool link_up, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* NET_SUPERVISOR_CORE_H */
