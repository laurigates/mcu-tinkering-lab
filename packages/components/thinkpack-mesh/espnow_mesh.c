/**
 * @file espnow_mesh.c
 * @brief ESP-NOW mesh layer implementation for the ThinkPack modular toy mesh.
 *
 * Two FreeRTOS tasks run on Core 0:
 *   - Beacon task: broadcasts MSG_BEACON every beacon_interval_ms, drives
 *     leader_election_tick() every 100 ms, and prunes stale peers.
 *   - Receive task: dequeues packets pushed by the ESP-NOW receive ISR
 *     callback, validates them, and dispatches to group_manager /
 *     leader_election / app callback.
 */

#include "espnow_mesh.h"

#include <inttypes.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "group_manager.h"
#include "leader_election.h"
#include "thinkpack_protocol.h"

static const char *TAG = "mesh";

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

/** Broadcast MAC used by ESP-NOW for group transmissions. */
static const uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/** Receive queue depth — holds up to this many packets before dropping. */
#define RECV_QUEUE_DEPTH 16

/** Stack size for the beacon task (words). */
#define BEACON_TASK_STACK 4096

/** Stack size for the receive task (words). */
#define RECV_TASK_STACK 4096

/** How often the beacon task calls leader_election_tick() (ms). */
#define TICK_INTERVAL_MS 100u

/** Minimum stale-peer threshold regardless of beacon_interval_ms setting. */
#define STALE_MS_MIN 3000u

/* ------------------------------------------------------------------ */
/* Receive queue entry                                                 */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t src_mac[6];
    thinkpack_packet_t packet;
    int8_t rssi;
} recv_entry_t;

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static thinkpack_mesh_config_t s_config;
static uint8_t s_own_mac[6] = {0};
static uint32_t s_own_priority = 0;
static uint8_t s_seq = 0;

static QueueHandle_t s_recv_queue = NULL;
static TaskHandle_t s_beacon_task_handle = NULL;
static TaskHandle_t s_recv_task_handle = NULL;

static thinkpack_mesh_event_cb_t s_event_cb = NULL;
static void *s_event_cb_ctx = NULL;
static SemaphoreHandle_t s_cb_mutex = NULL;

static bool s_running = false;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void fire_event(thinkpack_mesh_event_t type, const uint8_t peer_mac[6],
                       const thinkpack_packet_t *packet)
{
    xSemaphoreTake(s_cb_mutex, portMAX_DELAY);
    thinkpack_mesh_event_cb_t cb = s_event_cb;
    void *ctx = s_event_cb_ctx;
    xSemaphoreGive(s_cb_mutex);

    if (!cb) {
        return;
    }

    thinkpack_mesh_event_data_t ev = {
        .type = type,
        .packet = packet,
    };
    if (peer_mac) {
        memcpy(ev.peer_mac, peer_mac, 6);
    }
    cb(&ev, ctx);
}

/**
 * Ensure a unicast peer is registered with the ESP-NOW driver.
 * Idempotent — safe to call before every unicast send.
 */
static esp_err_t ensure_peer(const uint8_t mac[6])
{
    if (esp_now_is_peer_exist(mac)) {
        return ESP_OK;
    }
    esp_now_peer_info_t peer = {
        .channel = s_config.channel,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac, 6);
    esp_err_t ret = esp_now_add_peer(&peer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_add_peer " MACSTR " failed: %s", MAC2STR(mac), esp_err_to_name(ret));
    }
    return ret;
}

/* ------------------------------------------------------------------ */
/* Leader-election callback (called from beacon task via tick)        */
/* ------------------------------------------------------------------ */

static void on_leader_change(leader_state_t state, const uint8_t leader_mac[6],
                             uint32_t leader_priority, void *user_ctx)
{
    (void)user_ctx;

    switch (state) {
        case LEADER_STATE_LEADER: {
            ESP_LOGI(TAG, "Became leader (priority %" PRIu32 ")", leader_priority);
            /* Broadcast a leader claim so followers know immediately. */
            thinkpack_packet_t pkt;
            thinkpack_prepare_leader_claim(&pkt, s_seq++, s_own_mac, s_own_priority,
                                           s_config.channel);
            esp_now_send(BROADCAST_MAC, (const uint8_t *)&pkt, sizeof(pkt));
            fire_event(THINKPACK_EVENT_BECAME_LEADER, s_own_mac, NULL);
            fire_event(THINKPACK_EVENT_LEADER_ELECTED, s_own_mac, NULL);
            break;
        }
        case LEADER_STATE_FOLLOWER:
            ESP_LOGI(TAG, "Became follower; leader " MACSTR " priority %" PRIu32,
                     MAC2STR(leader_mac), leader_priority);
            fire_event(THINKPACK_EVENT_BECAME_FOLLOWER, leader_mac, NULL);
            fire_event(THINKPACK_EVENT_LEADER_ELECTED, leader_mac, NULL);
            break;

        case LEADER_STATE_UNKNOWN:
            ESP_LOGW(TAG, "Leader lost — re-election triggered");
            fire_event(THINKPACK_EVENT_LEADER_LOST, leader_mac, NULL);
            break;

        case LEADER_STATE_ELECTING:
            /* Broadcast an election bid so peers can compare priorities. */
            {
                thinkpack_packet_t bid_pkt;
                thinkpack_prepare_election_bid(&bid_pkt, s_seq++, s_own_mac, s_own_priority);
                esp_now_send(BROADCAST_MAC, (const uint8_t *)&bid_pkt, sizeof(bid_pkt));
            }
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Prune callback                                                      */
/* ------------------------------------------------------------------ */

static void on_peer_pruned(const thinkpack_peer_t *peer, void *user_ctx)
{
    (void)user_ctx;
    ESP_LOGI(TAG, "Peer lost: " MACSTR " '%s'", MAC2STR(peer->mac), peer->name);
    fire_event(THINKPACK_EVENT_PEER_LOST, peer->mac, NULL);
    /* Remove from ESP-NOW driver too; ignore error if not present. */
    esp_now_del_peer(peer->mac);
}

/* ------------------------------------------------------------------ */
/* ESP-NOW callbacks (WiFi task context — keep minimal)               */
/* ------------------------------------------------------------------ */

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "Send to " MACSTR " failed", MAC2STR(mac_addr));
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    if (!recv_info || !data || data_len <= 0) {
        return;
    }
    if ((size_t)data_len != sizeof(thinkpack_packet_t)) {
        ESP_LOGW(TAG, "Unexpected packet size %d (expected %zu)", data_len,
                 sizeof(thinkpack_packet_t));
        return;
    }

    recv_entry_t entry;
    memcpy(entry.src_mac, recv_info->src_addr, 6);
    memcpy(&entry.packet, data, sizeof(thinkpack_packet_t));
    entry.rssi = recv_info->rx_ctrl ? (int8_t)recv_info->rx_ctrl->rssi : 0;

    /* Non-blocking push — drop if the queue is full rather than stalling the WiFi task. */
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_recv_queue, &entry, &woken) != pdTRUE) {
        ESP_LOGW(TAG, "Receive queue full — packet dropped");
    }
    portYIELD_FROM_ISR(woken);
}

/* ------------------------------------------------------------------ */
/* Beacon task                                                         */
/* ------------------------------------------------------------------ */

static void beacon_task(void *arg)
{
    (void)arg;

    uint32_t last_tick_ms = now_ms();
    uint32_t last_beacon_ms = 0;

    /* Compute stale threshold: 3× beacon interval, minimum 3 s. */
    uint32_t stale_ms = s_config.beacon_interval_ms * 3;
    if (stale_ms < STALE_MS_MIN) {
        stale_ms = STALE_MS_MIN;
    }

    while (s_running) {
        uint32_t t = now_ms();

        /* Drive the election timer every TICK_INTERVAL_MS. */
        if ((t - last_tick_ms) >= TICK_INTERVAL_MS) {
            leader_election_tick(t);
            last_tick_ms = t;
        }

        /* Send a beacon every beacon_interval_ms. */
        if ((t - last_beacon_ms) >= s_config.beacon_interval_ms) {
            thinkpack_beacon_data_t beacon = {
                .box_type = s_config.box_type,
                .capabilities = s_config.capabilities,
                .priority = s_own_priority,
                .battery_level = s_config.battery_level,
                .group_state = 0,
            };
            memcpy(beacon.name, s_config.name, THINKPACK_BOX_NAME_LEN);

            thinkpack_packet_t pkt;
            thinkpack_prepare_beacon(&pkt, s_seq++, s_own_mac, &beacon);
            esp_now_send(BROADCAST_MAC, (const uint8_t *)&pkt, sizeof(pkt));

            /* Prune peers that have gone quiet. */
            group_manager_prune(t, stale_ms, on_peer_pruned, NULL);

            last_beacon_ms = t;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* Receive task                                                        */
/* ------------------------------------------------------------------ */

static void recv_task(void *arg)
{
    (void)arg;

    recv_entry_t entry;

    while (s_running) {
        if (xQueueReceive(s_recv_queue, &entry, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        thinkpack_packet_t *pkt = &entry.packet;

        /* Validate magic bytes. */
        if (pkt->magic[0] != THINKPACK_MAGIC_0 || pkt->magic[1] != THINKPACK_MAGIC_VERSION) {
            ESP_LOGW(TAG, "Bad magic from " MACSTR " — discarding", MAC2STR(entry.src_mac));
            continue;
        }

        /* Validate checksum. */
        if (!thinkpack_verify_checksum(pkt)) {
            ESP_LOGW(TAG, "Checksum fail from " MACSTR " — discarding", MAC2STR(entry.src_mac));
            continue;
        }

        uint32_t t = now_ms();

        switch ((thinkpack_msg_type_t)pkt->msg_type) {
            case MSG_BEACON: {
                if (pkt->data_length < sizeof(thinkpack_beacon_data_t)) {
                    break;
                }
                const thinkpack_beacon_data_t *b = (const thinkpack_beacon_data_t *)pkt->data;

                thinkpack_peer_t peer = {
                    .box_type = b->box_type,
                    .capabilities = b->capabilities,
                    .priority = b->priority,
                    .last_seen_ms = t,
                    .rssi = entry.rssi,
                };
                memcpy(peer.mac, entry.src_mac, 6);
                memcpy(peer.name, b->name, THINKPACK_BOX_NAME_LEN);

                bool newly_added = group_manager_upsert(&peer);
                if (newly_added) {
                    /* Register with ESP-NOW driver for future unicast. */
                    ensure_peer(entry.src_mac);
                    fire_event(THINKPACK_EVENT_PEER_DISCOVERED, entry.src_mac, NULL);
                }

                /* Let election module observe the beacon's priority as a bid. */
                leader_election_observe_bid(entry.src_mac, b->priority);
                break;
            }

            case MSG_CAPABILITY_REPLY: {
                if (pkt->data_length < sizeof(thinkpack_capability_reply_data_t)) {
                    break;
                }
                const thinkpack_capability_reply_data_t *r =
                    (const thinkpack_capability_reply_data_t *)pkt->data;

                thinkpack_peer_t peer = {
                    .box_type = r->box_type,
                    .capabilities = r->capabilities,
                    .priority = r->priority,
                    .last_seen_ms = t,
                    .rssi = entry.rssi,
                };
                memcpy(peer.mac, entry.src_mac, 6);
                memcpy(peer.name, r->name, THINKPACK_BOX_NAME_LEN);

                bool newly_added = group_manager_upsert(&peer);
                if (newly_added) {
                    ensure_peer(entry.src_mac);
                    fire_event(THINKPACK_EVENT_PEER_DISCOVERED, entry.src_mac, NULL);
                }

                leader_election_observe_bid(entry.src_mac, r->priority);
                break;
            }

            case MSG_ELECTION_BID: {
                if (pkt->data_length < sizeof(thinkpack_election_bid_data_t)) {
                    break;
                }
                const thinkpack_election_bid_data_t *bid =
                    (const thinkpack_election_bid_data_t *)pkt->data;
                leader_election_observe_bid(bid->mac, bid->priority);
                break;
            }

            case MSG_LEADER_CLAIM: {
                if (pkt->data_length < sizeof(thinkpack_leader_claim_data_t)) {
                    break;
                }
                const thinkpack_leader_claim_data_t *claim =
                    (const thinkpack_leader_claim_data_t *)pkt->data;
                leader_election_observe_claim(claim->mac, claim->priority);
                break;
            }

            case MSG_SYNC_PULSE:
                fire_event(THINKPACK_EVENT_SYNC_PULSE, entry.src_mac, pkt);
                break;

            case MSG_COMMAND:
                fire_event(THINKPACK_EVENT_COMMAND_RECEIVED, entry.src_mac, pkt);
                break;

            default:
                ESP_LOGD(TAG, "Unhandled msg_type 0x%02x from " MACSTR, pkt->msg_type,
                         MAC2STR(entry.src_mac));
                break;
        }
    }

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t thinkpack_mesh_init(const thinkpack_mesh_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_config, config, sizeof(thinkpack_mesh_config_t));

    /* ---- NVS ---- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated — erasing and re-initialising");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ---- Network stack ---- */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(config->channel, WIFI_SECOND_CHAN_NONE));

    /* ---- Local MAC ---- */
    ESP_ERROR_CHECK(esp_read_mac(s_own_mac, ESP_MAC_WIFI_STA));
    ESP_LOGI(TAG, "Local MAC: " MACSTR, MAC2STR(s_own_mac));

    /* ---- Priority ---- */
    s_own_priority = thinkpack_priority_for_capabilities(config->capabilities, s_own_mac);
    ESP_LOGI(TAG, "Own priority: %" PRIu32, s_own_priority);

    /* ---- ESP-NOW ---- */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Register broadcast peer so esp_now_send() accepts the broadcast address. */
    esp_now_peer_info_t bcast_peer = {
        .channel = config->channel,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(bcast_peer.peer_addr, BROADCAST_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&bcast_peer));

    /* ---- Queues / mutexes ---- */
    s_recv_queue = xQueueCreate(RECV_QUEUE_DEPTH, sizeof(recv_entry_t));
    if (!s_recv_queue) {
        ESP_LOGE(TAG, "Failed to create receive queue");
        return ESP_ERR_NO_MEM;
    }

    s_cb_mutex = xSemaphoreCreateMutex();
    if (!s_cb_mutex) {
        ESP_LOGE(TAG, "Failed to create callback mutex");
        return ESP_ERR_NO_MEM;
    }

    /* ---- Sub-modules ---- */
    ESP_ERROR_CHECK(group_manager_init());
    ESP_ERROR_CHECK(leader_election_init(s_own_priority, s_own_mac, on_leader_change, NULL));

    ESP_LOGI(TAG, "Mesh initialised on channel %d", config->channel);
    return ESP_OK;
}

esp_err_t thinkpack_mesh_start(void)
{
    s_running = true;

    BaseType_t rc;
    rc = xTaskCreatePinnedToCore(beacon_task, "tp_beacon", BEACON_TASK_STACK, NULL, 5,
                                 &s_beacon_task_handle, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create beacon task");
        s_running = false;
        return ESP_ERR_NO_MEM;
    }

    rc = xTaskCreatePinnedToCore(recv_task, "tp_recv", RECV_TASK_STACK, NULL, 5,
                                 &s_recv_task_handle, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create receive task");
        s_running = false;
        vTaskDelete(s_beacon_task_handle);
        s_beacon_task_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    /* Kick off the first election. */
    ESP_ERROR_CHECK(leader_election_trigger());

    ESP_LOGI(TAG, "Mesh started");
    return ESP_OK;
}

esp_err_t thinkpack_mesh_stop(void)
{
    s_running = false;

    /* Give tasks time to exit their loops before we tear down resources. */
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();

    if (s_recv_queue) {
        vQueueDelete(s_recv_queue);
        s_recv_queue = NULL;
    }

    ESP_LOGI(TAG, "Mesh stopped");
    return ESP_OK;
}

esp_err_t thinkpack_mesh_set_event_callback(thinkpack_mesh_event_cb_t cb, void *user_ctx)
{
    xSemaphoreTake(s_cb_mutex, portMAX_DELAY);
    s_event_cb = cb;
    s_event_cb_ctx = user_ctx;
    xSemaphoreGive(s_cb_mutex);
    return ESP_OK;
}

esp_err_t thinkpack_mesh_send(const uint8_t *mac, const thinkpack_packet_t *packet)
{
    if (!packet) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t *dest = mac ? mac : BROADCAST_MAC;

    if (mac) {
        esp_err_t ret = ensure_peer(mac);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return esp_now_send(dest, (const uint8_t *)packet, sizeof(thinkpack_packet_t));
}

void thinkpack_mesh_get_mac(uint8_t out_mac[6])
{
    if (out_mac) {
        memcpy(out_mac, s_own_mac, 6);
    }
}

size_t thinkpack_mesh_peer_count(void)
{
    return group_manager_count();
}
