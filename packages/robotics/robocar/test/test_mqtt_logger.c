/**
 * @file test_mqtt_logger.c
 * @brief mqtt_logger.c's client lifecycle against a recording esp-mqtt (#1054).
 *
 * mqtt_logger.c is compiled unmodified; mqtt_client.h, the queue and the
 * cJSON/esp_system/esp_wifi pieces come from test/include/mqtt_logger_host.
 *
 * The defect: ota_manager_init() registers the OTA check-now topic on the
 * first link-up, and the client used to be created and started in the same
 * step, after OTA. mqtt_logger_subscribe() refuses before mqtt_logger_init(),
 * so the subscription was dropped. ADR-028 §2's design is that init creates
 * the client at boot without starting it, and resume (on link-up) starts it;
 * a subscription registered in between is sent on the first connect. These
 * tests pin that sequence on the real module.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "config.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "mqtt_logger.h"

/* -------------------------------------------------------------------------- */
/* Recording esp-mqtt                                                           */
/* -------------------------------------------------------------------------- */

#define MAX_RECORDED_SUBS 16

static struct esp_mqtt_client {
    int unused;
} s_client_object;

static int s_init_calls;
static int s_start_calls;
static int s_stop_calls;
static int s_destroy_calls;
static bool s_started; /* esp-mqtt refuses a second start without a stop */
static esp_event_handler_t s_handler;
static void *s_handler_args;
static char s_subscribed[MAX_RECORDED_SUBS][128];
static int s_subscribed_count;

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config)
{
    (void)config;
    s_init_calls++;
    return &s_client_object;
}

esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client, esp_mqtt_event_id_t event,
                                         esp_event_handler_t handler, void *handler_args)
{
    (void)client;
    (void)event;
    s_handler = handler;
    s_handler_args = handler_args;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client)
{
    (void)client;
    s_start_calls++;
    if (s_started) {
        return ESP_FAIL;
    }
    s_started = true;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client)
{
    (void)client;
    s_stop_calls++;
    if (!s_started) {
        return ESP_FAIL;
    }
    s_started = false;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t client)
{
    (void)client;
    s_destroy_calls++;
    s_started = false;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_reconnect(esp_mqtt_client_handle_t client)
{
    (void)client;
    return ESP_OK;
}

int esp_mqtt_client_publish(esp_mqtt_client_handle_t client, const char *topic, const char *data,
                            int len, int qos, int retain)
{
    (void)client;
    (void)topic;
    (void)data;
    (void)len;
    (void)qos;
    (void)retain;
    return 1;
}

int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client, const char *topic, int qos)
{
    (void)client;
    (void)qos;
    if (s_subscribed_count < MAX_RECORDED_SUBS) {
        snprintf(s_subscribed[s_subscribed_count], sizeof(s_subscribed[0]), "%s", topic);
        s_subscribed_count++;
    }
    return 1;
}

/* -------------------------------------------------------------------------- */
/* Inert pieces the log task would use (it never runs here)                     */
/* -------------------------------------------------------------------------- */

static int s_queue_object;

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size)
{
    (void)length;
    (void)item_size;
    return &s_queue_object;
}

void vQueueDelete(QueueHandle_t queue)
{
    (void)queue;
}

BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait)
{
    (void)queue;
    (void)item;
    (void)wait;
    return pdFALSE;
}

BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait)
{
    (void)queue;
    (void)item;
    (void)wait;
    return pdTRUE;
}

BaseType_t xQueueSendToFront(QueueHandle_t queue, const void *item, TickType_t wait)
{
    return xQueueSend(queue, item, wait);
}

BaseType_t xQueueSendToBack(QueueHandle_t queue, const void *item, TickType_t wait)
{
    return xQueueSend(queue, item, wait);
}

UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue)
{
    (void)queue;
    return 0;
}

uint32_t esp_get_free_heap_size(void)
{
    return 0;
}

esp_err_t esp_wifi_sta_get_ap_info(wifi_ap_record_t *ap_info)
{
    (void)ap_info;
    return ESP_FAIL;
}

cJSON *cJSON_CreateObject(void)
{
    return NULL;
}

cJSON *cJSON_AddNumberToObject(cJSON *object, const char *name, double number)
{
    (void)object;
    (void)name;
    (void)number;
    return NULL;
}

cJSON *cJSON_AddStringToObject(cJSON *object, const char *name, const char *string)
{
    (void)object;
    (void)name;
    (void)string;
    return NULL;
}

char *cJSON_Print(const cJSON *item)
{
    (void)item;
    return NULL;
}

void cJSON_Delete(cJSON *item)
{
    (void)item;
}

/* -------------------------------------------------------------------------- */
/* Harness                                                                      */
/* -------------------------------------------------------------------------- */

static int s_failures;

#define ASSERT(cond)                                                 \
    do {                                                             \
        if (!(cond)) {                                               \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            fflush(stdout);                                          \
            s_failures++;                                            \
        }                                                            \
    } while (0)

static int s_notify_calls;
static char s_notify_topic[128];

static void notify_cb(const char *topic, const char *data, int data_len)
{
    (void)data;
    (void)data_len;
    s_notify_calls++;
    snprintf(s_notify_topic, sizeof(s_notify_topic), "%s", topic);
}

static const mqtt_logger_config_t k_cfg = {
    .broker_uri = "mqtt://192.0.2.1:1883",
    .client_id = MQTT_CLIENT_ID,
    .username = NULL,
    .password = NULL,
    .log_topic = MQTT_LOG_TOPIC_BASE,
    .status_topic = MQTT_STATUS_TOPIC,
    .command_topic = MQTT_COMMAND_TOPIC,
    .buffer_size = 4096,
    .keepalive_interval = 30,
    .qos_level = 1,
    .min_level = ESP_LOG_INFO,
    .retain_status = true,
};

static bool was_subscribed(const char *topic)
{
    for (int i = 0; i < s_subscribed_count; i++) {
        if (strcmp(s_subscribed[i], topic) == 0) {
            return true;
        }
    }
    return false;
}

static void fire(esp_mqtt_event_id_t id, const char *topic)
{
    esp_mqtt_event_t ev = {
        .event_id = id,
        .client = &s_client_object,
        .data = (char *)"check",
        .data_len = 5,
        .total_data_len = 5,
        .current_data_offset = 0,
        .topic = (char *)topic,
        .topic_len = topic ? (int)strlen(topic) : 0,
    };
    ASSERT(s_handler != NULL);
    if (s_handler) {
        s_handler(s_handler_args, "MQTT_EVENTS", (int32_t)id, &ev);
    }
}

static void reset_recorder(void)
{
    s_init_calls = s_start_calls = s_stop_calls = s_destroy_calls = 0;
    s_started = false;
    s_handler = NULL;
    s_handler_args = NULL;
    s_subscribed_count = 0;
    s_notify_calls = 0;
    s_notify_topic[0] = '\0';
}

/* -------------------------------------------------------------------------- */
/* Tests                                                                        */
/* -------------------------------------------------------------------------- */

/* The refusal ota_manager_init() must not ignore: before the client exists a
 * subscription is not recorded anywhere. */
static void test_subscribe_before_init_is_refused(void)
{
    reset_recorder();
    ASSERT(mqtt_logger_subscribe(OTA_MQTT_NOTIFY_TOPIC, notify_cb) == ESP_ERR_INVALID_STATE);
}

/* ADR-028 §2: init creates the client at boot and does not connect it. */
static void test_init_creates_client_without_starting_it(void)
{
    reset_recorder();
    ASSERT(mqtt_logger_init(&k_cfg) == ESP_OK);
    ASSERT(s_init_calls == 1);
    ASSERT(s_start_calls == 0);
    ASSERT(!mqtt_logger_is_connected());
    mqtt_logger_deinit();
}

/* The boot order after the fix: client at boot, OTA subscribes on the first
 * link-up, MQTT starts in the same step. The subscription must be on the
 * broker after the connect, and the check-now nudge must reach its handler. */
static void test_subscription_before_first_start_is_sent_on_connect(void)
{
    reset_recorder();
    ASSERT(mqtt_logger_init(&k_cfg) == ESP_OK);

    ASSERT(mqtt_logger_subscribe(OTA_MQTT_NOTIFY_TOPIC, notify_cb) == ESP_OK);
    ASSERT(s_subscribed_count == 0); /* not connected: nothing sent yet */

    ASSERT(mqtt_logger_resume() == ESP_OK);
    ASSERT(s_start_calls == 1);

    fire(MQTT_EVENT_CONNECTED, NULL);
    ASSERT(mqtt_logger_is_connected());
    ASSERT(was_subscribed(MQTT_COMMAND_TOPIC));
    ASSERT(was_subscribed(OTA_MQTT_NOTIFY_TOPIC));

    fire(MQTT_EVENT_DATA, OTA_MQTT_NOTIFY_TOPIC);
    ASSERT(s_notify_calls == 1);
    ASSERT(strcmp(s_notify_topic, OTA_MQTT_NOTIFY_TOPIC) == 0);

    mqtt_logger_deinit();
}

/* Link down then up: pause stops the client and clears the connected flag
 * itself (esp_mqtt_client_stop() dispatches no DISCONNECTED), and the next
 * connect re-issues the subscription. */
static void test_link_bounce_resubscribes(void)
{
    reset_recorder();
    ASSERT(mqtt_logger_init(&k_cfg) == ESP_OK);
    ASSERT(mqtt_logger_subscribe(OTA_MQTT_NOTIFY_TOPIC, notify_cb) == ESP_OK);
    ASSERT(mqtt_logger_resume() == ESP_OK);
    fire(MQTT_EVENT_CONNECTED, NULL);

    ASSERT(mqtt_logger_pause() == ESP_OK);
    ASSERT(s_stop_calls == 1);
    ASSERT(!mqtt_logger_is_connected());

    s_subscribed_count = 0;
    ASSERT(mqtt_logger_resume() == ESP_OK);
    ASSERT(s_start_calls == 2);
    fire(MQTT_EVENT_CONNECTED, NULL);
    ASSERT(was_subscribed(OTA_MQTT_NOTIFY_TOPIC));

    mqtt_logger_deinit();
}

int main(void)
{
    struct {
        const char *name;
        void (*fn)(void);
    } tests[] = {
        {"subscribe_before_init_is_refused", test_subscribe_before_init_is_refused},
        {"init_creates_client_without_starting_it", test_init_creates_client_without_starting_it},
        {"subscription_before_first_start_is_sent_on_connect",
         test_subscription_before_first_start_is_sent_on_connect},
        {"link_bounce_resubscribes", test_link_bounce_resubscribes},
    };
    const size_t n = sizeof(tests) / sizeof(tests[0]);
    for (size_t i = 0; i < n; i++) {
        printf("[%zu] Running: %s\n", i + 1, tests[i].name);
        fflush(stdout);
        tests[i].fn();
    }
    if (s_failures) {
        printf("test_mqtt_logger: %d assertion(s) failed\n", s_failures);
        return 1;
    }
    printf("test_mqtt_logger: all %zu tests passed\n", n);
    return 0;
}
