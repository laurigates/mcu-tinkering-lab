/**
 * @file mqtt_client.h — recording esp-mqtt shim for test_mqtt_logger (issue #1054).
 *
 * Only on test_mqtt_logger's include path, ahead of test/include. Declares the
 * slice of esp-mqtt that mqtt_logger.c calls; test_mqtt_logger.c implements
 * each function as a recorder, so the test can read which calls the real
 * mqtt_logger.c made (start, stop, subscribe) and deliver events to the handler
 * it registered.
 */

#ifndef ROBOCAR_HOST_TEST_MQTT_CLIENT_H
#define ROBOCAR_HOST_TEST_MQTT_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef const char *esp_event_base_t;
typedef void (*esp_event_handler_t)(void *handler_args, esp_event_base_t base, int32_t event_id,
                                    void *event_data);
#define ESP_EVENT_ANY_ID (-1)

typedef struct esp_mqtt_client *esp_mqtt_client_handle_t;

typedef enum {
    MQTT_EVENT_ANY = -1,
    MQTT_EVENT_ERROR = 0,
    MQTT_EVENT_CONNECTED,
    MQTT_EVENT_DISCONNECTED,
    MQTT_EVENT_SUBSCRIBED,
    MQTT_EVENT_UNSUBSCRIBED,
    MQTT_EVENT_PUBLISHED,
    MQTT_EVENT_DATA,
    MQTT_EVENT_BEFORE_CONNECT,
    MQTT_EVENT_DELETED,
} esp_mqtt_event_id_t;

typedef struct {
    esp_mqtt_event_id_t event_id;
    esp_mqtt_client_handle_t client;
    char *data;
    int data_len;
    int total_data_len;
    int current_data_offset;
    char *topic;
    int topic_len;
    int msg_id;
} esp_mqtt_event_t;

typedef esp_mqtt_event_t *esp_mqtt_event_handle_t;

typedef struct {
    struct {
        struct {
            const char *uri;
        } address;
    } broker;
    struct {
        const char *username;
        const char *client_id;
        struct {
            const char *password;
        } authentication;
    } credentials;
    struct {
        struct {
            const char *topic;
            const char *msg;
            int msg_len;
            int qos;
            int retain;
        } last_will;
        int keepalive;
    } session;
    struct {
        int reconnect_timeout_ms;
        int timeout_ms;
    } network;
} esp_mqtt_client_config_t;

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config);
esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client, esp_mqtt_event_id_t event,
                                         esp_event_handler_t handler, void *handler_args);
esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_reconnect(esp_mqtt_client_handle_t client);
int esp_mqtt_client_publish(esp_mqtt_client_handle_t client, const char *topic, const char *data,
                            int len, int qos, int retain);
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client, const char *topic, int qos);

#endif /* ROBOCAR_HOST_TEST_MQTT_CLIENT_H */
