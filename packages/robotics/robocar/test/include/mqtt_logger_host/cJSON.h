/**
 * @file cJSON.h — declarations-only shim for test_mqtt_logger (issue #1054).
 *
 * mqtt_logger.c builds log records with cJSON on a task the host test never
 * runs, so the test links inert stubs instead of libcjson and does not skip
 * when libcjson-dev is absent.
 */

#ifndef ROBOCAR_HOST_TEST_MQTT_LOGGER_CJSON_H
#define ROBOCAR_HOST_TEST_MQTT_LOGGER_CJSON_H

typedef struct cJSON cJSON;

cJSON *cJSON_CreateObject(void);
cJSON *cJSON_AddNumberToObject(cJSON *object, const char *name, double number);
cJSON *cJSON_AddStringToObject(cJSON *object, const char *name, const char *string);
char *cJSON_Print(const cJSON *item);
void cJSON_Delete(cJSON *item);

#endif /* ROBOCAR_HOST_TEST_MQTT_LOGGER_CJSON_H */
