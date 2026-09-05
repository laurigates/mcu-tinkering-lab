/**
 * @file esp_app_desc.h — host-test shim.
 *
 * self_report.c reads only the version string out of the application
 * descriptor. The real struct carries a build timestamp, an ELF SHA and the
 * project name too; none of that reaches the facts line, so the shim declares
 * just the field under test at its real width (a 32-byte array, which is what
 * makes the truncation the test exercises reproducible).
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_APP_DESC_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_APP_DESC_H

typedef struct {
    char version[32];
} esp_app_desc_t;

const esp_app_desc_t *esp_app_get_description(void);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_APP_DESC_H */
