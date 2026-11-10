/**
 * @file improv_wifi_device_info.c
 * @brief Device info strings for RPC 0x03, read from the running image.
 *
 * Kept apart from improv_wifi.c so the protocol code has no ESP-IDF
 * dependency and compiles in the host tests (packages/robocar/unified/test/),
 * which supply their own improv_wifi_device_info().
 *
 * ESP Web Tools compares the first string with the flasher manifest's `name`
 * to decide between "Update" and "Install" (which erases flash, NVS
 * credentials included), and the second with the manifest's `version`. The
 * component's CMakeLists.txt reads `name` from the project's flasher.json, the
 * file tools/generate-flasher-manifests.sh builds the manifest from, into
 * IMPROV_FIRMWARE_NAME. The version is PROJECT_VER, which each consumer reads
 * from its version.txt; release-please ("release-type": "simple") writes that
 * file and .release-please-manifest.json, where the manifest version comes
 * from, in the same commit.
 */

#include "esp_app_desc.h"
#include "improv_wifi.h"
#include "sdkconfig.h"

/* The chip family as esptool and ESP Web Tools spell it. */
#if CONFIG_IDF_TARGET_ESP32
#define IMPROV_CHIP "ESP32"
#elif CONFIG_IDF_TARGET_ESP32S2
#define IMPROV_CHIP "ESP32-S2"
#elif CONFIG_IDF_TARGET_ESP32S3
#define IMPROV_CHIP "ESP32-S3"
#elif CONFIG_IDF_TARGET_ESP32C3
#define IMPROV_CHIP "ESP32-C3"
#elif CONFIG_IDF_TARGET_ESP32C6
#define IMPROV_CHIP "ESP32-C6"
#elif CONFIG_IDF_TARGET_ESP32H2
#define IMPROV_CHIP "ESP32-H2"
#else
#define IMPROV_CHIP CONFIG_IDF_TARGET
#endif

void improv_wifi_device_info(improv_device_info_t *out)
{
    const esp_app_desc_t *app = esp_app_get_description();
#ifdef IMPROV_FIRMWARE_NAME
    out->firmware_name = IMPROV_FIRMWARE_NAME;
#else
    out->firmware_name = app->project_name;
#endif
    out->firmware_version = app->version;
    out->chip = IMPROV_CHIP;
    out->device_name = app->project_name;
}
