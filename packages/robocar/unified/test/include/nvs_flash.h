/**
 * @file nvs_flash.h — host-test shim.
 *
 * credentials_loader.c includes this but calls nothing from it (nvs_flash_init()
 * runs in main.c). Present only so the include resolves on the host.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_NVS_FLASH_H
#define ROBOCAR_UNIFIED_HOST_TEST_NVS_FLASH_H

#include "nvs.h"

#endif /* ROBOCAR_UNIFIED_HOST_TEST_NVS_FLASH_H */
