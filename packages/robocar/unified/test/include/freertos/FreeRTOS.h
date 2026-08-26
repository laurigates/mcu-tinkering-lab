/**
 * @file freertos/FreeRTOS.h — host-test shim.
 *
 * Just enough of the FreeRTOS vocabulary for audio_player.c to compile on the
 * host. Nothing here schedules anything: the host tests drive the producer side
 * of audio_player.c directly and never start the player task, so tick counts
 * only have to be representable, not honoured.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_FREERTOS_H
#define ROBOCAR_UNIFIED_HOST_TEST_FREERTOS_H

#include <stdint.h>

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdFAIL 0

#define portMAX_DELAY ((TickType_t)0xFFFFFFFFU)
#define configTICK_RATE_HZ 1000

#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

#endif /* ROBOCAR_UNIFIED_HOST_TEST_FREERTOS_H */
