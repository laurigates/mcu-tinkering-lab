/**
 * @file freertos/semphr.h — host-test shim.
 *
 * A mutex that is always free. The host tests are single-threaded and drive the
 * module under test directly, so the only property that has to hold is that a
 * take succeeds and a give does not corrupt anything — the contention path
 * (where activity_trace.c drops a counter update rather than blocking) is not
 * reachable without a scheduler, and a shim that pretended otherwise would be
 * testing itself.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_SEMPHR_H
#define ROBOCAR_UNIFIED_HOST_TEST_SEMPHR_H

#include "freertos/FreeRTOS.h"

typedef void *SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t wait);
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem);
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t wait);
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t sem);
void vSemaphoreDelete(SemaphoreHandle_t sem);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_SEMPHR_H */
