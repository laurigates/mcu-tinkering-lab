/**
 * @file freertos/task.h — host-test shim.
 *
 * xTaskCreatePinnedToCore() hands back an opaque non-NULL handle WITHOUT
 * running the task. That is deliberate: audio_player.c's producer path checks
 * `if (s_task)` before notifying, so the handle has to be non-NULL for the real
 * code path to be exercised — but a running player_task() would drain the ring
 * concurrently and make the byte-accounting assertions racy. The host tests
 * cover the producer's arithmetic; the player's drain is not under test here.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_TASK_H
#define ROBOCAR_UNIFIED_HOST_TEST_TASK_H

#include "freertos/FreeRTOS.h"

typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack, void *arg,
                                   UBaseType_t prio, TaskHandle_t *out, BaseType_t core);
void vTaskDelay(TickType_t ticks);
/* Declared here rather than in a portmacro shim: it is the only piece of the
 * port layer any host-tested module touches (self_report.c logs its core). */
BaseType_t xPortGetCoreID(void);
void vTaskDelete(TaskHandle_t task);
uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t wait);
void xTaskNotifyGive(TaskHandle_t task);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_TASK_H */
