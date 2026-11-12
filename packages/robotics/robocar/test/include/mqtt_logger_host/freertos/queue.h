/**
 * @file freertos/queue.h — inert queue shim for test_mqtt_logger (issue #1054).
 *
 * mqtt_logger.c's log queue is drained by a task the host test never runs, so
 * the queue only has to exist: creation returns a non-NULL handle, a send
 * succeeds and a receive finds nothing. test_mqtt_logger.c implements these.
 */

#ifndef ROBOCAR_HOST_TEST_QUEUE_H
#define ROBOCAR_HOST_TEST_QUEUE_H

#include "freertos/FreeRTOS.h"

typedef void *QueueHandle_t;

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
void vQueueDelete(QueueHandle_t queue);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait);
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait);
BaseType_t xQueueSendToFront(QueueHandle_t queue, const void *item, TickType_t wait);
BaseType_t xQueueSendToBack(QueueHandle_t queue, const void *item, TickType_t wait);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t queue);

#endif /* ROBOCAR_HOST_TEST_QUEUE_H */
