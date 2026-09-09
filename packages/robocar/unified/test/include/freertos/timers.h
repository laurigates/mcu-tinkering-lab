/**
 * @file freertos/timers.h — host-test shim.
 *
 * xTimerCreate() hands back an opaque non-NULL handle WITHOUT arming anything,
 * for the same reason the task shim does: led_controller.c checks the handle
 * before using it, so the real code path needs a non-NULL value, while an
 * actually-firing timer would rewrite the LEDs underneath the assertions and
 * make them racy. The blink callback's behaviour is not under test here — the
 * channel placement of a single write is.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_TIMERS_H
#define ROBOCAR_UNIFIED_HOST_TEST_TIMERS_H

#include "freertos/FreeRTOS.h"

typedef void *TimerHandle_t;
typedef void (*TimerCallbackFunction_t)(TimerHandle_t);

TimerHandle_t xTimerCreate(const char *name, TickType_t period, UBaseType_t auto_reload,
                           void *timer_id, TimerCallbackFunction_t callback);
BaseType_t xTimerStart(TimerHandle_t timer, TickType_t block);
BaseType_t xTimerStop(TimerHandle_t timer, TickType_t block);
BaseType_t xTimerDelete(TimerHandle_t timer, TickType_t block);
BaseType_t xTimerChangePeriod(TimerHandle_t timer, TickType_t period, TickType_t block);
BaseType_t xTimerIsTimerActive(TimerHandle_t timer);
void *pvTimerGetTimerID(TimerHandle_t timer);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_TIMERS_H */
