/**
 * @file esp_heap_caps.h — host-test shim.
 *
 * audio_player.c only uses these as capability flags passed to
 * xRingbufferCreateWithCaps(); the fake ring ignores them.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_ESP_HEAP_CAPS_H
#define ROBOCAR_UNIFIED_HOST_TEST_ESP_HEAP_CAPS_H

#define MALLOC_CAP_8BIT (1 << 2)
#define MALLOC_CAP_SPIRAM (1 << 10)
#define MALLOC_CAP_INTERNAL (1 << 11)

#endif /* ROBOCAR_UNIFIED_HOST_TEST_ESP_HEAP_CAPS_H */
