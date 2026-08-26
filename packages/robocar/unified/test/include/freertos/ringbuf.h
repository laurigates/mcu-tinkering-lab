/**
 * @file freertos/ringbuf.h — host-test shim.
 *
 * A byte FIFO with the same contract audio_player.c relies on from
 * RINGBUF_TYPE_BYTEBUF: xRingbufferSend() appends, xRingbufferReceiveUpTo()
 * hands back a contiguous run capped at the requested size, and the item must
 * be returned before the space is reclaimed.
 *
 * The fake additionally counts every byte it has ACCEPTED
 * (ringbuf_fake_bytes_received()). That counter is the ground truth the
 * accounting test compares s_written_total against — the producer's tally must
 * equal what the ring actually took, or ring_pending() never returns to zero.
 */

#ifndef ROBOCAR_UNIFIED_HOST_TEST_RINGBUF_H
#define ROBOCAR_UNIFIED_HOST_TEST_RINGBUF_H

#include <stddef.h>

#include "freertos/FreeRTOS.h"

typedef void *RingbufHandle_t;

typedef enum {
    RINGBUF_TYPE_NOSPLIT = 0,
    RINGBUF_TYPE_ALLOWSPLIT,
    RINGBUF_TYPE_BYTEBUF,
} RingbufferType_t;

RingbufHandle_t xRingbufferCreateWithCaps(size_t size, RingbufferType_t type, uint32_t caps);
void vRingbufferDeleteWithCaps(RingbufHandle_t rb);
BaseType_t xRingbufferSend(RingbufHandle_t rb, const void *data, size_t size, TickType_t wait);
void *xRingbufferReceiveUpTo(RingbufHandle_t rb, size_t *out_size, TickType_t wait, size_t max);
void vRingbufferReturnItem(RingbufHandle_t rb, void *item);

/** Test-only: total bytes this ring has accepted since creation. */
size_t ringbuf_fake_bytes_received(RingbufHandle_t rb);

/** Test-only: the most recently created ring. audio_player.c keeps its handle
 *  file-static, so this is how a test reaches the ring the module allocated. */
RingbufHandle_t ringbuf_fake_last_created(void);

#endif /* ROBOCAR_UNIFIED_HOST_TEST_RINGBUF_H */
