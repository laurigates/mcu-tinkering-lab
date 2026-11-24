// Core-0 side of the carrier's pack-voltage sense (#747): reads D0 with the ADC,
// runs pack_sense.c's monitor and hands core 1 the cutoff. Core 0 only: the ADC
// is shared with the CLI's battery reading, which also runs here.
#ifndef BALANCEBOT_PACK_H
#define BALANCEBOT_PACK_H

#include <stdint.h>

#include "ipc.h"
#include "pack_sense.h"

// `params` is core 0's mirror; its pack divider and cell count are read on
// every poll, so a `set` or `load` takes effect on the next one. Call after
// adc_init().
void pack_init(const balance_params_t *params);

// Take a reading every PACK_POLL_MS and update the cutoff. Cheap when not due.
void pack_poll(void);

// The latest reading and monitor state.
pack_reading_t pack_get_reading(void);
pack_level_t pack_get_level(void);

// The latest reading in 10 mV steps (0 = no pack on D0), for the log row.
uint16_t pack_get_centivolts(void);

#endif  // BALANCEBOT_PACK_H
