/* host-test shim: pca9685_restart() busy-waits with ets_delay_us(). */
#ifndef ROBOCAR_UNIFIED_HOST_TEST_ETS_SYS_H
#define ROBOCAR_UNIFIED_HOST_TEST_ETS_SYS_H
#include <stdint.h>
static inline void ets_delay_us(uint32_t us)
{
    (void)us;
}
#endif
