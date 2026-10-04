/**
 * @file improv_wifi_uart.c
 * @brief Default Improv reply transport: UART0.
 *
 * Kept apart from improv_wifi.c so the protocol code has no driver dependency
 * and compiles in the host tests (packages/robocar/unified/test/).
 */

#include "driver/uart.h"
#include "improv_wifi.h"

void improv_wifi_uart0_write(const uint8_t *data, size_t len)
{
    uart_write_bytes(UART_NUM_0, (const char *)data, len);
}
