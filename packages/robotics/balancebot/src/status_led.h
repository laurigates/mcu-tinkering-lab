// Onboard LEDs, driven from the core-0 service loop: the yellow user LED
// (GPIO25, active low) and the RGB LED (WS2812-type on GPIO22 via PIO1, its
// 5 V supply switched by GPIO23).
#ifndef BALANCEBOT_STATUS_LED_H
#define BALANCEBOT_STATUS_LED_H

#include "ipc.h"

void status_led_init(void);

// Call frequently from the core-0 loop; non-blocking.
// Yellow: IDLE slow blink (1 Hz), RUN solid, FAULT fast blink (5 Hz).
// RGB: status_color() (status_color.h), so the auto-arm window can be found
// by eye: yellow within 5 degrees, green inside the window, brightening as
// the hold counts toward arming.
void status_led_poll(const balance_snapshot_t *s);

#endif  // BALANCEBOT_STATUS_LED_H
