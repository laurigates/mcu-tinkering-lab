#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t button_init(void);

// Blocks until the next debounced press edge (or returns false on timeout).
bool button_wait_press(uint32_t timeout_ms);

#endif  // BUTTON_H
