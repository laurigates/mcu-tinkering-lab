/**
 * @file pin_config.h
 * @brief Melody Detector — XIAO ESP32-S3 Sense pin assignments
 *
 * GPIO budget on XIAO headers (11 pins). The OV2640 camera uses internal
 * GPIOs on the Sense module — no conflict with D0..D10. USB-Serial-JTAG is
 * the console (USB-C), so U0TX (GPIO43) and U0RX (GPIO44) are free for I2S.
 *
 *   D1  GPIO2   capture button (input, internal pull-up)
 *   D5  GPIO6   I2S BCLK   → MAX98357A BCLK
 *   D6  GPIO43  I2S LRC    → MAX98357A LRC (WS)
 *   D7  GPIO44  I2S DIN    → MAX98357A DIN
 *   --  GPIO21  onboard user LED (active LOW)
 *
 * Free for future use (Phase 2+):
 *   D0 GPIO1, D2 GPIO3, D3 GPIO4, D4 GPIO5, D8 GPIO7, D9 GPIO8, D10 GPIO9
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/gpio.h"

// ========================================
// Capture trigger
// ========================================
#define BUTTON_PIN GPIO_NUM_2  // XIAO D1
#define BUTTON_DEBOUNCE_MS 30
#define BUTTON_ACTIVE_LEVEL 0  // active LOW (button pulls to GND)

// ========================================
// Status LED (onboard user LED)
// ========================================
#define STATUS_LED_PIN GPIO_NUM_21
#define STATUS_LED_ACTIVE_LEVEL 0  // active LOW

// ========================================
// I2S → MAX98357A
// ========================================
#define I2S_BCLK_PIN GPIO_NUM_6  // XIAO D5
#define I2S_LRC_PIN GPIO_NUM_43  // XIAO D6 (U0TX-pad, free under USB-Serial-JTAG)
#define I2S_DIN_PIN GPIO_NUM_44  // XIAO D7 (U0RX-pad, free under USB-Serial-JTAG)

#define AUDIO_SAMPLE_RATE_HZ 44100  // matches FR-4.02 synth target

#endif  // PIN_CONFIG_H
