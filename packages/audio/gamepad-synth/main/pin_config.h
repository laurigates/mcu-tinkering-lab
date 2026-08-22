/**
 * @file pin_config.h
 * @brief ESP32-S3 pin definitions for gamepad-synth
 *
 * Board: any ESP32-S3 dev board with USB-Serial-JTAG (Waveshare ESP32-S3-Zero,
 * ESP32-S3-DevKitC-1). No PSRAM, WiFi or SD card required.
 *
 * All six pins below are plain GPIOs on the ESP32-S3 with no strapping role,
 * so any other free pins work — change them here and in WIRING.md together.
 *
 * See WIRING.md for the wiring diagram and parts list.
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "driver/gpio.h"

// ========================================
// MAX98357A I2S Class-D Amplifier
// ========================================
#define I2S_BCLK_PIN GPIO_NUM_5  // MAX98357A BCLK — bit clock
#define I2S_WS_PIN GPIO_NUM_6    // MAX98357A LRC  — word select / LRCLK
#define I2S_DOUT_PIN GPIO_NUM_7  // MAX98357A DIN  — serial data to amp

// ========================================
// Status LED
// ========================================
#define LED_PIN GPIO_NUM_2  // LED anode via 220-330 ohm resistor

// ========================================
// Piezo Voices (LEDC square wave, Drone mode only)
// Optional accent voices — the firmware runs fine with no discs fitted.
// ========================================
#define PIEZO_A_PIN GPIO_NUM_8  // Piezo disc A (+)
#define PIEZO_B_PIN GPIO_NUM_9  // Piezo disc B (+)

#endif  // PIN_CONFIG_H
