/**
 * @file camera_pins.h
 * @brief ESP32-CAM pin definitions
 */

#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

// ESP32-CAM (AI Thinker) pin definitions
#define CAM_PIN_PWDN     32
#define CAM_PIN_RESET    -1
#define CAM_PIN_XCLK      0
#define CAM_PIN_SIOD     26
#define CAM_PIN_SIOC     27

#define CAM_PIN_D7       35
#define CAM_PIN_D6       34
#define CAM_PIN_D5       39
#define CAM_PIN_D4       36
#define CAM_PIN_D3       21
#define CAM_PIN_D2       19
#define CAM_PIN_D1       18
#define CAM_PIN_D0        5
#define CAM_PIN_VSYNC    25
#define CAM_PIN_HREF     23
#define CAM_PIN_PCLK     22

// LED pin for status indication
#define CAM_LED_PIN       4

// Serial communication pins (UART2)
// Note: GPIO16/17 are used by PSRAM on ESP32-CAM
// Use GPIO14/15 which are safe and available
#define UART_TX_PIN      14  // GPIO14 (available pin)
#define UART_RX_PIN      15  // GPIO15 (available pin)

#endif // CAMERA_PINS_H