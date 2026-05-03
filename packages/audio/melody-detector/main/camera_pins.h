/**
 * @file camera_pins.h
 * @brief XIAO ESP32-S3 Sense OV2640 internal pin map
 *
 * The OV2640 is wired internally on the Sense expansion board. These pins
 * are NOT routed to the D0..D10 user headers, so there's zero conflict
 * with user GPIOs.
 */

#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 10
#define CAM_PIN_SIOD 40
#define CAM_PIN_SIOC 39

#define CAM_PIN_D7 48
#define CAM_PIN_D6 11
#define CAM_PIN_D5 12
#define CAM_PIN_D4 14
#define CAM_PIN_D3 16
#define CAM_PIN_D2 18
#define CAM_PIN_D1 17
#define CAM_PIN_D0 15

#define CAM_PIN_VSYNC 38
#define CAM_PIN_HREF 47
#define CAM_PIN_PCLK 13

#endif  // CAMERA_PINS_H
