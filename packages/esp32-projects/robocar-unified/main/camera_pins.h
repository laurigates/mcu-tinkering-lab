/**
 * @file camera_pins.h
 * @brief XIAO ESP32-S3 Sense camera pin definitions
 *
 * The XIAO ESP32-S3 Sense has the OV2640 camera connected internally.
 * These pins are NOT on the D0-D10 headers -- they are internal to
 * the Sense expansion board, so there is zero conflict with user GPIOs.
 *
 * Reference: Seeed Studio XIAO ESP32-S3 Sense schematic
 */

#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

// XIAO ESP32-S3 Sense (OV2640) internal pin definitions
#define CAM_PIN_PWDN -1   // Not connected on XIAO Sense
#define CAM_PIN_RESET -1  // Not connected on XIAO Sense
#define CAM_PIN_XCLK 10   // Camera master clock

#define CAM_PIN_SIOD 40  // SCCB SDA (camera I2C)
#define CAM_PIN_SIOC 39  // SCCB SCL (camera I2C)

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
