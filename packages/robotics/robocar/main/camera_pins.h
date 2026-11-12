/**
 * @file camera_pins.h
 * @brief XIAO ESP32-S3 Sense camera pin definitions
 *
 * The camera is connected internally on the Sense expansion board. These pins
 * are NOT on the D0-D10 headers, so there is zero conflict with user GPIOs.
 *
 * SENSOR: this board reports PID=0x3660, i.e. an **OV3660** — not the OV2640
 * this file used to claim. The difference is not cosmetic: the two sensors have
 * incompatible register maps (OV2640 is 8-bit-addressed and bank-switched,
 * OV3660 is 16-bit-addressed), and the esp32-camera driver's own
 * set_gainceiling() means completely different things on each — an enum index
 * 0..6 on OV2640, a RAW 10-bit ceiling value on OV3660. Writing the OV2640
 * "lowest" value of 0 to an OV3660 sets its gain ceiling to zero, which is what
 * made this robot's frames dark. Read the PID that camera_init() logs before
 * reasoning about any sensor register.
 *
 * Reference: Seeed Studio XIAO ESP32-S3 Sense schematic
 */

#ifndef CAMERA_PINS_H
#define CAMERA_PINS_H

// XIAO ESP32-S3 Sense internal camera pin definitions
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
