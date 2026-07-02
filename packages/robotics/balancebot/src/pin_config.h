// Balancebot pin map — single source of truth.
// Board: Seeed XIAO RP2350. Side-header pins only; see WIRING.md for the
// full harness (driver strapping, pull-ups, power topology).
#ifndef BALANCEBOT_PIN_CONFIG_H
#define BALANCEBOT_PIN_CONFIG_H

// MPU6050 on I2C1 (XIAO D4/D5 are the board's default I2C pins)
#define PIN_IMU_SDA 6  // XIAO D4
#define PIN_IMU_SCL 7  // XIAO D5
#define PIN_IMU_INT 5  // XIAO D3, MPU6050 data-ready, active high

// DRV8825 step/dir. STEP pins must be consecutive-agnostic (one PIO SM
// each); DIR and nENABLE are plain CPU GPIOs.
#define PIN_STEP_LEFT 2      // XIAO D8, PIO0 SM0
#define PIN_STEP_RIGHT 3     // XIAO D10, PIO0 SM1
#define PIN_DIR_LEFT 4       // XIAO D9
#define PIN_DIR_RIGHT 0      // XIAO D6
#define PIN_MOTOR_NENABLE 1  // XIAO D7, shared, active low, external 10k pull-up

// Onboard peripherals
#define PIN_STATUS_LED 25       // User LED, active low
#define PIN_BATT_ADC 29         // ADC3, behind a /2 divider
#define PIN_BATT_ADC_ENABLE 19  // Drive high to connect the divider

// Spare side-header pins (all ADC-capable): GPIO26 (D0), GPIO27 (D1), GPIO28 (D2)

#endif  // BALANCEBOT_PIN_CONFIG_H
