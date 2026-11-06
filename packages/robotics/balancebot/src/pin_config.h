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

// Carrier PCB only: pack voltage through a 100k/22k divider (ratio 0.1803:
// 12.6 V, a full 3S pack, reads 2.27 V; 16.8 V, a full 4S pack, 3.03 V). The
// hand-wired harness leaves D0 free; firmware does not read it yet (#747).
#define PIN_PACK_SENSE 26  // XIAO D0, ADC0

// Carrier PCB only (rev3): the expansion header J6 (5V, GND, D2, D1), each
// GPIO behind a 1 kOhm series resistor, for a cable to an ESP-NOW companion
// XIAO (#960), a servo's PWM or an ADC input. Both are ADC-capable. Firmware
// does not use them yet; the hand-wired harness leaves them free. TX and RX
// are on different UARTs (uart0 TX; uart1 RX via GPIO_FUNC_UART_AUX): a
// hardware-UART link initialises both at one baud rate, or uses a PIO UART.
#define PIN_EXP_D1 27  // XIAO D1, ADC1; UART1 RX (GPIO function F11)
#define PIN_EXP_D2 28  // XIAO D2, ADC2; UART0 TX (GPIO function F2)

#endif  // BALANCEBOT_PIN_CONFIG_H
