# Hardware Overview

This document outlines the primary hardware components and their connections for the RoboCar project, based on the `main_test.ino` configuration.

## Core Components

*   **Main Microcontroller:** DOIT ESP32 DevKit V1 (36-pin variant)
*   **Motor Driver:** SparkFun Motor Driver - Dual TB6612FNG (1A)
*   **Motors:** 2 x DC Motors (Left and Right)
*   **LEDs:** 2 x Common Anode RGB LEDs (Left and Right)
*   **Servos:** 2 x SG90 Micro Servos (for Pan and Tilt)
*   **PWM Driver:** PCA9685 16-Channel 12-bit PWM/Servo Driver (I2C Interface)
*   **Sound Output:** Piezo Buzzer
*   **Camera Module:** AI-Thinker ESP32-CAM board. Responsible for capturing images, potentially running local processing, and communicating with external AI services (like Claude). Communicates results/commands back to the main ESP32 via Serial (UART2).

## ESP32 Pin Connections (`main_test.ino`)

| GPIO | Function             | Component Connected To | Notes                                       |
| :--- | :------------------- | :------------------- | :------------------------------------------ |
| 33   | `LEFT_PWMB`          | Motor Driver         | PWM Signal for Left Motor Speed             |
| 32   | `LEFT_IN2`           | Motor Driver         | Direction Control 2 for Left Motor          |
| 35   | `LEFT_IN1`           | Motor Driver         | Direction Control 1 for Left Motor          |
| 25   | `STBY`               | Motor Driver         | Standby Pin (Enable/Disable Driver)         |
| 18   | `RIGHT_IN1`          | Motor Driver         | Direction Control 1 for Right Motor         |
| 19   | `RIGHT_IN2`          | Motor Driver         | Direction Control 2 for Right Motor         |
| 23   | `RIGHT_PWMA`         | Motor Driver         | PWM Signal for Right Motor Speed            |
| 5    | `PIEZO_PIN`          | Piezo Buzzer         | Output for Sound Generation                 |
| 21   | `I2C_SDA_PIN`        | PCA9685              | I2C Data Line                               |
| 22   | `I2C_SCL_PIN`        | PCA9685              | I2C Clock Line                              |
| 16   | `SerialCommand` (RX) | ESP32-CAM (TX)       | UART2 RX - Receiving commands from CAM      |
| 17   | `SerialCommand` (TX) | ESP32-CAM (RX)       | UART2 TX - Sending commands/data to CAM     |
| 10   | `LEFT_RGB_RED`       | *(PCA9685)*          | *(Pin definition remains, control via PCA)* |
| 9    | `LEFT_RGB_GREEN`     | *(PCA9685)*          | *(Pin definition remains, control via PCA)* |
| -    | `LEFT_RGB_BLUE`      | *(PCA9685)*          | *(Pin definition remains, control via PCA)* |
| -    | `RIGHT_RGB_RED`      | *(PCA9685)*          | *(Pin definition remains, control via PCA)* |
| -    | `RIGHT_RGB_GREEN`    | *(PCA9685)*          | *(Pin definition remains, control via PCA)* |
| -    | `RIGHT_RGB_BLUE`     | *(PCA9685)*          | *(Pin definition remains, control via PCA)* |

**Note:** Pins 10, 9, 13, 18, 19, 21 were originally defined for direct RGB LED control but are now superseded by the PCA9685 driver. Some of these original pins (18, 19, 21) have been repurposed for motor control or I2C. Pin 13 is now free. Pins 10 and 9 remain defined but unused directly.

## PCA9685 Connections

*   **I2C:** Connected to ESP32 GPIO 21 (SDA) and GPIO 22 (SCL).
*   **Address:** Assumed default (0x40).
*   **Channel Assignments:**
    *   Channel 0: Left LED Red
    *   Channel 1: Left LED Green
    *   Channel 2: Left LED Blue
    *   Channel 3: Right LED Red
    *   Channel 4: Right LED Green
    *   Channel 5: Right LED Blue
    *   Channel 6: Pan Servo (SG90)
    *   Channel 7: Tilt Servo (SG90)

## Power Considerations

*   **Main Power Source:** 2 x 18650 Li-ion batteries connected in series (providing ~7.4V nominal).
*   **Voltage Regulation:** The battery output is fed into an XL6009 DC-DC Boost Converter module, adjusted to provide a stable 5V output. This 5V rail powers the ESP32, Motor Driver (logic), PCA9685 (logic and V+ for servos), and the ESP32-CAM.
*   **ESP32 Power:** The ESP32 DevKit V1 is powered via its 5V input pin from the regulated 5V rail.
*   **Motor Driver Power:** The motor driver's logic (VCC) is powered by the 5V rail. The motor power input (VMotor) should also be connected to the 5V rail (assuming the motors are rated for 5V). Ensure grounds are connected.
*   **PCA9685 Power:** The PCA9685 logic (VCC) and servo power (V+) are connected to the 5V rail. Ensure grounds are connected.
*   **SG90 Servos:** Powered by the 5V rail via the PCA9685 V+ pin. Ensure grounds are connected.
*   **ESP32-CAM:** Powered by the 5V rail. Ensure grounds are connected.
*   **Grounding:** It is crucial that all components share a common ground connection.

## ESP32-CAM Pin Connections (`cam/cam.ino`)

The AI-Thinker ESP32-CAM module uses specific GPIOs for the camera interface:

| GPIO | Function        | Notes                     |
| :--- | :-------------- | :------------------------ |
| 32   | `PWDN_GPIO_NUM` | Camera Power Down         |
| -1   | `RESET_GPIO_NUM`| Not used                  |
| 0    | `XCLK_GPIO_NUM` | Camera Clock Input        |
| 26   | `SIOD_GPIO_NUM` | Camera I2C Data (SCCB)    |
| 27   | `SIOC_GPIO_NUM` | Camera I2C Clock (SCCB)   |
| 35   | `Y9_GPIO_NUM`   | Camera Data Bit 9         |
| 34   | `Y8_GPIO_NUM`   | Camera Data Bit 8         |
| 39   | `Y7_GPIO_NUM`   | Camera Data Bit 7         |
| 36   | `Y6_GPIO_NUM`   | Camera Data Bit 6         |
| 21   | `Y5_GPIO_NUM`   | Camera Data Bit 5         |
| 19   | `Y4_GPIO_NUM`   | Camera Data Bit 4         |
| 18   | `Y3_GPIO_NUM`   | Camera Data Bit 3         |
| 5    | `Y2_GPIO_NUM`   | Camera Data Bit 2         |
| 25   | `VSYNC_GPIO_NUM`| Camera Vertical Sync      |
| 23   | `HREF_GPIO_NUM` | Camera Horizontal Reference |
| 22   | `PCLK_GPIO_NUM` | Camera Pixel Clock        |

**Serial Communication:**
*   GPIO 16: UART2 RX (Connected to Main ESP32 TX - GPIO 17)
*   GPIO 17: UART2 TX (Connected to Main ESP32 RX - GPIO 16)

## Software Dependencies

*   The `cam/cam.ino` sketch requires the `base64` library by Densaugeo: `https://github.com/Densaugeo/base64_arduino`
