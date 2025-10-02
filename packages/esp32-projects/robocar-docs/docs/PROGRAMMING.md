# RoboCar Programming Instructions

This document provides instructions for compiling and uploading code to the RoboCar's two ESP32 boards: the main controller and the ESP32-CAM module.

## Hardware Requirements

- ESP32 Development Board (Main Controller)
- ESP32-CAM Module 
- USB-to-Serial adapter for the ESP32-CAM (since it doesn't have a USB port)
- Micro USB cable for the main ESP32
- Jumper wires

## Software Requirements

- Arduino IDE (1.8.x or later) or Arduino CLI
- ESP32 board support package installed
- Required libraries:
  - IRremote
  - ArduinoJson (for the ESP32-CAM)
  - ESP32 Camera library

## Preparing Your Environment

### Installing ESP32 Board Support

1. In Arduino IDE, go to File → Preferences
2. Add this URL to the "Additional Boards Manager URLs" field:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Go to Tools → Board → Boards Manager
4. Search for "esp32" and install the ESP32 package by Espressif Systems

### Installing Required Libraries

In Arduino IDE, go to Tools → Manage Libraries and install:
- IRremote
- ArduinoJson
- ESP32 Camera support (if not included in board package)

## Programming the Main ESP32 Controller

### Using Arduino IDE

1. Open `robocar_main/robocar_main.ino` in Arduino IDE
2. Select Tools → Board → ESP32 → ESP32 Dev Module
3. Select the correct port under Tools → Port
4. Click the Upload button (or press Ctrl+U/Cmd+U)

### Using Arduino CLI

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32doit-devkit-v1 /Users/lgates/Documents/Arduino/robocar_test/robocar_main

# Upload (replace [PORT] with your actual port)
arduino-cli upload -p [PORT] --fqbn esp32:esp32:esp32doit-devkit-v1 /Users/lgates/Documents/Arduino/robocar_test/robocar_main
```

## Programming the ESP32-CAM Module

### Hardware Connection for Programming

The ESP32-CAM needs to be connected to a USB-to-Serial adapter as follows:

| ESP32-CAM Pin | USB-to-Serial Adapter |
|---------------|------------------------|
| 5V/VCC        | 5V/VCC                 |
| GND           | GND                    |
| U0R (GPIO3)   | TX                     |
| U0T (GPIO1)   | RX                     |

Additionally, for programming mode:
- Connect GPIO0 to GND during programming
- Some adapters include a BOOT button for this purpose

### Using Arduino IDE

1. Open `cam/cam.ino` in Arduino IDE
2. Select Tools → Board → ESP32 → AI Thinker ESP32-CAM
3. Select Tools → Partition Scheme → "Huge APP (3MB No OTA/1MB SPIFFS)"
4. Select the correct port under Tools → Port
5. Connect GPIO0 to GND (or press and hold the BOOT button)
6. Press the RESET button on the ESP32-CAM
7. Click the Upload button in Arduino IDE
8. When you see "Connecting..." message, release the BOOT button/disconnect GPIO0 from GND
9. After programming completes, press RESET again to start the program

### Using Arduino CLI

```bash
# Compile
arduino-cli compile --fqbn esp32:esp32:esp32cam /Users/lgates/Documents/Arduino/robocar_test/cam

# Upload (replace [PORT] with your actual port)
arduino-cli upload -p [PORT] --fqbn esp32:esp32:esp32cam /Users/lgates/Documents/Arduino/robocar_test/cam
```

Remember to follow the same GPIO0-to-GND and reset procedure as described in the Arduino IDE section.

## Using the Makefile (Recommended Command-Line Method)

The project includes a `Makefile` that simplifies the command-line build and upload process using `arduino-cli` and `esptool.py`. Ensure you have `make` installed on your system (common on macOS and Linux, may need installation on Windows).

1.  **Navigate to the project root directory** in your terminal.
2.  **Set up the environment (first time only):**
    ```bash
    make setup
    ```
3.  **Build the code for both boards:**
    ```bash
    make build
    ```
    Or build individually:
    ```bash
    make build-main
    make build-camera
    ```
4.  **Upload to the main ESP32:**
    Find your port first using `make list-ports`. Then:
    ```bash
    make upload PORT=/dev/cu.your-main-esp32-port
    ```
    (Replace `/dev/cu.your-main-esp32-port` with the actual port). If you don't specify `PORT`, it will use the default `/dev/cu.SLAB_USBtoUART`.
5.  **Upload to the ESP32-CAM:**
    Connect the ESP32-CAM via your USB-to-Serial adapter. Find the port using `make list-ports`. Then:
    ```bash
    make upload-camera PORT=/dev/cu.your-cam-port
    ```
    (Replace `/dev/cu.your-cam-port` with the actual port). Remember to follow the on-screen prompts regarding GPIO0 connection.
6.  **Monitor serial output:**
    ```bash
    make monitor PORT=/dev/cu.your-port
    ```
7.  **Clean build files:**
    ```bash
    make clean
    ```
8.  **See all available commands:**
    ```bash
    make help
    ```

## Checking Your Upload

After uploading, you can monitor the output of either board:

### Using Arduino IDE
- Open Tools → Serial Monitor
- Set baud rate to 115200

### Using Arduino CLI
```bash
arduino-cli monitor -p [PORT] -c baudrate=115200
```

## Troubleshooting

### Common Issues with ESP32 Main Controller

- **Upload Fails**: Press and hold the BOOT button on your ESP32 board while the upload starts
- **Board Not Found**: Check your USB cable and try a different USB port
- **Wrong Driver**: Some ESP32 boards use CH340 or CP210x USB chips which may need drivers

### Common Issues with ESP32-CAM

- **Cannot Enter Programming Mode**: Double-check GPIO0 is properly connected to GND during upload
- **Brown-out Detected**: Ensure your power supply provides enough current (at least 500mA)
- **Camera Init Failed**: Check camera cable connection; it may be loose or inserted backward
- **Upload Seems to Work but Camera Doesn't Initialize**: Make sure you've selected the correct partition scheme

### Serial Communication Issues

If the two boards aren't communicating:
1. Verify the wiring between ESP32-CAM and main ESP32 (TX → RX, RX → TX)
2. Confirm both are using the same baud rate (115200)
3. Check the RX/TX pin definitions in both code files match the physical connections
