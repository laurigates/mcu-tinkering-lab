---
allowed-tools: Bash(mkdir:*), Bash(cp:*), Bash(idf.py:*), Write, Edit
argument-hint: <name> <platform>
description: Create a new MCU project from template
---

## Task
Create a new project named "$1" for platform "$2"

## Supported Platforms
- `esp32` - ESP-IDF based ESP32 project
- `esp32-cam` - ESP32-CAM specific project
- `arduino` - Arduino platform project
- `stm32` - STM32 platform project

## Project Structure
Create the following structure based on platform:

### ESP32/ESP32-CAM
```
packages/esp32-projects/$1/
├── main/
│   ├── CMakeLists.txt
│   ├── main.c
│   └── Kconfig.projbuild
├── CMakeLists.txt
├── sdkconfig.defaults
└── README.md
```

### Arduino
```
packages/arduino-projects/$1/
├── src/
│   └── main.cpp
├── include/
├── lib/
├── platformio.ini
└── README.md
```

### STM32
```
packages/stm32-projects/$1/
├── src/
│   └── main.c
├── include/
├── Makefile
└── README.md
```

## Steps
1. Create directory structure
2. Generate template files with proper boilerplate
3. Add project to root Makefile if ESP32
4. Print next steps for the user
