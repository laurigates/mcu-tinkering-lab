# RoboCar ESP32 Design Document

## Overview

This document outlines the design decisions made in the implementation of the RoboCar ESP32 firmware. The RoboCar is a remote-controlled robot that can be operated via an IR remote or through "AI" commands received via serial communication from an ESP32-CAM module.

## System Architecture

The system consists of the following primary components:

- ESP32 microcontroller (main controller)
- Dual DC motors with H-bridge driver
- IR receiver for remote control
- RGB LED for status indication
- Buzzer for audio feedback
- Serial communication with ESP32-CAM

## Design Decisions

### 1. Motor Control Implementation

#### Decision
Motor control is implemented using standard Arduino `analogWrite()` function rather than ESP32-specific `ledc` functions.

#### Rationale
- **Compatibility**: The initial implementation using ESP32's `ledcSetup()` and `ledcAttachPin()` functions caused compilation errors. These functions may not be consistently available across all ESP32 Arduino core versions.
- **Simplicity**: Using `analogWrite()` simplifies the code and makes it more portable.
- **Adequacy**: The standard Arduino PWM functionality provides sufficient control for this application.

### 2. Sound Generation Approach

#### Decision
A custom sound generation function was implemented using basic digital output and timing functions rather than using `tone()` or ESP32-specific `ledcWriteTone()`.

#### Rationale
- **Compatibility**: Neither the Arduino `tone()` function nor the ESP32 `ledcWriteTone()` function was reliably available.
- **Control**: The custom implementation gives direct control over the waveform generation.
- **Reliability**: Using fundamental functions like `digitalWrite()` and `delayMicroseconds()` ensures compatibility across different Arduino core versions.

Implementation detail:
```arduino
void playSound(int frequency, int duration) {
  unsigned long period = 1000000 / frequency; // Period in microseconds
  unsigned long startTime = millis();

  isSoundPlaying = true;
  soundStartTime = startTime;
  soundDuration = duration;

  while (millis() - startTime < duration) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(period / 2);
  }

  digitalWrite(BUZZER_PIN, LOW);
}
```

### 3. Dual Control Modes

#### Decision
The system supports two control modes:
1. IR Remote Control (default)
2. AI Control via Serial

#### Rationale
- **Flexibility**: Allows for both manual control and autonomous operation.
- **Development Path**: Enables incremental development from manual to autonomous control.
- **Debugging**: Manual control provides a way to verify hardware functionality independently of the AI system.

### 4. Safety Features

#### Decision
Implemented a command timeout system that automatically stops the motors if no commands are received within a specified period.

#### Rationale
- **Safety**: Prevents the robot from continuing movement if communication is lost.
- **Predictability**: Creates deterministic behavior in case of control system failures.
- **Battery Conservation**: Prevents battery drain from unintended continued operation.

### 5. State-Based LED Feedback

#### Decision
The RGB LED changes color based on the current movement state of the robot.

#### Rationale
- **User Feedback**: Provides immediate visual indication of the robot's current state.
- **Debugging Aid**: Helps in troubleshooting by making the robot's state visible.
- **Engagement**: Enhances user interaction through visual feedback.

### 6. Command Processing Architecture

#### Decision
Commands are processed through dedicated handler functions that update a global state variable.

#### Rationale
- **Modularity**: Separates control logic from movement implementation.
- **State Awareness**: Maintains awareness of current state across control modes.
- **Extensibility**: Makes it easy to add new commands or control modes.

## Technical Constraints and Tradeoffs

### PWM Frequency and Resolution

- Using standard Arduino PWM functions means accepting the default PWM frequency and resolution.
- This is adequate for motor control but may be suboptimal for certain sound generation applications.

### Blocking Sound Generation

- The current sound generation approach blocks execution during tone generation.
- This was chosen for simplicity and reliability but means the robot cannot move while generating sounds.
- A more advanced implementation could use interrupts for non-blocking sound generation.

### Common Anode RGB LED

- The implementation assumes a common anode RGB LED where LOW signals activate the LEDs.
- This is a hardware-specific decision that would need to be changed if different LED types are used.

## Future Enhancements

1. **Non-blocking Sound Generation**: Implement interrupt-based sound generation for concurrent operation.
2. **PID Motor Control**: Add speed feedback and PID control for more precise movement.
3. **Configurable Parameters**: Make timeout periods, motor speeds, and other parameters configurable at runtime.
4. **Advanced Morse Code**: Implement a full Morse code translator for the sound system.
5. **Power Management**: Add battery monitoring and power-saving modes.

## Conclusion

The RoboCar ESP32 firmware design prioritizes compatibility, reliability, and simplicity while providing a flexible platform for both manual and autonomous control. The design choices favor standard Arduino API functions over ESP32-specific ones to ensure broader compatibility across different Arduino core versions.
