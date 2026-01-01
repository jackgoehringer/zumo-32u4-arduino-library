# AGENTS.md - Zumo 32U4 Arduino Library

This document provides guidance for AI coding agents working on this codebase.

## Project Overview

This is a C++ Arduino library for the Pololu Zumo 32U4 robot. The library provides
interfaces for motors, encoders, IMU sensors, proximity sensors, line sensors, LCD/OLED
displays, buttons, and buzzer. Target platform is AVR ATmega32U4.

## Build and Test Commands

### CI Build (Arduino-CI)
```bash
# Full CI build using nix (as used in GitHub Actions)
nix-shell -I nixpkgs=channel:nixpkgs-unstable -p arduino-ci --run "arduino-ci"
```

### Manual Arduino IDE Compilation
```bash
# Install Arduino CLI if needed
arduino-cli compile --fqbn arduino:avr:leonardo examples/Balancing/Balancing.ino
arduino-cli compile --fqbn arduino:avr:leonardo examples/MazeSolver/MazeSolver.ino

# Compile a specific sketch
arduino-cli compile --fqbn arduino:avr:leonardo <path-to-sketch.ino>
```

### Target Board
- Board: `arduino:avr:leonardo` (ATmega32U4-based)
- Architecture: AVR only (see `.arduino-ci.yaml`)

### Dependencies
These libraries must be installed (handled automatically by Arduino Library Manager):
- FastGPIO
- USBPause
- Pushbutton
- PololuBuzzer
- PololuHD44780
- PololuOLED
- PololuMenu

## Code Style Guidelines

### File Structure
- Library source files: `src/`
- Example sketches: `examples/<ExampleName>/<ExampleName>.ino`
- Custom sketches: `sketches/<sketchname>/<sketchname>.ino`

### Copyright Header
All source files should include the Pololu copyright:
```cpp
// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
```

### Include Guards
Use `#pragma once` for header files (not `#ifndef` guards).

### Includes
Order includes as:
1. Standard library headers (`<stdint.h>`, `<avr/io.h>`, `<stdlib.h>`)
2. Arduino headers (`<Wire.h>`, `<Arduino.h>`)
3. Library headers (`<Zumo32U4.h>`, `<FastGPIO.h>`)

In sketches, always include Wire.h before Zumo32U4.h:
```cpp
#include <Wire.h>
#include <Zumo32U4.h>
```

### Naming Conventions

**Classes**: PascalCase with `Zumo32U4` prefix
```cpp
class Zumo32U4Motors { };
class Zumo32U4IMU { };
class Zumo32U4ProximitySensors { };
```

**Methods**: camelCase
```cpp
void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);
bool gyroDataReady();
int16_t getCountsLeft();
```

**Constants/Macros**: SCREAMING_SNAKE_CASE
```cpp
#define PWM_L 10
#define LSM303D_ADDR 0b0011101
#define LSM303D_REG_WHO_AM_I 0x0F
```

**Variables**: camelCase
```cpp
float gyroOffsetY;
static bool flipLeft = false;
uint8_t lastError = 0;
```

**Enums**: PascalCase for type, PascalCase for values
```cpp
enum class Zumo32U4IMUType : uint8_t {
  Unknown,
  LSM303D_L3GD20H,
  LSM6DS33_LIS3MDL
};
```

### Types
- Use fixed-width integer types: `int16_t`, `uint8_t`, `uint16_t`, `int32_t`
- Use `bool` for boolean values (not `int` or `uint8_t` for flags)
- Motor speeds: `int16_t` range -400 to 400

### Formatting
- Braces: Allman style (opening brace on new line for functions/classes)
- Indentation: 4 spaces (no tabs)
- Inline braces for short conditionals are acceptable:
  ```cpp
  if (lastError) { return; }
  if (sensorNumber >= numSensors) { return 0; }
  ```

### Comments
- Use `/* */` for multi-line block comments at file/function level
- Use `//` for inline comments
- Use Doxygen-style comments (`/*! */`, `\brief`, `\param`, `\return`) in headers

### Class Patterns

**Lazy Initialization Pattern** (used by Motors, Encoders):
```cpp
class Zumo32U4Motors {
private:
    static inline void init() {
        static bool initialized = false;
        if (!initialized) {
            initialized = true;
            init2();
        }
    }
    static void init2();  // Actual initialization
public:
    static void setLeftSpeed(int16_t speed) {
        init();  // Ensure initialized before use
        // ... implementation
    }
};
```

**Error Handling Pattern** (used by IMU):
```cpp
writeReg(addr, reg, value);
if (lastError) { return; }
```

### Hardware Constants
- Motor PWM pins: 9 (right), 10 (left)
- Motor direction pins: 15 (right), 16 (left)
- Max motor speed: 400
- Encoder pins: 7, 8, 23, IO_E2

### Interrupt Safety
When accessing shared variables modified by ISRs, use `cli()`/`sei()`:
```cpp
cli();
int16_t counts = countLeft;
sei();
return counts;
```

### Arduino Flash Memory
Use `F()` macro for string literals to save RAM:
```cpp
display.print(F("Gyro cal"));
display.print(F("Press A"));
```

## Common Patterns

### Motor Control
```cpp
motors.setSpeeds(leftSpeed, rightSpeed);  // -400 to 400
motors.setSpeeds(0, 0);                   // Stop

// Turn in place (counterclockwise)
motors.setSpeeds(-200, 200);

// Turn in place (clockwise)
motors.setSpeeds(200, -200);
```

### IMU Reading
```cpp
imu.init();
imu.enableDefault();
imu.configureForBalancing();  // or configureForTurnSensing()

while(!imu.gyroDataReady()) {}
imu.readGyro();
// Access readings via imu.g.x, imu.g.y, imu.g.z
```

### Button Waiting
```cpp
while (!buttonA.getSingleDebouncedRelease()) {}
```

## Do Not

- Do not use architectures other than AVR
- Do not assume specific IMU chip (library auto-detects LSM303D/L3GD20H or LSM6DS33/LIS3MDL)
- Do not forget `Wire.begin()` before using IMU
- Do not exceed motor speed range of -400 to 400
- Do not use blocking delays in time-critical balancing loops
