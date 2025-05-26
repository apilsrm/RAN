# MotorControl Library for Arduino

The `MotorControl` library provides a simple interface to control a DC motor with a KY-040 rotary encoder on an Arduino platform. It supports precise motor positioning by accepting relative step commands via the Serial Monitor, with speed ramping near the target position and position limit checks.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Code Structure](#code-structure)
  - [Library Files](#library-files)
  - [Pin Definitions](#pin-definitions)
  - [Key Variables and Parameters](#key-variables-and-parameters)
  - [Main Functions](#main-functions)
- [Example Sketch](#example-sketch)
- [Debugging](#debugging)
- [Contributing](#contributing)
- [License](#license)

## Overview

The `MotorControl` library enables precise control of a DC motor's position using a KY-040 rotary encoder for feedback. The library handles motor movement with a fixed speed and a ramping mechanism near the target position to ensure smooth operation. It supports serial input for relative position commands and includes safety features like position limits.

## Features

- **Position Control**: Move the motor to a target position based on encoder steps.
- **Speed Ramping**: Reduces speed near the target for smoother stops.
- **Position Limits**: Prevents the motor from exceeding defined position boundaries.
- **Serial Interface**: Accepts relative step commands via the Serial Monitor.
- **Encoder Integration**: Uses the `KY040rotary` library for encoder handling.
- **Interrupt-Driven Encoder**: Ensures accurate position tracking.

## Hardware Requirements

- **Arduino Board** (e.g., Arduino Uno)
- **DC Motor** with a KY-040 rotary encoder
- **Motor Driver** (e.g., L298N, supporting PWM and direction control)
- **Jumper wires** and optional **breadboard**
- **Power Supply** suitable for the motor and Arduino

## Pin Configuration

The library uses the following Arduino pins:

| Pin | Function | Description |
|-----|----------|-------------|
| 9   | INA      | Motor driver input A (direction) |
| 10  | INB      | Motor driver input B (direction) |
| 5   | PWM      | PWM pin for motor speed control |
| A0  | EN       | Motor driver enable pin |
| 2   | CLK      | Encoder clock signal (interrupt pin) |
| 3   | DT       | Encoder data signal |

## Software Dependencies

- **Arduino IDE** (or compatible IDE)
- **KY040rotary Library**: Required for encoder functionality (must be installed separately)
- Standard Arduino library (included with Arduino IDE)

## Installation

1. **Install the KY040rotary Library**:
   - Download the `KY040rotary` library ( you can found it on github )  and place it in your Arduino `libraries` folder.
   - Two files `KY040rotary.cpp` and `KY040rotary.h`.
2. **Install the MotorControl Library**:
   - Clone or download this repository.
   - Copy the `MotorControl` folder (containing `MotorControl.h` and `MotorControl.cpp` and the implementation file) to your Arduino `libraries` folder.
3. **Load the Example Sketch**:
   - Open the `main.ino` sketch in the Arduino IDE.
   - Connect your Arduino board via USB.
   - Select the appropriate board and port in the Arduino IDE.
   - Upload the sketch to the Arduino.

## Usage

1. **Wiring**:
   - Connect the motor driver to the Arduino as per the [Pin Configuration](#pin-configuration).
   - Attach the KY-040 encoder's CLK and DT pins to Arduino pins 2 and 3.
   - Connect the motor driver's enable pin to A0.
   - Ensure proper power supply for the motor driver and Arduino.

2. **Running the Code**:
   - Open the Serial Monitor in the Arduino IDE (9600 baud).
   - Enter a relative step count (positive or negative) and press Enter to move the motor.
   - Example: Entering `100` moves the motor 100 encoder steps forward; `-50` moves it 50 steps backward.
   - The motor stops when it reaches the target or hits position limits (`-500` to `500`).

3. **Speed Ramping**:
   - The motor runs at a fixed speed (`50`) unless within the ramp zone (`20` steps from the target).
   - In the ramp zone, speed scales linearly from `minRampSpeed` (`20`) to `fixedSpeed` (`50`).

4. **Debug Output**:
   - The Serial Monitor displays position updates, target reached messages, or errors (e.g., position limit reached).

## Code Structure

### Library Files

- **`MotorControl.h`**:
  - Defines pin configurations, function prototypes, and global variables (e.g., `encoder`, `encoderPos`, `targetPos`, `motorRunning`).
- **Implementation File**:
  - Contains the core logic for motor control, encoder handling, and serial command processing.
- **`KY040rotary.h`**:
  - External library for managing the KY-040 encoder (not included in this repository).

### Pin Definitions

Defined in `MotorControl.h`:

- `INA` (9): Motor driver direction input A.
- `INB` (10): Motor driver direction input B.
- `PWM` (5): PWM pin for speed control.
- `EN` (A0): Motor driver enable pin.
- `CLK` (2): Encoder clock signal (interrupt-enabled).
- `DT` (3): Encoder data signal.

### Key Variables and Parameters

- **Variables**:
  - `encoderPos`: Tracks the current encoder position (volatile for interrupt safety).
  - `targetPos`: Target position for the motor.
  - `motorRunning`: Indicates if the motor is actively moving.
- **Parameters**:
  - `fixedSpeed` (50): Default motor speed.
  - `minRampSpeed` (20): Minimum speed in the ramp zone.
  - `rampZone` (20): Distance from target where speed ramping applies.
  - `targetTolerance` (0): Acceptable error for considering the target reached.
  - `maxPos` (500), `minPos` (-500): Position limits.

### Main Functions

- **`initializeMotor()`**:
  - Configures motor driver pins (INA, INB, PWM, EN) and sets the enable pin HIGH.
- **`initializeEncoder()`**:
  - Initializes the KY-040 encoder and sets up interrupt handling.
  - Registers `onEncoderRight` and `onEncoderLeft` callbacks for encoder events.
- **`moveMotor(direction, speed)`**:
  - Sets motor direction (1, -1, or 0) and speed via PWM.
- **`moveToTarget(currentPos, targetPos)`**:
  - Calculates the error and adjusts motor speed/direction.
  - Implements speed ramping and stops the motor if the target is reached or limits are hit.
- **`rotateInterruptHandler()`**:
  - Handles encoder interrupts by calling the KY-040 library's handler.
- **`onEncoderRight()`, `onEncoderLeft()`**:
  - Updates `encoderPos` within position limits and prints position to Serial.
- **`processSerialCommand()`**:
  - Reads relative step commands from Serial and updates `targetPos`.

## Example Sketch

The `main.ino` sketch demonstrates library usage:

- **Setup**:
  - Initializes serial communication (9600 baud).
  - Calls `initializeMotor()` and `initializeEncoder()`.
- **Loop**:
  - Monitors `encoderPos` for changes.
  - Processes serial commands to set new target positions.
  - Calls `moveToTarget()` when the motor is running.
  - Updates the encoder state using `encoder.Process(millis())`.

## Debugging

- **Serial Output**:
  - Position updates (`Position +: ` or `Position - : `) when the encoder moves.
  - Confirmation when the target is reached (`Target reached!`).
  - Error messages if position limits are reached (`Error: Position limit reached!`).
  - Invalid serial input warnings (`Invalid input. Enter a number.`).
- **Troubleshooting**:
  - Ensure the `KY040rotary` library is installed.
  - Verify wiring, especially for interrupt pins (2 and 3).
  - Check power supply compatibility with the motor and driver.
  - If the motor doesn't move, confirm the enable pin (A0) is HIGH and the driver is functional.

## Contributing

Contributions are welcome! To contribute:

1. Fork the repository.
2. Create a feature or bug-fix branch.
3. Submit a pull request with a clear description of your changes.

Please ensure your code follows the library's style and includes appropriate comments.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.