# Motor Position Control System

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
  - [Pin Configuration](#pin-configuration)
- [Setup Instructions](#setup-instructions)
- [Usage](#usage)
  - [Example Serial Commands](#example-serial-commands)
- [Code Logic](#code-logic)
- [Sensor Logic](#sensor-logic)
  - [Sensor Code](#sensor-code)
  - [Sensor Logic Details](#sensor-logic-details)
  - [Sensor Requirements](#sensor-requirements)
- [Code Structure](#code-structure)
  - [Constants and Pin Definitions](#constants-and-pin-definitions)
  - [Functions](#functions)
- [Troubleshooting](#troubleshooting)
  - [Issue: Downward Movement Not Working](#issue-downward-movement-not-working)
  - [Other Issues](#other-issues)
- [Notes](#notes)
- [License](#license)
- [Contributing](#contributing)

## Overview
This Arduino project controls a DC motor with precise position tracking using a sensor and limit switches. The system employs a PID controller and trapezoidal velocity profiles to move the motor to user-specified positions (0 to 30 steps) via serial input. It includes calibration to a known position, limit switch handling to prevent over-travel, and EEPROM storage for persistent position data. The motor is driven by a motor driver (e.g., L298N, VNH2SP30), supporting both upward and downward movements with varying speeds for smooth operation.

## Features
- **Position Control**: Moves the motor to a target position (0–30 steps) specified via serial input.
- **PID Control**: Adjusts motor speed using proportional, integral, and derivative terms for precise positioning.
- **Trapezoidal Velocity Profiles**: Implements soft start, high-speed, and medium-speed phases for upward and downward movements.
- **Limit Switches**: Prevents over-travel using upper and lower limit switches.
- **Calibration**: Initializes the motor to the lower limit (`pos = 0`) and then moves to position 15 (`INITIAL_POS`).
- **EEPROM Storage**: Saves the current position to retain state across power cycles.
- **Sensor-Based Tracking**: Uses an interrupt-driven sensor to track motor steps.(IR Encoder Speed Motion Sensor Module)

## Hardware Requirements
- **Arduino Board**: Compatible with Arduino Uno, Mega, or similar (tested with pins supporting interrupts, e.g., pin 18 on Mega).
- **Motor Driver**: VNH2SP30 or similar with PWM support (connected to pins `IN1`, `IN2`, `PWM`, `EN`).
- **DC Motor**: Connected to the motor driver.
- **Sensor**: IR Encoder Speed Motion Sensor Module (optical: didode emitter and receiver) connected to `SENSOR_PIN` (pin 18).
- **Limit Switches**: Two switches connected to `UPPER_LIMIT_PIN` (pin 7) and `LOWER_LIMIT_PIN` (pin 6) with internal pull-up resistors.
- **Power Supply**: Adequate voltage and current for the motor and driver (e.g., 12V for VNH2SP30).
- **Serial Monitor**: For sending target positions and viewing debug output (9600 baud).

### Pin Configuration
| Pin Name          | Arduino Pin | Description                     |
|-------------------|-------------|---------------------------------|
| `SENSOR_PIN`      | 18          | Position sensor input (interrupt) |
| `UPPER_LIMIT_PIN` | 7           | Upper limit switch (active LOW) |
| `LOWER_LIMIT_PIN` | 6           | Lower limit switch (active LOW) |
| `PWM`             | 10          | PWM output for motor speed      |
| `IN1`             | 27          | Motor driver input 1            |
| `IN2`             | 29          | Motor driver input 2            |
| `EN`              | A0          | Motor driver enable             |

## Setup Instructions
1. **Hardware Connections**:
   - Connect the motor driver to the Arduino as per the pin configuration.
   - Wire the position sensor to `SENSOR_PIN` (ensure it supports interrupts).
   - Connect limit switches to `UPPER_LIMIT_PIN` and `LOWER_LIMIT_PIN` with pull-up resistors (internal pull-ups enabled).
   - Ensure the motor driver is powered with an appropriate voltage supply.

2. **Software Setup**:
   - Install the Arduino IDE (version 1.8.x or later).
   - Copy the provided code into a new Arduino sketch.
   - Include the `EEPROM` library (built-in with Arduino IDE).

3. **Upload Code**:
   - Connect the Arduino to your computer via USB.
   - Upload the sketch to the Arduino board.

4. **Serial Monitor**:
   - Open the Serial Monitor in the Arduino IDE (set to 9600 baud).
   - The system will calibrate on startup, moving to the lower limit and then to position 15.

## Usage
- **Calibration**: On startup, the system calibrates by moving the motor to the lower limit (`pos = 0`) and then to the initial position (`INITIAL_POS = 15`). The current position is stored in EEPROM.
- **Input Target Position**:
  - In the Serial Monitor, enter a number between 0 and 30 (e.g., `10` or `25`) and press Enter.
  - The motor moves to the specified position using the appropriate direction (`moveUpward` or `moveDownward`).
  - The system outputs the current position and PWM values to the Serial Monitor.
- **Limit Switch Handling**: If a limit switch is triggered (upper or lower), the motor stops, moves slightly in the opposite direction, and recalibrates.
- **Position Persistence**: The current position is saved to EEPROM after each move, ensuring the system resumes from the last position after a power cycle.

### Example Serial Commands
- Input `25`: Moves the motor to position 25 (upward from 15).
- Input `10`: Moves the motor to position 10 (downward from 15).
- Invalid input (e.g., `31`): Outputs "Invalid position! Must be between 0 and 30".

## Code Logic
The code follows a structured approach to control the motor's position with safety and precision:

1. **Initialization (`setup`)**:
   - Configures pins for the sensor, limit switches, and motor driver.
   - Initializes serial communication (9600 baud) for user input and debug output.
   - Attaches an interrupt to `SENSOR_PIN` to track position changes.
   - Runs `calibratePosition` to set the motor to the lower limit (`pos = 0`) and then to `INITIAL_POS = 15`.
   - Loads the last saved position from EEPROM (address 0).

2. **Main Loop (`loop`)**:
   - Reads serial input for a target position.
   - Validates the input (must be between `minPos = 0` and `maxPos = 30`).
   - Calls `moveToPosition` to move the motor.
   - Continuously checks limit switches and calls `handleLimitHit` if triggered.

3. **Position Control (`moveToPosition`)**:
   - Compares the current position (`pos`) with the target position.
   - Calls `moveUpward` if `pos < targetPos` or `moveDownward` if `pos > targetPos`.
   - Stops the motor and updates EEPROM after reaching the target.

4. **Movement Functions**:
   - **Upward Movement (`moveUpward`)**:
     - Uses a trapezoidal velocity profile:
       - 0–5 steps: Soft start (PWM ramps from `MIN_SPEED = 60` to `UPWARD_MEDIUM_SPEED = 90`).
       - 5–20 steps: High speed (`UPWARD_MAX_SPEED = 100`).
       - 20–30 steps: Medium speed (`UPWARD_MEDIUM_SPEED = 90`).
     - Applies PID control to adjust PWM based on position error.
     - Checks the upper limit switch to prevent over-travel.
   - **Downward Movement (`moveDownward`)**:
     - Uses a trapezoidal velocity profile:
       - 30–25 steps: Soft start (PWM ramps from 40 to `DOWNWARD_LOW_SPEED = 50`).
       - 25–10 steps: Low speed (`DOWNWARD_LOW_SPEED = 50`).
       - 10–0 steps: High speed (`DOWNWARD_MAX_SPEED = 80`).
     - Applies PID control to fine-tune PWM.
     - Checks the lower limit switch to prevent over-travel.

5. **Limit Handling (`handleLimitHit`)**:
   - Stops the motor if a limit switch is triggered.
   - Moves slightly in the opposite direction (500ms at low/medium speed).
   - Triggers recalibration to reset the position.

6. **Calibration (`calibratePosition`)**:
   - Moves the motor downward at PWM 60 until the lower limit switch is triggered.
   - Sets `pos = 0` and `minPos = 0`.
   - Moves to `INITIAL_POS = 15` and saves the position to EEPROM.

7. **EEPROM Usage**:
   - Stores the current position at address 0 after each move.
   - Loads the position on startup to maintain state.

## Sensor Logic
The IR Encoder Speed Motion Sensor Module as a position sensor generates pulses as the motor moves, which are counted to track the motor's position. The `readSensor` function is triggered on every change (rising or falling edge) of the sensor signal via an interrupt on `SENSOR_PIN` (pin 18). The logic includes debouncing to filter noise and checks the motor direction to increment or decrement the position.

### Sensor Code
```cpp
void readSensor() {
  static int lastState = HIGH;
  static unsigned long lastInterruptTime = 0;
  const unsigned long debounceDelay = 40;

  int currentState = digitalRead(SENSOR_PIN);
  unsigned long currentTime = millis();

  if ((currentState != lastState) && (currentTime - lastInterruptTime >= debounceDelay)) {
    if (digitalRead(IN1) == LOW && digitalRead(IN2) == HIGH) {
      if (digitalRead(LOWER_LIMIT_PIN) == HIGH) {
        pos = pos - 1; // Decrement for downward motion
      }
    } else if (digitalRead(IN1) == HIGH && digitalRead(IN2) == LOW) {
      if (digitalRead(UPPER_LIMIT_PIN) == HIGH) {
        pos = pos + 1; // Increment for upward motion
      }
    }
    lastInterruptTime = currentTime;
  }
  lastState = currentState;
}
```

### Sensor Logic Details
- **Interrupt Trigger**: The function is called on `CHANGE` (rising or falling edge) of `SENSOR_PIN`.
- **Debouncing**: A 40ms debounce delay (`debounceDelay`) ensures noise doesn’t cause false triggers.
- **Direction Detection**:
  - Downward motion: `IN1 = LOW`, `IN2 = HIGH`, decrements `pos` if the lower limit switch is not triggered.
  - Upward motion: `IN1 = HIGH`, `IN2 = LOW`, increments `pos` if the upper limit switch is not triggered.
- **Limit Switch Check**: Prevents position updates when a limit switch is active to avoid erroneous counts.
- **Position Update**: The global `pos` variable tracks the motor’s position in steps (0–30).

### Sensor Requirements
- The sensor must generate clean pulses (e.g., 1 pulse per motor step).
- The sensor pin (18) must support interrupts (check your Arduino board’s interrupt-capable pins).
- If the sensor is noisy, adjust `debounceDelay` (e.g., 20ms or 50ms) to balance responsiveness and stability.

## Code Structure
### Constants and Pin Definitions
- **Pin Definitions**: Defines pins for sensor, limit switches, and motor driver (`SENSOR_PIN`, `UPPER_LIMIT_PIN`, `LOWER_LIMIT_PIN`, `PWM`, `IN1`, `IN2`, `EN`).
- **Position Limits**: `maxPos = 30`, `minPos = 0`, `INITIAL_POS = 15`.
- **Speed Parameters**:
  - Upward: `UPWARD_MAX_SPEED = 100`, `UPWARD_MEDIUM_SPEED = 90`, `MIN_SPEED = 60`.
  - Downward: `DOWNWARD_LOW_SPEED = 50`, `DOWNWARD_MAX_SPEED = 80`.
- **Control Parameters**: `ACCEL_STEPS = 5` for soft start, `DOWNWARD_DELAY = 10` for downward motion delay, PID gains (`Kp = 0.5`, `Ki = 0.01`, `Kd = 0.1`).

### Functions
- `setup()`: Initializes serial communication, pin modes, and interrupts. Performs calibration and loads position from EEPROM.
- `loop()`: Reads serial input for target positions, validates input, and calls `moveToPosition`. Monitors limit switches.
- `moveToPosition(int targetPos)`: Directs the motor to move upward or downward based on the current position (`pos`) and target.
- `moveUpward(int targetPos)`: Moves the motor upward with a trapezoidal velocity profile and PID control.
- `moveDownward(int targetPos)`: Moves the motor downward with a trapezoidal velocity profile and PID control.
- `applyBraking(int delayTime)`: Applies dynamic braking by setting both motor inputs high and PWM to 0.
- `setMotor(int dir, int pwmVal, int pwm, int in1, int in2)`: Controls motor direction and speed via the motor driver.
- `readSensor()`: Interrupt-driven function to update `pos` based on sensor pulses, with debouncing.
- `handleLimitHit()`: Stops the motor, moves slightly away from the limit, and recalibrates when a limit switch is triggered.
- `calibratePosition()`: Moves the motor to the lower limit, sets `pos = 0`, and moves to `INITIAL_POS = 15`.

## Troubleshooting
### Issue: Downward Movement Not Working
- **Symptom**: Inputting a lower position (e.g., 10 from 15) results in no movement, while upward movement (e.g., to 25) works.
- **Possible Causes**:
  - **Low PWM Value**: `DOWNWARD_LOW_SPEED = 50` may be too low for the motor to move under load.
  - **Sensor Issues**: The position sensor may not trigger correctly, preventing `pos` updates in `readSensor`.
  - **Braking**: Frequent braking (`applyBraking(10)`) in `moveDownward` may stall the motor.
  - **PID Tuning**: Incorrect PID gains may reduce PWM below the motor’s operating threshold.
- **Solutions**:
  - Increase `DOWNWARD_LOW_SPEED` to 80:
    ```cpp
    const int DOWNWARD_LOW_SPEED = 80;
    ```
  - Comment out `applyBraking(10)` in `moveDownward` to test without braking:
    ```cpp
    setMotor(-1, pwmVal, PWM, IN1, IN2);
    // applyBraking(10); // Comment out
    ```
  - Add debug output in `readSensor` to verify `pos` updates:
    ```cpp
    Serial.print("Sensor triggered, IN1: ");
    Serial.print(digitalRead(IN1));
    Serial.print(", IN2: ");
    Serial.println(digitalRead(IN2));
    ```
  - Check the lower limit switch state in `moveDownward`:
    ```cpp
    Serial.print("Lower Limit Pin: ");
    Serial.println(digitalRead(LOWER_LIMIT_PIN));
    ```
  - Verify motor driver and sensor wiring. Ensure the power supply provides sufficient current.

### Other Issues
- **Motor Doesn’t Move**: Check power supply voltage, motor driver connections, and enable pin (`EN`).
- **Position Drifts**: Adjust `debounceDelay` in `readSensor` (e.g., 20ms or 50ms) to handle sensor noise.
- **Limit Switch Errors**: Ensure switches are wired correctly (active LOW) and not stuck.
- **Sensor Not Triggering**: Verify sensor wiring and signal quality with an oscilloscope or multimeter.

## Notes
- The code related to  a motor driver with PWM control and direction pins (e.g. VNH2SP30 monstor shield ). Adjust pin definitions for other drivers.
- The position sensor must generate clean pulses (e.g., 1 pulse per motor step).
- The trapezoidal velocity profile and PID gains may need tuning for specific motors and loads.
- EEPROM usage is minimal (address 0 for position), but ensure no other sketches overwrite this address.
- The initial position is set to 15 steps after calibration, as defined by `INITIAL_POS`.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for bug reports or feature suggestions.