# DC Motor PID Control with Encoder

This project implements a PID (Proportional-Integral-Derivative) control system for a DC motor with an  HW-040 encoder using an Arduino microcontroller. The system allows precise control of the motor's position by reading user input for a target position via the Serial Monitor and adjusting motor speed and direction accordingly.

## Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Code Structure](#code-structure)
  - [Pin Definitions](#pin-definitions)
  - [Encoder Variables](#encoder-variables)
  - [PID Control Variables](#pid-control-variables)
  - [Timing Variables](#timing-variables)
  - [Speed Modulation](#speed-modulation)
- [PID Control Logic](#pid-control-logic)
- [Functions](#functions)
- [Debugging](#debugging)
- [Contributing](#contributing)
- [License](#license)

## Overview

This Arduino sketch controls a DC motor's position using a PID algorithm. An encoder provides feedback on the motor's current position, and the system adjusts the motor's speed and direction to reach a user-defined target position. The code supports serial input for setting the target position and includes optional speed modulation for fine-tuned control.

## Hardware Requirements

- **Arduino Board** (e.g., Arduino Uno)
- **DC Motor** with an  HW-040 encoder
- **Motor Driver** ( VNH2SP30 Monster)
  - Must support PWM for speed control and direction control
- **Encoder** connected to the motor shaft
- **Jumper wires** and a **breadboard** (if needed)
- **Power Supply** suitable for the motor and Arduino

## Pin Configuration

The following pins are used to interface with the motor driver and encoder:

| Pin | Function | Description |
| --- | --- | --- |
| 9 | INA | Motor driver input A (direction control) |
| 10 | INB | Motor driver input B (direction control) |
| 5 | PWM | PWM pin for motor speed control |
| A0 | EN | Enable pin for motor driver |
| 2 | CLK | Encoder clock signal (interrupt pin) |
| 3 | DT | Encoder data signal |

## Software Dependencies

- **Arduino IDE** (or compatible IDE)
- No additional libraries are required; the code uses the standard Arduino library.

## Installation

1. Clone or download this repository to your local machine.
2. Open the `.ino` file in the Arduino IDE.
3. Connect your Arduino board to your computer via USB.
4. Configure the board and port in the Arduino IDE.
   - [sudo chmod a+rw /dev/ttyACM0]
   - [ls /dev/ttyACM0]

5. Upload the sketch to your Arduino board.

## Usage

1. **Wiring**:

   - Connect the motor driver to the Arduino as per the Pin Configuration.
   - Connect the encoder's CLK and DT pins to Arduino pins 2 and 3, respectively.
   - Ensure the motor driver's enable pin is connected to A0.
   - Power the motor driver and Arduino appropriately.

2. **Running the Code**:

   - Open the Serial Monitor in the Arduino IDE (set to 9600 baud).
   - Enter a target position (in encoder steps) and press Enter.
   - The motor will move to the specified position, and debug information (target position, current position, error, and speed) will be printed to the Serial Monitor.

3. **Optional Speed Modulation**:

   - Enable speed modulation by setting `useSpeedModulation = true` in the code.
   - This toggles the motor speed every 30ms to reduce oscillations near the target.

## Code Structure

### Pin Definitions

Defines the Arduino pins connected to the motor driver and encoder:

- `INA` (9): Motor driver direction pin A.
- `INB` (10): Motor driver direction pin B.
- `PWM` (5): PWM pin for speed control.
- `EN` (A0): Motor driver enable pin.
- `pinCLK` (2): Encoder clock signal (interrupt-enabled).
- `pinDT` (3): Encoder data signal.

### Encoder Variables

- `encoderPos`: Tracks the motor's current position (volatile for interrupt safety).
- `lastCLK`: Stores the previous state of the CLK pin to detect encoder changes.

### PID Control Variables

- `targetPosition`: User-defined target position in encoder steps.
- `Kp`, `Ki`, `Kd`: PID tuning parameters (Proportional, Integral, Derivative).
- `maxSpeed` (200), `minSpeed` (50): Speed limits for motor control.
- `integral`: Accumulates error over time for integral term.
- `lastError`: Stores the previous error for derivative calculation.

### Timing Variables

- `lastControlTime`: Tracks the last PID update time.
- `controlInterval` (30ms): Time between PID updates.

### Speed Modulation

- `useSpeedModulation`: Enables/disables speed modulation (default: false).
- `lastModTime`, `speedPhase`: Manages periodic speed adjustments.

## PID Control Logic

The PID algorithm calculates the motor speed based on the error between the target and current positions:

- **Proportional (P)**: `Kp * error` adjusts speed based on the current error.
- **Integral (I)**: `Ki * integral` corrects for accumulated error over time.
- **Derivative (D)**: `Kd * derivative` reduces overshoot by considering the rate of error change.

The speed is constrained based on error zones:

- **Fast Zone** (&gt;8 steps): Speed between 100–150.
- **Cruise Zone** (5–8 steps): Speed between 80–120.
- **Approach Zone** (&lt;5 steps): Speed between 50–80.

If `useSpeedModulation` is enabled, the speed toggles ±20 every 30ms within the defined limits.

## Functions

- **setup()**:

  - Initializes serial communication (9600 baud).
  - Configures motor and encoder pins.
  - Attaches an interrupt to `pinCLK` for encoder updates.

- **loop()**:

  - Reads target position from Serial input.
  - Updates motor control every `controlInterval` (30ms).

- **readEncoder()**:

  - Interrupt-driven function to update `encoderPos` based on encoder signals.
  - Determines direction by comparing CLK and DT states.

- **moveMotor(direction, speed)**:

  - Sets motor direction (1, -1, or 0) and speed via PWM.
  - Constrains speed within 0 to `maxSpeed`.

- **updateMotor()**:

  - Calculates PID output and adjusts motor speed and direction.
  - Applies speed zones and optional modulation.
  - Outputs debug information when the motor is moving.

## Debugging

The code prints debug information to the Serial Monitor when the motor is moving (`absError > 0`):

- **Target**: The user-set target position.
- **Pos**: Current encoder position.
- **Err**: Position error (target - current).
- **Speed**: Calculated motor speed.

To troubleshoot:

- Ensure encoder pins are correctly connected and interrupt-capable (pins 2 and 3 on Uno).
- Verify motor driver wiring and power supply.
- Adjust `Kp`, `Ki`, and `Kd` for smoother control if oscillations occur.

## Contributing

Contributions are welcome! Please:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a clear description of your changes.

## License

This project is licensed under the `MIT` License. See the `LICENSE file` for details.