# NEMA 23 Stepper Motor (23HS5628) & TB6600 Driver — Documentation

---

## Table of Contents

1. [Introduction](#introduction)  
2. [Motor Specifications](#motor-specifications)  
3. [Winding Details & Wire Color Code](#winding-details--wire-color-code)  
4. [🧪 Testing the Wire Pairs with a Multimeter](#testing-the-wire-pairs-with-a-multimeter)  
5. [Microstep Driver Overview](#microstep-driver-overview)  
   - [Key Specifications](#key-specifications)  
   - [Microstepping & Current Limit (DIP Switches)](#microstepping--current-limit-dip-switches)  
   - [SW Control Signals & Strategies](#sw-control-signals--strategies)  
6. [Wiring Diagrams](#wiring-diagrams)  
   - [Motor → TB6600](#motor--tb6600)  
   - [TB6600 → Arduino Uno](#tb6600--arduino-uno)  
   - [Power Supply → TB6600](#power-supply--tb6600)  
7. [Basic Arduino Test Sketch](#basic-arduino-test-sketch)  
8. [Troubleshooting & Tips](#troubleshooting--tips)  

---

 ![][stepperimage]
 

## Introduction

The **23HS5628** is a NEMA 23 bipolar stepper motor (frame size 57×56 mm) with a 6.35 mm or 8 mm D-cut shaft. It delivers high holding torque and is widely used in CNC machines, 3D printers, robotics, and automation.

---

## Motor Specifications

| Parameter               | Value                 |
|-------------------------|-----------------------|
| Step Angle              | 1.8° (200 steps/rev)  |
| Rated Current           | 2.8 A/phase           |
| Phase Resistance        | 0.9 Ω                 |
| Phase Inductance        | 2.5 mH                |
| Holding Torque          | 126 N·cm (12.6 kg·cm) |
| Insulation Resistance   | ≥ 100 MΩ @ 500 VDC    |
| Insulation Strength     | 500 VAC, 1 min        |
| Ambient Temp. Range     | −20 °C to +50 °C      |
| Shaft Diameter          | 6.35 mm or 8 mm “D”   |
| Cable                   | 4-wire, ~30 cm lead   |

---

## Winding Details & Wire Color Code

This motor is **bipolar**, meaning two independent coils (phases) each with two leads:

| Wire Color | Coil/Phase | Polarity |
|------------|------------|----------|
| **Black**  | Phase A    | A+       |
| **Green**  | Phase A    | A–       |
| **Red**    | Phase B    | B+       |
| **Blue**   | Phase B    | B–       |

> **Note:** Color codes can vary by vendor. Always verify with a multimeter.

---
![][image6]

## 🧪 Testing the Wire Pairs with a Multimeter

1. **Set** your multimeter to **continuity** or low-ohm resistance.  
2. **Probe** any two leads:  
   - **If** resistance ≈ 0.9 Ω (few ohms), they belong to the same coil.  
   - **If** open circuit, they are from different coils.  
3. **Identify** both pairs: one pair is Phase A, the other Phase B.  
4. **Swap** a pair’s wires in your driver’s A+ / A– to **reverse rotation**.

---

## Microstep Driver Overview

The **TB6600** is a popular, rugged microstepping driver for bipolar steppers up to ~4.5 A. It accepts Step/Dir/Enable inputs and features selectable microstep resolution via DIP switches.

 ![][image2]

### Key Specifications

| Parameter              | Value                         |
|------------------------|-------------------------------|
| Supply Voltage (VCC)   | 9–42 V DC                     |
| Output Current         | 0.5–4.5 A (adjustable)        |
| Microstep Resolutions  | Full, 1/2, 1/4, 1/8, 1/16, 1/32 |
| Logic Input Levels     | 5 V TTL (Step/Dir/ENA)        |
| Isolation              | Opto-isolated inputs          |
| Protection             | Over-current, over-temp, under-voltage |

---

### Microstepping & Current Limit (DIP Switches)

| SW1–SW3 (Microstep)     | Resolution | SW4–SW6 (Current) | I_OUT per Phase |
|-------------------------|------------|-------------------|-----------------|
| OFF OFF OFF             | Full-step  | OFF OFF OFF       | 500 mA          |
| ON  OFF OFF             | 1/2-step   | ON  OFF OFF       | 1000 mA         |
| OFF ON  OFF             | 1/4-step   | OFF ON  OFF       | 1500 mA         |
| ON  ON  OFF             | 1/8-step   | ON  ON  OFF       | 2000 mA         |
| OFF OFF ON              | 1/16-step  | OFF OFF ON        | 2500 mA         |
| ON  OFF ON              | 1/32-step  | ON  OFF ON        | 3000 mA         |
| *…and so on*            | *…*        | *…*               | *…*             |

> ➤ **Set current** to `~2.8 A` for 23HS5628.  
> ➤ **Recommend** `1/8 or 1/16 `microstepping for smooth motion.

---
 ![][image3]
 ![][image4]


### SW Control Signals & Strategies

- **PUL+ / PUL–**: Step pulses. Rising edge = one microstep.  
- **DIR+ / DIR–**: Direction logic. `High/Low` selects `CW/CCW`.  
- **ENA+ / ENA–**: Enable (active LOW on many boards). Tie high or leave open if unused.

– **Polarity:** All `inputs` are `opto-isolated`; tie the `“−”` pins to Arduino `GND`.  
– **Signal Timing:** Min. pulse width `2.5 µs`; max. `500 kHz` step rate.

---

## Wiring Diagrams

### Motor → TB6600

```
   Motor Coil A     ┐
 Black ────────── A+ │
 Green ────────── A– │
 Red   ────────── B+ │ TB6600
 Blue  ────────── B– │
                   ┘
```
> **Note:** Color codes can vary by vendor. Always verify with a multimeter.

### TB6600 → Arduino Uno

![][image5]

| TB6600 Pin | Arduino Pin | Signal   |
|------------|-------------|----------|
| PUL+       | D3          | STEP     |
| PUL–       | GND         | Ground   |
| DIR+       | D4          | DIRECTION|
| DIR–       | GND         | Ground   |
| ENA+       | D5          | ENABLE   |
| ENA–       | GND         | Ground   |

### Power Supply → TB6600

| TB6600 Pin | Connection             |
|------------|------------------------|
| VCC (+)    | 24 V DC supply (+)     |
| GND (−)    | 24 V DC supply (−)     |

---

## Basic Arduino Test Sketch

```cpp
#include <AccelStepper.h>

// STEP = D3, DIR = D4
AccelStepper stepper(AccelStepper::DRIVER, 3, 4);

void setup() {
  stepper.setSpeed(500);  // 500 steps/sec
}

void loop() {
  stepper.runSpeed();     // Continuous rotation
}
```

1. Upload this sketch  
2. Power on TB6600 with motor connected  
3. Motor should spin at constant speed

---

## Troubleshooting & Tips

- **No motion?** Check these once again:
   - `driver DIP switches`,
   - `power supply`,
   - `coil wiring`, and
   - `enable pin`.  
   
- **Overheating?** Ensure `current limit` is correctly set and add a heatsink/fan.  
- **Vibration or noise?** Increase microstepping `resolution or reduce speed`.  
- **Reversed direction?** Swap either `A+↔A– or B+↔B– `(or use negative speed in code).  


![][video]
Watch out the video and images in Image folder.



[stepperimage]:/Stepper_motor_with_Microstep_Driver//images/stepperimage.jpg
[image2]:/Stepper_motor_with_Microstep_Driver/images/driverimage.jpg
[image3]:/Stepper_motor_with_Microstep_Driver/images/dipswitch.jpg
[image4]:/Stepper_motor_with_Microstep_Driver/images/currentsizesetting.jpg
[image5]:/Stepper_motor_with_Microstep_Driver/images/Wiring_image.jpeg
[image6]:/Stepper_motor_with_Microstep_Driver/images/stepper-motor-wiring.jpg
[video]:/Stepper_motor_with_Microstep_Driver/images/Output_Video.mp4