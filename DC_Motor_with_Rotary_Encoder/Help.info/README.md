# 🔄  Rotary Encoder and VNH2SP30

## 🧭 HW-040 Rotary Encoder (Incremental Type)
 ### 🔧 Description:
The HW-040 is a mechanical incremental rotary encoder module that outputs a series of pulses as it is rotated. It typically includes:

- A rotary shaft with detents (clicks),

- Two output pins: CLK and DT (A/B channels),

- One push-button switch (SW) (optional use),

- Common GND and VCC (3.3V or 5V).


[Rotary Encoder](/DC_Motor_with_Rotary_Encoder/Help.info/images/Rotary-Encoder-Pinout.webp)

### 📌 Pinout:


| Pin | Function           | Connect to Arduino                 |
| --- | ------------------ | ---------------------------------- |
| CLK | Channel A (output) | Digital Pin 2 (interrupt pin)      |
| DT  | Channel B (output) | Digital Pin 3 (or any digital pin) |
| SW  | Push button        | Digital Pin (optional)             |
| +   | Power              | 5V                                 |
| GND | Ground             | GND                                |


### ⚙️ Specs:
- `Steps/Pulses per revolution:` ~20 - 30 (can vary slightly depending on model)

- `Signal type:` Quadrature (two out-of-phase square waves)

- `Debouncing:` May be required for accurate pulse detection

### 🧠 How It Works:
- As the knob turns, it generates quadrature signals on CLK and DT.

- By comparing the phase difference between CLK and DT, the direction of rotation can be determined.

- An interrupt is usually attached to CLK for responsive counting.

```

```

## ⚡ VNH2SP30 Motor Driver
 - The ~VNH2SP30~ is a powerful full-bridge motor driver IC capable of driving large DC motors. It can handle high current (~up to 14A continuous~) and high voltages (~up to 16V~ typical, ~41V max~).


 [VNH2SP30 Motor Driver Image](/DC_Motor_with_Rotary_Encoder//Help.info/images/Monster_shield.jpeg)

### 🔌 Pin Connections

| Pin     | Function                   | Arduino Pin Example                                             |
| ------- | -------------------------- | --------------------------------------------------------------- |
| INA     | Direction control A        | Pin 7                                                           |
| INB     | Direction control B        | Pin 8                                                           |
| PWM     | Speed control (PWM input)  | Pin 9 (Timer1 for 31kHz PWM)                                    |
| EN/DIAG | Enable / diagnostic output | (optional, can ignore or connect to input pin for fault detect) |
| VCC     | Logic voltage (5V)         | 5V                                                              |
| GND     | Ground                     | GND                                                             |
| VIN     | Motor power supply         | External (e.g. 12V battery)                                     |
| OUTA/B  | Output to motor terminals  | Connect to DC motor                                             |

---

### ⚙️ Specs:
- ~Voltage Range:~ 5.5V – 16V (typical operation)

- ~Continuous Output Current~: 14A (with heatsink)

- ~Peak Output Current~: 30A

- ~Logic Voltage~: 5V

- ~PWM Frequency:~ Recommended 20–40 kHz for quiet operation

### 🧠 How It Works:
- INA/INB control the direction:

```
    NA = HIGH, INB = LOW → Forward

    INA = LOW, INB = HIGH → Reverse

    Both LOW or both HIGH → Brake/Coast
```
- PWM pin controls speed via analogWrite.

```

```

## 🧩 Arduino Uno Setup Summary:

| Component    | Arduino Pin | Purpose                         |
| ------------ | ----------- | ------------------------------- |
| HW-040 CLK   | 2 (INT0)    | Encoder pulse input (interrupt) |
| HW-040 DT    | 3           | Direction detection             |
| HW-040 GND   | GND         | Ground                          |
| HW-040 +V    | 5V          | Power supply                    |
| VNH2SP30 INA | 7           | Motor direction control A       |
| VNH2SP30 INB | 8           | Motor direction control B       |
| VNH2SP30 PWM | 9 (Timer1)  | Motor speed control via PWM     |
| VNH2SP30 VCC | 5V          | Logic supply                    |
| VNH2SP30 GND | GND         | Ground                          |
| VNH2SP30 VIN | 12V (EXT)   | Motor power supply              |
