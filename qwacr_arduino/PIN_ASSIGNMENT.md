# Arduino Mega Pin Assignment - QWACR 4-Motor Controller

## Overview

**Arduino Mega 2560** has 54 digital pins and 6 hardware interrupt pins (INT0-INT5).

This project uses:
- **4 hardware interrupt pins** (for encoder A channels) - INT1, INT3, INT4, INT5
- **4 GPIO pins** (for encoder B channels) - Pins 24-27
- **8 motor control pins** (PWM + DIR for 4 motors) - Pins 5-12
- **1 serial port** (USB/UART to Raspberry Pi) - Pin 1 (RX), Pin 0 (TX)

**Total: 17 pins used, 37 pins available for future expansion**

## Hardware Interrupt Pins Available

| Interrupt | Pin | Usage | Available |
|-----------|-----|-------|-----------|
| INT0 | 21 | (Not used) | ✅ Available |
| INT1 | 20 | Encoder FR_A | 🔴 Used |
| INT2 | 19 | (Not used) | ✅ Available |
| INT3 | 18 | Encoder BL_A | 🔴 Used |
| INT4 | 2 | Encoder FL_A | 🔴 Used |
| INT5 | 3 | Encoder BR_A | 🔴 Used |

✅ **Result:** 2 spare interrupt pins available for future sensors/encoders

## Complete Pin Assignment

### Motor Control Pins (8 pins)

| Function | Pin | Mode | Purpose |
|----------|-----|------|---------|
| Motor FL PWM | 9 | PWM Output | Speed control (0-255) |
| Motor FL DIR | 8 | Digital Output | Direction (HIGH=forward, LOW=reverse) |
| Motor BL PWM | 10 | PWM Output | Speed control (0-255) |
| Motor BL DIR | 7 | Digital Output | Direction (HIGH=forward, LOW=reverse) |
| Motor FR PWM | 11 | PWM Output | Speed control (0-255) |
| Motor FR DIR | 6 | Digital Output | Direction (HIGH=forward, LOW=reverse) |
| Motor BR PWM | 12 | PWM Output | Speed control (0-255) |
| Motor BR DIR | 5 | Digital Output | Direction (HIGH=forward, LOW=reverse) |

### Encoder Pins (8 pins)

#### Interrupt Pins (Channel A - Both Edges Trigger ISR)

| Function | Pin | Interrupt | Mode | Purpose |
|----------|-----|-----------|----- -|---------|
| Encoder FL_A | 2 | INT4 | Interrupt Input | Front-Left encoder channel A |
| Encoder BL_A | 18 | INT3 | Interrupt Input | Back-Left encoder channel A |
| Encoder FR_A | 20 | INT1 | Interrupt Input | Front-Right encoder channel A |
| Encoder BR_A | 3 | INT5 | Interrupt Input | Back-Right encoder channel A |

#### GPIO Pins (Channel B - Read in ISR for Direction)

| Function | Pin | Mode | Purpose |
|----------|-----|------|---------|
| Encoder FL_B | 24 | Digital Input | Front-Left encoder channel B |
| Encoder BL_B | 25 | Digital Input | Back-Left encoder channel B |
| Encoder FR_B | 26 | Digital Input | Front-Right encoder channel B |
| Encoder BR_B | 27 | Digital Input | Back-Right encoder channel B |

### Serial Communication (2 pins)

| Function | Pin | Mode | Purpose |
|----------|-----|------|---------|
| Serial RX | 0 | Serial Input | Receive from Raspberry Pi |
| Serial TX | 1 | Serial Output | Send to Raspberry Pi |

### Unassigned Pins (37 available for expansion)

- Analog: A0-A15 (16 pins)
- Digital: 4, 13-17, 22-23, 28-53 (many pins)

## Why This Configuration?

### 1. **Encoder Channel A on Interrupts, Channel B on GPIO**

**Encoder Quadrature Principle (2x Mode):**
```
Channel A: ————┐    ┌————┐    ┌————
              └────┘    └────┘
Channel B: ──┐    ┐————┐    ┐────
            └────┘    └────┘
              ↑↓        ↑↓
        CHANGE mode = both edges trigger ISR
```

- **Channel A:** Both edges trigger ISR (CHANGE mode) → **MUST be on interrupt pin**
- **Channel B:** Read inside ISR to determine direction → **GPIO is fine**
- **Result:** 2x resolution (3200 CPR) - captures rising + falling of A

### 2. **Four Separate Interrupt Pins (No Sharing)**

- Each encoder has dedicated interrupt
- No need to check which pin changed
- Simpler, more robust ISR code
- Faster response time

### 3. **No Pin Change Interrupt Library Needed**

- Arduino Mega has 6 hardware interrupts for only 4 motors
- Dedicated interrupt for each motor eliminates complexity
- Easier maintenance and debugging
- Pin Change Interrupts slower and add complexity

### 4. **Motor Control Pins (5-12)**

- PWM pins (5-12, 44-46) are used for speed control
- Direction pins can be any GPIO
- Chosen pins 5-12 to keep motor control together

## Verification Checklist

Before uploading Arduino code, verify physical connections:

### Motor Driver Connections
- [ ] Driver 1 Power: 24V to both motors (FL, BL)
- [ ] Driver 2 Power: 24V to both motors (FR, BR)
- [ ] Driver 1 PWM_A → Pin 9, DIR_A → Pin 8 (Motor FL)
- [ ] Driver 1 PWM_B → Pin 10, DIR_B → Pin 7 (Motor BL)
- [ ] Driver 2 PWM_A → Pin 11, DIR_A → Pin 6 (Motor FR)
- [ ] Driver 2 PWM_B → Pin 12, DIR_B → Pin 5 (Motor BR)

### Encoder Connections
- [ ] Encoder FL: A→Pin2, B→Pin24
- [ ] Encoder BL: A→Pin18, B→Pin25
- [ ] Encoder FR: A→Pin20, B→Pin26
- [ ] Encoder BR: A→Pin3, B→Pin27
- [ ] All encoders: Ground to Arduino GND

### Serial Connection
- [ ] Raspberry Pi TX → Arduino Pin 0 (RX)
- [ ] Raspberry Pi RX → Arduino Pin 1 (TX)
- [ ] Shared ground between systems

## Future Expansion

Available hardware interrupts for additional sensors:
- **INT0 (Pin 21):** Could add another encoder or sensor
- **INT2 (Pin 19):** Could add another encoder or sensor

Suggested future enhancements:
- Limit switches on spare interrupt pins
- IMU sensor on I2C
- Ultrasonic distance sensors on GPIO
- Emergency stop button on interrupt pin

## Testing Without Physical Hardware

To verify pin assignments without soldering:
```cpp
// Quick test - toggle all pins
void setup() {
  pinMode(9, OUTPUT);    // Motor FL PWM
  pinMode(8, OUTPUT);    // Motor FL DIR
  // ... other pins ...
}

void loop() {
  digitalWrite(9, HIGH);
  digitalWrite(8, HIGH);
  delay(1000);
  
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
  delay(1000);
}
```

Then verify with multimeter:
- PWM pins: Should show varying voltage 0-5V
- DIR pins: Should toggle 0V ↔ 5V
- Encoder pins: Should see pulses on oscilloscope

## Code Reference

Pin definitions in `qwacr_main.ino`:
```cpp
// Motor Front-Left
#define MOTOR_FL_PWM 9
#define MOTOR_FL_DIR 8
#define ENCODER_FL_A 2
#define ENCODER_FL_B 24

// Motor Back-Left
#define MOTOR_BL_PWM 10
#define MOTOR_BL_DIR 7
#define ENCODER_BL_A 18
#define ENCODER_BL_B 25

// Motor Front-Right
#define MOTOR_FR_PWM 11
#define MOTOR_FR_DIR 6
#define ENCODER_FR_A 20
#define ENCODER_FR_B 26

// Motor Back-Right
#define MOTOR_BR_PWM 12
#define MOTOR_BR_DIR 5
#define ENCODER_BR_A 3
#define ENCODER_BR_B 27
```

ISR Attachment in setup():
```cpp
attachInterrupt(digitalPinToInterrupt(ENCODER_FL_A), encoder_fl_ISR, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER_BL_A), encoder_bl_ISR, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER_FR_A), encoder_fr_ISR, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER_BR_A), encoder_br_ISR, RISING);
```

**✅ All pins assigned with no conflicts**
**✅ Maximum reliability with hardware interrupts**
**✅ No complex libraries or bit manipulation needed**
