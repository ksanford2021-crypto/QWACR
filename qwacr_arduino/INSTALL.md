# Arduino Installation Guide

## Required Libraries

This project requires two Arduino libraries:

### 1. Arduino PID Library

The **PID_v1** library by Brett Beauregard - a well-tested PID controller implementation.

### 2. PacketSerial Library

The **PacketSerial** library by Chris Baker - provides reliable serial communication with COBS encoding for robust packet framing.

## Installation Methods

#### Method 1: Arduino IDE Library Manager (Recommended)

1. Open Arduino IDE
2. Go to **Sketch** → **Include Library** → **Manage Libraries...**
3. In the search box, type: `PID`
4. Find **"PID" by Brett Beauregard**
5. Click **Install**
6. Search for: `PacketSerial`
7. Find **"PacketSerial" by Chris Baker (bakercp)**
8. Click **Install**
9. Wait for installation to complete

#### Method 2: Manual Installation

```bash
# Download from GitHub
cd ~/Arduino/libraries/
git clone https://github.com/br3ttb/Arduino-PID-Library.git PID_v1
git clone https://github.com/bakercp/PacketSerial.git PacketSerial

# Restart Arduino IDE
```

#### Method 3: arduino-cli (Command Line)

```bash
# Install arduino-cli if not already installed
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install both libraries
arduino-cli lib install "PID"
arduino-cli lib install "PacketSerial"
```

## Verify Installation

After installing, verify by checking:
- Arduino IDE → **Sketch** → **Include Library** → you should see "PID" and "PacketSerial" in the list
- Or check directories exist:
  - `~/Arduino/libraries/PID_v1/`
  - `~/Arduino/libraries/PacketSerial/`

## Upload Code

### Using Arduino IDE

1. Open `qwacr_main.ino` in Arduino IDE
2. Select **Tools** → **Board** → **Arduino AVR Boards** → **Arduino Mega or Mega 2560**
3. Select **Tools** → **Port** → **/dev/ttyACM0** (or /dev/ttyUSB0)
4. Click **Upload** button (→)
5. Wait for "Done uploading" message

### Using arduino-cli

```bash
cd ~/qwacr_arduino

# Compile
arduino-cli compile --fqbn arduino:avr:mega qwacr_main.ino

# Upload
arduino-cli upload --fqbn arduino:avr:mega -p /dev/ttyACM0 qwacr_main.ino

# Monitor serial output (optional, for debugging)
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## Pin Verification Checklist

Before uploading, double-check these pin assignments match your hardware:

### Pololu Dual G2 Driver 1 (Left Motors)
- [ ] Motor FL PWM → Arduino Pin 9
- [ ] Motor FL DIR → Arduino Pin 8
- [ ] Motor BL PWM → Arduino Pin 10
- [ ] Motor BL DIR → Arduino Pin 7

### Pololu Dual G2 Driver 2 (Right Motors)
- [ ] Motor FR PWM → Arduino Pin 11
- [ ] Motor FR DIR → Arduino Pin 6
- [ ] Motor BR PWM → Arduino Pin 12
- [ ] Motor BR DIR → Arduino Pin 5

### Encoders (Interrupt Pins - Hardware Interrupts Only)
- [ ] Encoder FL_A → Pin 2 (INT4)
- [ ] Encoder FL_B → Pin 24 (GPIO)
- [ ] Encoder BL_A → Pin 18 (INT3)
- [ ] Encoder BL_B → Pin 25 (GPIO)
- [ ] Encoder FR_A → Pin 20 (INT1)
- [ ] Encoder FR_B → Pin 26 (GPIO)
- [ ] Encoder BR_A → Pin 3 (INT5)
- [ ] Encoder BR_B → Pin 27 (GPIO)

**✅ All pins unique - no conflicts!**
**✅ Uses 4 of 6 available hardware interrupts**
**✅ No Pin Change Interrupt library needed**

### Recommended Pin Configuration

The key insight: **Channel A needs interrupt, Channel B only needs to be read**

Only 4 hardware interrupt pins needed for Channel A (all on unique pins):
- Channel A triggers ISR on rising edge
- Channel B read inside ISR to determine direction
- No conflicts, maximum reliability

Old configuration (❌ AVOIDED):
```cpp
// DON'T use this - pin 3 conflict!
#define ENCODER_BR_A 21     // Same as encoder FR_B
```

New configuration (✅ CORRECT):
```cpp
// Use this - all unique pins, no conflicts
#define ENCODER_FL_A 2      // INT4
#define ENCODER_FL_B 24     // GPIO
#define ENCODER_BL_A 18     // INT3
#define ENCODER_BL_B 25     // GPIO
#define ENCODER_FR_A 20     // INT1
#define ENCODER_FR_B 26     // GPIO
#define ENCODER_BR_A 3      // INT5
#define ENCODER_BR_B 27     // GPIO
```

## Troubleshooting

### Compilation Error: "PID.h: No such file or directory"
- PID library not installed
- Solution: Follow installation steps above

### Upload Error: "Permission denied on /dev/ttyACM0"
```bash
# Add your user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Upload Error: "Programmer not responding"
- Wrong board selected (must be Mega 2560)
- Wrong port selected
- Try pressing reset button on Arduino before upload

### Motors Not Moving After Upload
- Check 24V power to motor drivers
- Verify pin connections with multimeter
- Check serial monitor for feedback packets (115200 baud)

## Next Steps

After successful upload:
1. Test with `python3 test_serial.py` (basic test)
2. Integrate with ROS 2 hardware interface
3. Tune PID parameters based on motor response
