# QWACR Arduino Motor Control

Complete Arduino firmware for 4-wheel differential drive robot with independent PID-based motor velocity control and **request/response odometry protocol**.

## Key Features

✅ **4-Motor Differential Drive** - Smooth PID velocity tracking  
✅ **Full Quadrature Encoding** - 3200 CPR per motor with 4x decoding  
✅ **COBS Serial Protocol** - Reliable packet framing (115200 baud)  
✅ **Request/Response Odometry** - On-demand encoder queries, non-blocking  
✅ **Safety Features** - 2-second timeout, sleep control, velocity limits  

## Hardware Configuration

### 4-Wheel Motor Layout
```
   Front-Left (FL)      Front-Right (FR)
        |                      |
   Back-Left (BL)       Back-Right (BR)
```

**Control Strategy:**
- Left motors (FL + BL) share the same velocity setpoint
- Right motors (FR + BR) share the same velocity setpoint  
- Each motor has **independent PID controller** for accurate tracking
- This enables differential steering (left faster = turn left)

### Motor Specifications
- **Type:** Brushed DC motors with quadrature encoders
- **Nominal Speed:** 90 RPM (9.42 rad/s)
- **Gear Ratio:** 100:1
- **Encoder:** 64 CPR (input shaft)
- **Effective Resolution:** 3200 counts per output shaft revolution (full quadrature, 4x decoding)

## Hardware Setup

### Pololu Dual G2 Motor Driver Configuration

Two Pololu Dual G2 High-Power Motor Drivers (24V, 14A) - one for left motors, one for right motors.

**Driver 1 (Left Motors):**
- Motor FL (Front-Left):
  - PWM Pin: Arduino Pin 9
  - DIR Pin: Arduino Pin 8
- Motor BL (Back-Left):
  - PWM Pin: Arduino Pin 10
  - DIR Pin: Arduino Pin 7

**Driver 2 (Right Motors):**
- Motor FR (Front-Right):
  - PWM Pin: Arduino Pin 11
  - DIR Pin: Arduino Pin 6
- Motor BR (Back-Right):
  - PWM Pin: Arduino Pin 12
  - DIR Pin: Arduino Pin 5

### Encoder Setup

**Motor FL Encoder (Front-Left):**
- Channel A: Arduino Pin 2 (Interrupt INT4) ← **Hardware Interrupt (CHANGE)**
- Channel B: Arduino Pin 24 (GPIO) - read in ISR

**Motor BL Encoder (Back-Left):**
- Channel A: Arduino Pin 18 (Interrupt INT3) ← **Hardware Interrupt (CHANGE)**
- Channel B: Arduino Pin 25 (GPIO) - read in ISR

**Motor FR Encoder (Front-Right):**
- Channel A: Arduino Pin 20 (Interrupt INT1) ← **Hardware Interrupt (CHANGE)**
- Channel B: Arduino Pin 26 (GPIO) - read in ISR

**Motor BR Encoder (Back-Right):**
- Channel A: Arduino Pin 3 (Interrupt INT5) ← **Hardware Interrupt (CHANGE)**
- Channel B: Arduino Pin 27 (GPIO) - read in ISR

**Encoder Specifications:**
- Base Resolution: 64 CPR at motor input shaft
- Gear Ratio: 100:1
- Output Shaft Resolution: **3200 counts per revolution** (2x quadrature)
- Detection Method: **2x quadrature decoding** using state machine
  - Channel A: Hardware interrupt (CHANGE mode captures rising + falling edges)
  - Channel B: GPIO input (read in ISR for direction detection)
  - 2x resolution improvement over single-edge detection
  - Immediate direction detection from A/B state relationship
  - Error/noise detection via invalid state transitions
  - **Note:** True 4x quadrature (6400 CPR) would require interrupts on both A and B channels (8 total), exceeding Mega's 6 hardware interrupt pins

### Serial Connection
- Arduino Mega → Raspberry Pi via USB/UART
- Baud rate: 115200
- Used for sending motor commands and receiving encoder feedback

## Code Structure

### qwacr_main.ino
Main sketch that:
- Initializes 4 motor controllers with independent PIDs
- Sets up encoder interrupts for **full quadrature (4-edge) decoding**
- Reads velocity commands from PacketSerial (sent by ROS 2)
- Processes encoder interrupts for real-time feedback with quadrature decoder
- Updates 4 independent PID controllers
- Sends all 4 encoder/velocity readings back to ROS 2
- Safety timeout (stops all motors if no command for 500ms)
- Velocity limiting to ±90 RPM (±9.42 rad/s)

### quadrature_decoder.h
Robust quadrature encoder decoder using state machine:
- **2x quadrature decoding:** Captures both edges of channel A
- **Gray code state machine:** Tracks transitions for 3200 CPR effective resolution
- **Error detection:** Identifies invalid state transitions (noise/faults)
- **2x resolution:** Compared to single-edge detection (3200 CPR vs 1600 CPR)
- **Common in robotics:** Mobile robots, AGVs, consumer platforms
- **Fast ISR:** Uses lookup table for O(1) decoding
- **Efficient:** Only needs interrupts on channel A (B read as GPIO)

### motor_control.h
Motor class with Arduino PID library integration:
- Uses industry-standard **PID_v1** library by Brett Beauregard
- **Proportional (Kp):** Error scaling for immediate response
- **Integral (Ki):** Steady-state error correction
- **Derivative (Kd):** Damping and smoothing
- Automatic anti-windup protection (built into library)
- Sample time management (20ms = 50Hz control loop)
- Converts PID output to PWM (0-255) with direction control
- Real-time velocity calculation from encoder deltas

### serial_protocol.h
Protocol using **PacketSerial** library for reliable communication:
- **COBS Encoding:** Consistent Overhead Byte Stuffing for robust framing
- No manual start/end markers needed - handled automatically
- Graceful handling of corrupted packets
- **Command format:** `[TYPE][vel_left_float][vel_right_float]`
- **Feedback format:** `[TYPE][4_encoders_int32][4_velocities_float]`
- Type-prefixed packets for future extensibility
- Industry-standard approach used in production systems

## Installation

### Arduino IDE Setup
1. Open Arduino IDE
2. Select Board: Arduino Mega 2560
3. Select Port: /dev/ttyACM0 or /dev/ttyUSB0
4. Sketch → Include Library → Add .ZIP Library... (if needed)

### Upload to Arduino
```bash
# Option 1: Arduino IDE - File → Upload
# Option 2: Command line (with arduino-cli)
arduino-cli upload --fqbn arduino:avr:mega -p /dev/ttyACM0 ./qwacr_main.ino
```

## PID Tuning

Current default PID coefficients (starting point for 90 RPM motors):
- **Kp = 15.0** (proportional gain - scales error to output)
- **Ki = 1.0** (integral gain - eliminates steady-state error)
- **Kd = 0.1** (derivative gain - reduces overshoot and oscillation)

These values are tuned for the Arduino PID library's expected scaling with motors running at nominal 90 RPM (9.42 rad/s).

### How to Tune PID

1. **Start with Kp only** (Ki=0, Kd=0):
   - Increase Kp until system starts oscillating
   - Back off to ~70% of that value
   
2. **Add Ki** to eliminate steady-state error:
   - Start with Ki = Kp / 100
   - Increase if offset remains, decrease if oscillations occur
   
3. **Add Kd** to reduce overshoot:
   - Start with Kd = Kp * 0.1
   - Increase for smoother response

4. **Test with step commands:**
   ```bash
   # Use test_serial.py to send commands (needs updating for PacketSerial)
   python3 test_serial.py
   ```

5. **Per-Motor Tuning:**
   - Motors may have slight mechanical differences
   - Can tune each motor individually by modifying constructor in qwacr_main.ino
   - Example: `Motor motor_fl(MOTOR_FL_PWM, MOTOR_FL_DIR, COUNTS_PER_REV, 16.0, 1.2, 0.12);`

### Tuning Strategy
- Measure actual motor velocity vs setpoint
- Look for: no overshoot, quick settling, minimal oscillation
- Document working values in motor_control.h

## Encoder Resolution & Quadrature Decoding

### Why 2x Quadrature (3200 CPR)?

Your encoder produces **2 state transitions when only channel A has interrupts:**
```
       01
      /  \
     /    \
   00 ← → 11
     \    /
      \  /
       10

2 edges of channel A (with B for direction) = 3200 CPR
Note: Full 4x quadrature (6400 CPR) would require interrupts on both A and B
```

### Comparison: Single-Edge vs Full Quadrature

| Aspect | Single-Edge | 2x Quadrature (Current) | 4x Quadrature |
|--------|---|---|---|
| **CPR** | 1600 | **3200 (2x)** ✅ | 6400 (4x) |
| **Interrupt Pins** | 4 | **4** ✅ | 8 (exceeds Mega!) |
| **PID Resolution** | ±0.0625 rad/s | **±0.0312 rad/s** ✅ | ±0.0156 rad/s |
| **Direction Detection** | Delayed | **Immediate** ✅ | Immediate |
| **Noise Detection** | No | **Yes** ✅ | Yes |
| **Implementation** | Simple | **State machine** ✅ | State machine |
| **Industry Use** | ❌ Rare | **✅ Common** | ✅ CNC/High-end |

### How It Works

The quadrature decoder tracks the **Gray code sequence** of A and B channels:

**Key Point:** Only channel A has an interrupt. Channel B is just a GPIO input read when A changes.

```
When Channel A interrupt fires (rising or falling):
1. Read current state of A (just changed)
2. Read current state of B (GPIO input)
3. Compare [prev_A, prev_B] → [curr_A, curr_B]
4. Lookup table tells us: forward (+1), backward (-1), or error (0)
```

**Example - Forward:**
```
Time  A  B  State  A Changed?  B State?  Direction
─────────────────────────────────────────────────
 0    0  0   00       -          -         -
 1    0  1   01       No         -         (no interrupt)
 2    1  1   11       YES!       B=1       Forward (+1)
 3    1  0   10       No         -         (no interrupt)
 4    0  0   00       YES!       B=0       Forward (+1)
```

**Example - Backward:**
```
Time  A  B  State  A Changed?  B State?  Direction
─────────────────────────────────────────────────
 0    0  0   00       -          -         -
 1    1  0   10       YES!       B=0       Backward (-1)
 2    1  1   11       No         -         (no interrupt)
 3    0  1   01       YES!       B=1       Backward (-1)
```

**Why B doesn't need an interrupt:**
- B acts as a "direction indicator"
- When A changes, we check: "Is B high or low?"
- The A/B relationship determines forward vs backward
- B doesn't "lead" - it just indicates which way we're going

**Limitation:** We only capture 2 edges per cycle (A rising + A falling), not all 4 edges. This gives 3200 CPR instead of 6400 CPR.

**Invalid transitions** (e.g., 00 → 11) indicate encoder errors or electrical noise and are ignored.

### Benefits for Your Robot

1. **Better PID Control:** 4x resolution = finer adjustments
2. **Immediate Direction:** Know direction change instantly
3. **Robust:** Detects encoder faults automatically
4. **Standard:** Industry-proven approach
5. **Future-proof:** Can detect motor stalling or slipping

### Command Packet (ROS 2 → Arduino)
```
Byte 0:      0x01 (CMD_SET_VELOCITY type)
Bytes 1-4:   vel_left (float, rad/s, little-endian)
Bytes 5-8:   vel_right (float, rad/s, little-endian)
Total:       9 bytes (before COBS encoding)
```

**Velocity Convention:**
- Positive: Motors rotate forward
- Negative: Motors rotate backward  
- Range: ±9.42 rad/s (±90 RPM) - enforced by firmware
- Left setpoint controls FL and BL motors
- Right setpoint controls FR and BR motors

### Feedback Packet (Arduino → ROS 2)
```
Byte 0:       0x10 (FEEDBACK_DATA type)
Bytes 1-4:    encoder_fl (int32, absolute count)
Bytes 5-8:    encoder_bl (int32, absolute count)
Bytes 9-12:   encoder_fr (int32, absolute count)
Bytes 13-16:  encoder_br (int32, absolute count)
Bytes 17-20:  velocity_fl (float, rad/s)
Bytes 21-24:  velocity_bl (float, rad/s)
Bytes 25-28:  velocity_fr (float, rad/s)
Bytes 29-32:  velocity_br (float, rad/s)
Total:        33 bytes (before COBS encoding)
```

**Feedback Rate:** ~50 Hz (20ms)

**COBS Encoding Benefits:**
- Packets delimited by 0x00 byte (never appears in encoded data)
- Automatic synchronization after corruption
- Minimal overhead (~0.4% for typical packets)
- No escape sequences needed

## Testing

### Basic Serial Test
```bash
python3 test_serial.py
```

This will:
1. Send STOP command (0 rad/s)
2. Send FORWARD command (2 rad/s both motors)
3. Read encoder and velocity feedback
4. Send STOP command

**Expected Behavior:**
- Encoders increase during forward motion
- Velocities approach 2 rad/s setpoint
- Motors stop when commanded

### ROS 2 Integration Test
```bash
# In one terminal (ROS 2)
cd ~/qwacr_ws
source install/setup.bash
ros2 launch qwacr_build display.launch.py use_gazebo:=false use_teleop:=true

# In another terminal (Python script)
python3 ~/qwacr_arduino/test_serial.py --ros2
```

This bridges the serial protocol directly to `/diff_cont/cmd_vel` commands.

## Troubleshooting

### Motors Don't Move
- Check 24V power to Pololu drivers
- Verify PWM pins with multimeter (voltage should vary 0-5V)
- Test motor direction pins (should be HIGH or LOW)
- Check encoder connections (monitor serial feedback for count changes)
- Verify all 4 motors initialized correctly

### Inconsistent Velocities Between Motors
- Check encoder wiring (especially channel B for direction)
- Verify mechanical condition (binding, friction differences)
- Tune PID gains individually per motor
- Confirm 1600 CPR is correct for your encoder/gearbox combination

### Serial Communication Failures
- Verify both PID and PacketSerial libraries installed
- Check baud rate matches (115200)
- PacketSerial will automatically recover from corrupted packets
- Monitor with: `arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200`

### Oscillating Motor Velocity
- Reduce Kp (proportional gain) - try 10.0 or lower
- Increase Kd (derivative gain) - try 0.2 or higher
- Verify encoder readings are stable (check for electrical noise)
- Ensure PWM frequency is appropriate (default Arduino ~490 Hz on most pins)

### Sluggish Motor Response
- Increase Kp (proportional gain) - try 20.0 or higher  
- Increase Ki (integral gain) if steady-state error persists
- Check motor power supply voltage (should be 24V under load)
- Verify mechanical load isn't excessive

### Encoder Count Errors
- **If counts seem 2x expected:** Using full quadrature somehow - verify only A has interrupts
- **If counts seem half expected:** May be in single-edge mode - check CHANGE interrupt mode
- **If direction wrong on one motor:** Swap encoder A/B wires for that motor
- **If counts erratic:** Check for electrical noise, add filtering capacitors

## Integration with ROS 2

The Arduino firmware works with the `mega_diff_drive_control` hardware interface in your ROS 2 workspace. The interface:

1. **Receives wheel velocities** from diff_drive_controller (4 wheels: FL, BL, FR, BR)
2. **Averages left wheels** (FL + BL) and **right wheels** (FR + BR) for 2 setpoints
3. **Sends commands** to Arduino via PacketSerial protocol
4. **Receives encoder feedback** from all 4 motors
5. **Publishes to /joint_states** for ROS 2 odometry and visualization

### ROS 2 Configuration Required

In your hardware interface code (`mega_diff_drive_control`):
```cpp
// Must match Arduino configuration
const float WHEEL_RADIUS = 0.165;       // meters
const float WHEEL_SEPARATION = 0.38;    // meters  
const int COUNTS_PER_REV = 3200;        // 2x quadrature encoding
const float MAX_VELOCITY_RAD_S = 9.42;  // 90 RPM nominal
```

**Important:** Update your ROS 2 hardware interface to:
- Use PacketSerial protocol (COBS encoded)
- Handle 4 encoder feedbacks instead of 2
- Average FL+BL for left setpoint, FR+BR for right setpoint
- Publish all 4 wheel positions to /joint_states

## Safety Features

1. **Command Timeout:** All motors stop if no command received for 500ms
2. **Velocity Limits:** Firmware clamps commands to ±9.42 rad/s (90 RPM)
3. **Integral Windup Prevention:** Built into Arduino PID library
4. **PWM Saturation:** Output limited to 0-255 range
5. **Packet Validation:** PacketSerial COBS encoding ensures data integrity

## Libraries Used

- **PID_v1** by Brett Beauregard - Industry-standard PID controller
- **PacketSerial** by Chris Baker - Reliable serial communication with COBS encoding

## References

- **Pololu Dual G2 Datasheet:** https://www.pololu.com/product/3751
- **Arduino Mega Pinout:** https://docs.arduino.cc/hardware/mega-2560
- **PID Control Theory:** https://en.wikipedia.org/wiki/PID_controller
- **Arduino PID Library:** https://github.com/br3ttb/Arduino-PID-Library
- **PacketSerial Library:** https://github.com/bakercp/PacketSerial
- **COBS Encoding:** https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing

## Important Notes

- **Encoder Resolution:** Using 3200 CPR (2x quadrature with interrupts only on channel A)
  - True 4x quadrature (6400 CPR) would need interrupts on both A and B channels (8 total pins)
  - 3200 CPR provides excellent resolution for PID control with simpler hardware
- **Motor Count:** 4 independent motors with 4 independent PIDs, but paired setpoints (left/right)
- **Serial Protocol:** PacketSerial provides automatic error recovery - much more reliable than manual markers
- **PID Tuning:** Start conservative, tune gradually. Each motor can have individual gains if needed.
