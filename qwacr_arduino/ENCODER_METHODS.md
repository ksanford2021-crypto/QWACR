# Encoder Counting Methods - Technical Comparison

## Question: Which method is best?

**Answer: Full Quadrature (4-Edge) Decoding = Industry Standard**

This document explains why and how it works.

## Three Encoder Counting Methods

### Method 1: Single-Edge Detection (Basic)
```
Only count rising edge of Channel A

Channel A: ─────┐     ┌─────┐     ┐
                └─────┘     └─────┘
                  ↑           ↑
                Count 1     Count 2

Counts per revolution: 1600
```

**Pros:**
- Simplest implementation
- One ISR per encoder
- Fast code execution

**Cons:**
- Lowest resolution
- No direction feedback until too late
- Can't detect encoder faults
- **NOT industry standard**

---

### Method 2: Two-Edge Detection (Intermediate)
```
Count rising AND falling edges of Channel A

Channel A: ─────┐     ┐─────┐
                └─────┘     └─────
                  ↑ ↑ ↑ ↑ ↑
              Count 1,2,3,4

Counts per revolution: 3200
```

**Pros:**
- 2x better resolution than single-edge
- Still relatively simple

**Cons:**
- Ignores Channel B data
- Not using full encoder information
- **NOT industry standard**

---

### Method 3: Full Quadrature (4-Edge) - Requires Interrupts on Both Channels
```
Count all edges of BOTH channels using state machine

Channel A: ─────┐     ┬─────┬
                └─────┘     └─────
Channel B: ───────┐     ┌─────┐
                   └─────┘     └──
                ↓↓↓↓↓↓↓↓
         State machine → counts all transitions
         
Counts per revolution: 6400
```

**Pros:**
- ✅ **4x resolution** (6400 vs 1600)
- ✅ **Immediate direction detection** - know direction change in ISR
- ✅ **Error detection** - invalid transitions indicate noise/faults
- ✅ **Industry standard** - used everywhere
- ✅ **Better PID control** - finer control with more counts
- ✅ **Fault tolerance** - can detect stalling

**Cons:**
- Slightly more complex (but we use lookup table)
- More ISR calls (but faster with CHANGE mode)

---

## Industry Standard Validation

### Where 2x Quadrature is Used:
- ✅ Mobile robotics (ROS wheeled robots)
- ✅ Consumer drones
- ✅ AGVs (Automated Guided Vehicles)
- ✅ Educational robotics platforms
- ✅ Most hobby/DIY robots

### Where Full 4x Quadrature is Used:
- ✅ High-precision CNC machines
- ✅ Industrial robot arms
- ✅ Servo motor control
- ✅ Medical equipment positioning

### Where Single-Edge is Used:
- ❌ Rarely in professional applications

---

## Your Encoder Specs

### Encoder Characteristics:
- **Base Resolution:** 64 CPR at motor input shaft
- **Gear Ratio:** 100:1
- **Output Shaft Resolution:** 6400 counts per revolution (full quadrature)

### Your Measurement Confirmed It:
- Single channel (1 edge): 1600 counts
- Both channels (4 edges): 6400 counts
- **Ratio:** 4:1 (this is quadrature!)

---

## Implementation Details

### Quadrature State Machine

The encoder produces states in a specific sequence:

**Forward Rotation (Clockwise):**
```
00 → 01 → 11 → 10 → 00 → 01 → 11 → ...
```

**Backward Rotation (Counter-Clockwise):**
```
00 → 10 → 11 → 01 → 00 → 10 → 11 → ...
```

### Gray Code Lookup Table

We use a 4×4 lookup table:

```cpp
const int8_t STATE_TRANSITION[4][4] = {
  //     To: 00  01  11  10
  // From 00
  {  0,  +1,  0,  -1 },  // Forward or backward
  // From 01
  { -1,  0,  +1,  0 },
  // From 11
  {  0,  -1,  0,  +1 },
  // From 10
  { +1,  0,  -1,  0 }
};
```

**How it works:**
1. Read current A and B states → [1 bit][1 bit] = 4 possible states
2. Look up [previous_state][current_state] in table
3. Get direction: +1 (forward), -1 (backward), or 0 (invalid/noise)

---

## Interrupt Mode Comparison

### RISING (old single-edge method)
```cpp
attachInterrupt(pin, ISR, RISING);  // Only rising edge of A
```
- ❌ Only captures 1 event per cycle
- ❌ Only 1600 counts per revolution

### CHANGE (new full-quadrature method)
```cpp
attachInterrupt(pin, ISR, CHANGE);  // Both edges of A
```
- ✅ Captures all state transitions
- ✅ State machine determines if it's forward or backward
- ✅ 6400 counts per revolution
- ✅ Immediate direction detection

---

## PID Control Impact

### Resolution Improvement:
- **Single-edge:** 1600 CPR = 2π rad / 1600 = **0.0625 rad per count**
- **Full quadrature:** 6400 CPR = 2π rad / 6400 = **0.0156 rad per count** (4x finer!)

### Velocity Control Granularity:
- **Single-edge:** Velocity changes in ~0.0625 rad/s increments
- **Full quadrature:** Velocity changes in ~0.0156 rad/s increments

At 50 Hz control loop:
- **Single-edge:** ~10 discrete velocity levels
- **Full quadrature:** ~40 discrete velocity levels ✅ Much better!

---

## Error Detection Example

### Valid Transitions:
```
00 → 01  ✅  (forward, normal)
01 → 11  ✅  (forward, normal)
11 → 10  ✅  (backward, normal)
10 → 00  ✅  (backward, normal)
```

### Invalid Transitions (Noise):
```
00 → 11  ❌  (illegal transition - would skip a state)
01 → 10  ❌  (illegal - noise or electrical glitch)
```

Lookup table returns `0` for invalid transitions → ignored, doesn't affect count.

---

## Why Not Single-Edge for Your Robot?

1. **Poor PID Control:** Only 10 velocity levels vs 40
2. **Direction Detection:** Too slow - miss direction changes
3. **No Fault Detection:** Can't detect encoder problems
4. **Industry Practice:** Not used in professional robotics
5. **Future-Proof:** Full quadrature needed for advanced features (dead reckoning, etc.)

---

## Summary

| Aspect | Single-Edge | **2x Quadrature** | Full 4x Quadrature |
|--------|---|---|---|
| **CPR** | 1600 | **3200** ✅ | 6400 |
| **Interrupt Pins** | 4 | **4** ✅ | 8 (exceeds Mega) |
| **Implementation** | Simple | **State machine** ✅ | State machine |
| **Direction** | Delayed | **Immediate** ✅ | Immediate |
| **Error Detection** | None | **Yes** ✅ | Yes |
| **Robotics Use** | Rare | **Common** ✅ | CNC/High-end |
| **PID Granularity** | Adequate | **Excellent** ✅ | Exceptional |
| **Code Complexity** | Low | **Low (lookup table)** ✅ | Low |
| **Additional Libraries** | None | **None** ✅ | None (or PinChange) |

**Recommendation for your robot:** Use **2x quadrature (3200 CPR)** ✅

This is what your QWACR Arduino code uses - excellent balance of simplicity and performance!
