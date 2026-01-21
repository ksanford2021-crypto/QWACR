/*
 * Quadrature Encoder Decoder
 * Full 4-edge quadrature decoding with error detection
 * Industry-standard approach used in robotics, CNC, industrial equipment
 */

#ifndef QUADRATURE_DECODER_H
#define QUADRATURE_DECODER_H

#include <Arduino.h>

class QuadratureDecoder {
private:
  // Quadrature state values (Gray code)
  // 00 (0), 01 (1), 11 (3), 10 (2) - allows detection of invalid transitions
  enum State {
    STATE_00 = 0,  // A=0, B=0
    STATE_01 = 1,  // A=0, B=1
    STATE_11 = 3,  // A=1, B=1
    STATE_10 = 2   // A=1, B=0
  };
  
  // State transition table for quadrature decoding
  // Maps: [previous_state][current_state] → count_direction
  // +1 = forward (clockwise), -1 = backward (counter-clockwise), 0 = invalid/noise
  static const int8_t STATE_TRANSITION[4][4];
  
  State current_state;
  State previous_state;
  volatile long count;
  
public:
  QuadratureDecoder() 
    : current_state(STATE_00), previous_state(STATE_00), count(0) {}
  
  // Initialize with current A and B pin states
  void begin(uint8_t pin_a, uint8_t pin_b) {
    uint8_t a = digitalRead(pin_a);
    uint8_t b = digitalRead(pin_b);
    
    current_state = static_cast<State>((a << 1) | b);
    previous_state = current_state;
    count = 0;
  }
  
  // Update with new A and B states (call from ISR)
  // Returns count change: +1, -1, or 0
  int8_t update(uint8_t pin_a, uint8_t pin_b) {
    previous_state = current_state;
    current_state = static_cast<State>((digitalRead(pin_a) << 1) | digitalRead(pin_b));
    
    int8_t direction = STATE_TRANSITION[previous_state][current_state];
    
    if (direction != 0) {
      count += direction;
    }
    
    return direction;
  }
  
  // Update using pre-read pin states (faster for ISR)
  int8_t updateFast(uint8_t a_state, uint8_t b_state) {
    previous_state = current_state;
    current_state = static_cast<State>((a_state << 1) | b_state);
    
    int8_t direction = STATE_TRANSITION[previous_state][current_state];
    
    if (direction != 0) {
      count += direction;
    }
    
    return direction;
  }
  
  // Get total count
  long getCount() const {
    return count;
  }
  
  // Reset count
  void resetCount() {
    count = 0;
  }
  
  // Get last detected direction
  // +1 = forward, -1 = backward, 0 = no change/invalid
  int8_t getLastDirection() const {
    return STATE_TRANSITION[previous_state][current_state];
  }
  
  // Check if last transition was valid
  bool isLastTransitionValid() const {
    return STATE_TRANSITION[previous_state][current_state] != 0;
  }
  
  // Detect encoder errors (invalid state transition)
  bool hasError() const {
    return !isLastTransitionValid();
  }
};

// State transition lookup table
// Quadrature decoding: count transitions in the Gray code sequence
// Valid sequence: 00 → 01 → 11 → 10 → 00 (forward)
//                 00 → 10 → 11 → 01 → 00 (backward)
const int8_t QuadratureDecoder::STATE_TRANSITION[4][4] = {
  //     To: 00  01  11  10
  // From 00
  {  0,  +1,  0,  -1 },  // 00→01=+1(forward), 00→10=-1(backward)
  // From 01
  { -1,  0,  +1,  0 },   // 01→00=-1(backward), 01→11=+1(forward)
  // From 11
  {  0,  -1,  0,  +1 },  // 11→10=-1(backward), 11→01=+1(forward)
  // From 10
  { +1,  0,  -1,  0 }    // 10→00=+1(forward), 10→11=-1(backward)
};

#endif
