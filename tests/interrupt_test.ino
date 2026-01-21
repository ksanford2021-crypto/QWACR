/*
 * Simple interrupt test - just count raw interrupts
 */

#define ENCODER_FL_A 2

volatile long interrupt_count = 0;

void test_ISR() {
  interrupt_count++;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_FL_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FL_A), test_ISR, CHANGE);
  Serial.println("Interrupt test ready - wiggle FL encoder pin 2");
}

void loop() {
  static unsigned long last_print = 0;
  if (millis() - last_print > 1000) {
    Serial.print("Interrupt count: ");
    Serial.println(interrupt_count);
    last_print = millis();
  }
}
