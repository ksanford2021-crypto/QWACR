/*
 * HARDWARE TEST: Directly toggle FL motor driver to diagnose wiring/driver fault
 * This bypasses the normal motor control code
 */

#define DRIVER_1_M1SLP 28   // M1SLP for FL motor
#define MOTOR_FL_PWM 9
#define MOTOR_FL_DIR 8

void setup() {
  Serial.begin(115200);
  delay(100);
  
  pinMode(DRIVER_1_M1SLP, OUTPUT);
  pinMode(MOTOR_FL_PWM, OUTPUT);
  pinMode(MOTOR_FL_DIR, OUTPUT);
  
  Serial.println("=== FL HARDWARE TEST ===");
  Serial.println("Testing direct driver control...\n");
}

void loop() {
  // Test 1: Wake driver, set direction, apply PWM
  Serial.println("[Test 1] Waking driver and applying PWM...");
  digitalWrite(DRIVER_1_M1SLP, HIGH);  // Wake driver
  delay(100);
  
  digitalWrite(MOTOR_FL_DIR, LOW);   // Forward direction
  analogWrite(MOTOR_FL_PWM, 200);    // 50% PWM (~200/255)
  
  Serial.println("  Sleep pin: HIGH (driver active)");
  Serial.println("  Dir pin: LOW (forward)");
  Serial.println("  PWM: 200 (78% duty)");
  Serial.println("  Listen for motor spin...");
  delay(3000);
  
  // Test 2: Reverse direction
  Serial.println("\n[Test 2] Reversing direction...");
  digitalWrite(MOTOR_FL_DIR, HIGH);  // Reverse direction
  Serial.println("  Dir pin: HIGH (reverse)");
  delay(3000);
  
  // Test 3: Stop
  Serial.println("\n[Test 3] Stopping motor...");
  analogWrite(MOTOR_FL_PWM, 0);
  Serial.println("  PWM: 0 (stopped)");
  delay(1000);
  
  // Test 4: Sleep driver
  Serial.println("\n[Test 4] Putting driver to sleep...");
  digitalWrite(DRIVER_1_M1SLP, LOW);   // Sleep driver
  Serial.println("  Sleep pin: LOW (driver sleeping)");
  delay(1000);
  
  Serial.println("\n=== TEST CYCLE COMPLETE ===\n");
  delay(2000);
}
