// Test SLP pins: FL on 37, BL on 29, FR on 30, BR on 31.

void setup() {
  pinMode(37, OUTPUT);  // DRIVER_1_M1SLP (FL)
  pinMode(29, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
}

void loop() {
  // HIGH for 2 seconds
  digitalWrite(37, HIGH);
  digitalWrite(29, HIGH);
  digitalWrite(30, HIGH);
  digitalWrite(31, HIGH);
  delay(2000);

  // LOW for 2 seconds
  digitalWrite(37, LOW);
  digitalWrite(29, LOW);
  digitalWrite(30, LOW);
  digitalWrite(31, LOW);
  delay(2000);
}
