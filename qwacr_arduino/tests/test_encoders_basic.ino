// Encoder test for FL, BL, FR, BR
// A channels on 2,18,20,3 (interrupts)
// B channels on 41,45,49,53 (GPIO)

volatile long c_fl = 0, c_bl = 0, c_fr = 0, c_br = 0;

void isr_fl() { c_fl++; }
void isr_bl() { c_bl++; }
void isr_fr() { c_fr++; }
void isr_br() { c_br++; }

void setup() {
  Serial.begin(115200);

  pinMode(2,  INPUT);  // FL A
  pinMode(18, INPUT);  // BL A
  pinMode(20, INPUT);  // FR A
  pinMode(3,  INPUT);  // BR A

  pinMode(41, INPUT);  // FL B
  pinMode(45, INPUT);  // BL B
  pinMode(49, INPUT);  // FR B
  pinMode(53, INPUT);  // BR B

  attachInterrupt(digitalPinToInterrupt(2),  isr_fl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), isr_bl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), isr_fr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3),  isr_br, CHANGE);
}

void loop() {
  // Snapshot volatile counts
  noInterrupts();
  long fl = c_fl, bl = c_bl, fr = c_fr, br = c_br;
  interrupts();

  int fl_b = digitalRead(41);
  int bl_b = digitalRead(45);
  int fr_b = digitalRead(49);
  int br_b = digitalRead(53);

  Serial.print("FL A: "); Serial.print(fl);
  Serial.print("  B: "); Serial.print(fl_b);
  Serial.print("   | BL A: "); Serial.print(bl);
  Serial.print("  B: "); Serial.print(bl_b);
  Serial.print("   | FR A: "); Serial.print(fr);
  Serial.print("  B: "); Serial.print(fr_b);
  Serial.print("   | BR A: "); Serial.print(br);
  Serial.print("  B: "); Serial.println(br_b);

  delay(200);
}
