// Minimal test sketch - no LoRa, just verify board is alive
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== BOARD ALIVE ===");
}

void loop() {
  Serial.println("Hello from ESP32");
  delay(1000);
}
