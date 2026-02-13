#define LORA_ROLE_ROBOT
#include "../common/lora_bridge_common.h"

void setup() {
  Serial.begin(115200);
  qwacr_lora::setup_radio();
}

void loop() {
  qwacr_lora::poll_radio();
  qwacr_lora::handle_uart();
  qwacr_lora::publish_status();
  delay(5);
}