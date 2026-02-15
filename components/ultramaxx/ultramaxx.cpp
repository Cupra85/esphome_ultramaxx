#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::send_wakeup_() {
  // 2400 8N1 Wakeup
  uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE);

  uint8_t wake = 0x55;

  for (int i = 0; i < 360; i++) {  // ~3 Sekunden
    this->write_array(&wake, 1);
    delay(8);
  }

  delay(234);

  // zurück auf 8E1
  uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);
}

void UltraMaXXComponent::send_init_frame_() {
  // CI=A6 Init Frame aus Thread #8009682
  uint8_t snd_ud[] = {
      0x68, 0x03, 0x03, 0x68,
      0x53, 0xFE, 0xA6,
      0xF7,
      0x16};

  this->write_array(snd_ud, sizeof(snd_ud));

  ESP_LOGI(TAG, "SND_UD Init gesendet");
}

void UltraMaXXComponent::loop() {
  if (millis() - last_wakeup_ > 10000) {
    last_wakeup_ = millis();

    ESP_LOGI(TAG, "Wakeup start");

    send_wakeup_();
    send_init_frame_();

    waiting_for_frame_ = true;
  }

  // RX lesen (non-blocking!)
  while (available()) {
    uint8_t c;
    if (read_byte(&c)) {
      ESP_LOGD(TAG, "RX byte: 0x%02X", c);
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
