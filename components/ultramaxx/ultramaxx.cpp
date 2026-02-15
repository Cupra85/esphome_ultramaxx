#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");

  // -------------------------------------------------
  // Wakeup + Init Zyklus exakt nach mikrocontroller.net
  // -------------------------------------------------
  set_interval("ultramaxx_cycle", 10000, [this]() {

    ESP_LOGI(TAG, "Wakeup start");

    // 1️⃣ Wakeup: 2400 8N1
    uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE);

    uint8_t wake = 0x55;

    // ~3 Sekunden Wakeup (nicht blockierend genug für ESPHome)
    for (int i = 0; i < 360; i++) {
      this->write_array(&wake, 1);
      delay(2);   // klein halten!
    }

    // 2️⃣ Pause laut Thread (~234 ms)
    delay(234);

    // 3️⃣ zurück auf 8E1
    uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);

    // 4️⃣ SND_UD Init (CI=A6)
    uint8_t snd_ud[] = {
        0x68, 0x03, 0x03, 0x68,
        0x53, 0xFE, 0xA6,
        0xF7,
        0x16};

    this->write_array(snd_ud, sizeof(snd_ud));

    ESP_LOGI(TAG, "SND_UD Init gesendet");
  });
}

void UltraMaXXComponent::loop() {
  // -------------------------------------------------
  // RX nicht blockierend lesen
  // -------------------------------------------------
  while (available()) {
    uint8_t c;
    if (read_byte(&c)) {
      ESP_LOGD(TAG, "RX byte: 0x%02X", c);
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
