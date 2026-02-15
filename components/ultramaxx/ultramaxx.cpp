#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_SWITCH,
  UM_REQ,
  UM_RX
};

static UMState state = UM_IDLE;
static uint32_t ts = 0;
static uint16_t wake_bytes = 0;

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // ---------------------------------------------
  // Start Zyklus
  // ---------------------------------------------
  if (state == UM_IDLE && now - ts > 10000) {

    ESP_LOGI(TAG, "Wakeup start (2400 8N1)");

    uart_set_baudrate(UART_NUM_1, 2400);
    uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE); // 8N1

    wake_bytes = 0;
    ts = now;
    state = UM_WAKEUP;
  }

  // ---------------------------------------------
  // Wakeup senden (0x55 ~2.2s)
  // ---------------------------------------------
  if (state == UM_WAKEUP && now - ts > 4) {

    uint8_t wake = 0x55;
    this->write_array(&wake,1);

    wake_bytes++;
    ts = now;

    if (wake_bytes >= 530) {   // wie Script
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      ts = now;
    }
  }

  // ---------------------------------------------
  // Pause ~100ms
  // ---------------------------------------------
  if (state == UM_WAIT && now - ts > 100) {
    state = UM_SWITCH;
  }

  // ---------------------------------------------
  // Switch zu 2400 8E1
  // ---------------------------------------------
  if (state == UM_SWITCH) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);

    state = UM_REQ;
    ts = now;
  }

  // ---------------------------------------------
  // REQ_UD2 senden
  // ---------------------------------------------
  if (state == UM_REQ) {

    uint8_t req[] = {0x10,0x7B,0xFE,0x79,0x16};
    this->write_array(req,sizeof(req));

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    state = UM_RX;
  }

  // ---------------------------------------------
  // RX lesen
  // ---------------------------------------------
  while (available()) {
    uint8_t c;
    if (read_byte(&c)) {
      ESP_LOGI(TAG, "RX: 0x%02X", c);
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
