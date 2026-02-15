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

static uint32_t wake_start = 0;
static uint32_t last_send = 0;
static uint32_t state_ts = 0;

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // ------------------------------------------------
  // Start Zyklus alle 10 Sekunden
  // ------------------------------------------------
  if (state == UM_IDLE && now - state_ts > 10000) {

    ESP_LOGI(TAG, "Wakeup start (2400 8N1)");

    uart_set_baudrate(UART_NUM_1, 2400);
    uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE); // 8N1

    wake_start = now;
    last_send = 0;
    state = UM_WAKEUP;
  }

  // ------------------------------------------------
  // Wakeup ca. 2.2 Sekunden
  // ------------------------------------------------
  if (state == UM_WAKEUP) {

    // ⭐ geändert: 12ms statt 5ms (Scheduler entlasten)
    if (now - last_send > 12) {
      uint8_t b = 0x55;
      this->write_array(&b,1);
      last_send = now;
    }

    if (now - wake_start > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts = now;
    }
  }

  // ------------------------------------------------
  // Pause nach Wakeup (~100ms)
  // ------------------------------------------------
  if (state == UM_WAIT && now - state_ts > 100) {
    state = UM_SWITCH;
  }

  // ------------------------------------------------
  // Wechsel zu 2400 8E1
  // ------------------------------------------------
  if (state == UM_SWITCH) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);

    state = UM_REQ;
  }

  // ------------------------------------------------
  // REQ_UD2 senden
  // ------------------------------------------------
  if (state == UM_REQ) {

    uint8_t req[] = {0x10,0x7B,0xFE,0x79,0x16};
    this->write_array(req,sizeof(req));

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    state = UM_RX;
    state_ts = now;   // ⭐ wichtig: Start RX-Zeitfenster
  }

  // ------------------------------------------------
  // RX lesen (2 Sekunden aktiv lauschen)
  // ------------------------------------------------
  if (state == UM_RX && now - state_ts < 2000) {

    while (available()) {
      uint8_t c;
      if (read_byte(&c)) {
        ESP_LOGI(TAG, "RX: 0x%02X", c);
      }
    }
  }

  // ------------------------------------------------
  // Danach wieder Idle
  // ------------------------------------------------
  if (state == UM_RX && now - state_ts >= 2000) {
    state = UM_IDLE;
    state_ts = now;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
