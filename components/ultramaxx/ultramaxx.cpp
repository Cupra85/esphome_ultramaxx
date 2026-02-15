#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_SEND,
  UM_RX
};

static UMState state = UM_IDLE;

static uint32_t wake_start = 0;
static uint32_t last_send = 0;
static uint32_t state_ts = 0;

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup Mode 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);

  wake_start = millis();
  last_send = 0;

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // ------------------------------------------------
  // Wakeup (53x 0x55 ≈ 2.2s)
  // ------------------------------------------------
  if (state == UM_WAKEUP) {

    if (now - last_send > 40) {   // entspricht ungefähr deinem for-loop
      uint8_t buf[10] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};
      this->write_array(buf,10);
      last_send = now;
    }

    if (now - wake_start > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts = now;
    }
  }

  // ------------------------------------------------
  // Pause 350 ms (wie Script!)
  // ------------------------------------------------
  if (state == UM_WAIT && now - state_ts > 350) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);

    state = UM_SEND;
  }

  // ------------------------------------------------
  // ⭐ SND_UD (105BFE5916) — exakt wie Script
  // ------------------------------------------------
  if (state == UM_SEND) {

    uint8_t req[] = {0x10,0x5B,0xFE,0x59,0x16};
    this->write_array(req,sizeof(req));

    ESP_LOGI(TAG, "SND_UD Init gesendet");

    state = UM_RX;
    state_ts = now;
  }

  // ------------------------------------------------
  // RX lesen
  // ------------------------------------------------
  if (state == UM_RX) {

    int count = available();
    if (count > 0) {
      ESP_LOGI(TAG, "RX available bytes: %d", count);
    }

    while (available()) {
      uint8_t c;
      if (read_byte(&c)) {
        ESP_LOGI(TAG, "RX: 0x%02X", c);
      }
    }

    if (now - state_ts > 3000) {
      state = UM_IDLE;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
