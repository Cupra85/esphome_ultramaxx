#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
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

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);

  wake_start = millis();
  last_send = 0;

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // ------------------------------------------------
  // Wakeup (~2.2s)
  // ------------------------------------------------
  if (state == UM_WAKEUP) {

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
  // Pause + Switch to 8E1
  // ------------------------------------------------
  if (state == UM_WAIT && now - state_ts > 100) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);

    state = UM_REQ;
  }

  // ------------------------------------------------
  // SND_NKE + REQ_UD2
  // ------------------------------------------------
  if (state == UM_REQ) {

    uint8_t reset[] = {0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    ESP_LOGI(TAG, "SND_NKE gesendet");

    delay(50);

    uint8_t req[] = {0x10,0x7B,0xFE,0x79,0x16};
    this->write_array(req,sizeof(req));
    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    state = UM_RX;
    state_ts = millis();
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

    if (now - state_ts > 2000) {
      state = UM_IDLE;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
