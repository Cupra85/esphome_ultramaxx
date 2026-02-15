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

  // 2400 8N1 – wie sensor53: sml(-1 1 "2400:8N1")
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();   // WICHTIG: echter UART-Reset

  wake_start = millis();
  last_send  = 0;

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // ------------------------------------------------
  // Wakeup: >1000 Bytes 0x55 für ~2,2s
  // ------------------------------------------------
  if (state == UM_WAKEUP) {

    if (now - last_send >= 15) {
      uint8_t buf[20];
      for (int i = 0; i < 20; i++) buf[i] = 0x55;
      this->write_array(buf, sizeof(buf));
      last_send = now;
    }

    if (now - wake_start >= 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts = now;
    }
  }

  // ------------------------------------------------
  // Pause 350 ms + Switch auf 2400 8E1
  // ------------------------------------------------
  if (state == UM_WAIT && now - state_ts >= 350) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_baud_rate(2400);
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();   // exakt wie sml(-1 1 "2400:8E1")

    state = UM_SEND;
  }

  // ------------------------------------------------
  // SND_UD Init: 10 5B FE 59 16
  // ------------------------------------------------
  if (state == UM_SEND) {

    uint8_t snd_ud[] = {0x10, 0x5B, 0xFE, 0x59, 0x16};
    this->write_array(snd_ud, sizeof(snd_ud));

    // TX freigeben (wichtig bei Optolink!)
    this->flush();

    ESP_LOGI(TAG, "SND_UD Init gesendet");

    state = UM_RX;
    state_ts = millis();
  }

  // ------------------------------------------------
  // RX-Fenster (UltraMaXX antwortet teils spät)
  // ------------------------------------------------
  if (state == UM_RX) {

    int avail = available();
    if (avail > 0) {
      ESP_LOGI(TAG, "RX available bytes: %d", avail);
    }

    while (available()) {
      uint8_t c;
      if (read_byte(&c)) {
        ESP_LOGI(TAG, "RX: 0x%02X", c);
      }
    }

    if (millis() - state_ts >= 6000) {
      state = UM_IDLE;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
