#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE = 0,
  UM_WAKEUP_300,
  UM_WAIT_AFTER_WAKE,
  UM_SWITCH_2400,
  UM_SEND_NKE,
  UM_SEND_A6,
  UM_RX
};

static UMState state = UM_IDLE;
static uint32_t state_ts = 0;
static uint16_t wake_cnt = 0;

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // -------------------------------------------------
  // Alle 10 Sekunden neuer Zyklus
  // -------------------------------------------------
  if (state == UM_IDLE && now - state_ts > 10000) {

    ESP_LOGI(TAG, "Wakeup start (300 Baud)");

    uart_set_baudrate(UART_NUM_1, 300);
    uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE); // 8N1

    wake_cnt = 0;
    state = UM_WAKEUP_300;
    state_ts = now;
  }

  // -------------------------------------------------
  // Wakeup bei 300 Baud (~2.2s)
  // -------------------------------------------------
  if (state == UM_WAKEUP_300) {

    if (now - state_ts > 20) {   // langsam senden bei 300 Baud

      uint8_t wake = 0x55;
      this->write_array(&wake,1);

      wake_cnt++;
      state_ts = now;

      if (wake_cnt >= 110) {     // ca 2.2s
        state = UM_WAIT_AFTER_WAKE;
        state_ts = now;
      }
    }
  }

  // -------------------------------------------------
  // Pause nach Wakeup (~250ms)
  // -------------------------------------------------
  if (state == UM_WAIT_AFTER_WAKE && now - state_ts > 250) {
    state = UM_SWITCH_2400;
  }

  // -------------------------------------------------
  // Wechsel auf 2400 8E1
  // -------------------------------------------------
  if (state == UM_SWITCH_2400) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    uart_set_baudrate(UART_NUM_1, 2400);
    uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);

    state = UM_SEND_NKE;
    state_ts = now;
  }

  // -------------------------------------------------
  // SND_NKE senden
  // -------------------------------------------------
  if (state == UM_SEND_NKE) {

    uint8_t snd_nke[] = {0x10,0x40,0x00,0x40,0x16};
    this->write_array(snd_nke, sizeof(snd_nke));

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND_A6;
    state_ts = now;
  }

  // -------------------------------------------------
  // CI=A6 Init senden
  // -------------------------------------------------
  if (state == UM_SEND_A6 && now - state_ts > 50) {

    uint8_t snd_ud[] = {
      0x68,0x03,0x03,0x68,
      0x53,0xFE,0xA6,
      0xF7,
      0x16
    };

    this->write_array(snd_ud, sizeof(snd_ud));

    ESP_LOGI(TAG, "SND_UD Init gesendet");

    state = UM_RX;
  }

  // -------------------------------------------------
  // RX lesen (non blocking!)
  // -------------------------------------------------
  while (available()) {
    uint8_t c;
    if (read_byte(&c)) {
      ESP_LOGI(TAG, "RX byte: 0x%02X", c);
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
