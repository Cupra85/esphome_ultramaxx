#include "ultramaxx.h"
#include "driver/uart.h" // Wichtig für Register-Zugriffe

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_RESET,
  UM_SEND,
  UM_RX
};

static UMState state = UM_IDLE;
static uint32_t wake_start = 0;
static uint32_t last_send  = 0;
static uint32_t state_ts   = 0;

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // UART Initialisierung für Wakeup (8N1)
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // ⭐ S3-Spezifisch: Invertierung NACH load_settings setzen
  // Wir nutzen UART_NUM_1 (entspricht meistens deinem bus_uart)
  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

  wake_start = millis();
  last_send  = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  uint32_t now = millis();

  // 1. PHASE: WAKEUP (2.2 Sekunden 0x55)
  if (state == UM_WAKEUP) {
    if (now - last_send >= 15) {
      uint8_t buf[20];
      for (int i=0; i<20; i++) buf[i]=0x55;
      this->write_array(buf, 20);
      last_send = now;
    }

    if (now - wake_start >= 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts = now;
    }
  }

  // 2. PHASE: SWITCH UART (350ms Pause laut Tasmota)
  if (state == UM_WAIT && now - state_ts >= 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_baud_rate(2400);
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // ⭐ Erneut Invertierung setzen, da load_settings die Register resettet
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

    // Puffer radikal leeren (Echo des Wakeups entfernen)
    uint8_t dummy;
    while (this->available()) this->read_byte(&dummy);

    state = UM_RESET;
  }

  // 3. PHASE: SND_NKE (Reset an Broadcast FE)
  if (state == UM_RESET) {
    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts = millis();
  }

  // 4. PHASE: SND_UD (Eigentlicher Trigger)
  if (state == UM_SEND && millis() - state_ts > 50) {
    // Vor dem Senden nochmal kurz Puffer leeren (Echo-Vermeidung)
    uint8_t dummy;
    while (this->available()) this->read_byte(&dummy);

    uint8_t snd_ud[] = {0x10, 0x5B, 0xFE, 0x59, 0x16};
    this->write_array(snd_ud, sizeof(snd_ud));
    this->flush();

    ESP_LOGI(TAG, "SND_UD Init gesendet");

    state = UM_RX;
    state_ts = millis();
  }

  // 5. PHASE: EMPFANG
  if (state == UM_RX) {
    // Wir warten etwas länger (bis zu 6 Sek), da der Zähler träge ist
    if (this->available()) {
      ESP_LOGI(TAG, "Daten empfangen! Lese Bytes...");
      while (this->available()) {
        uint8_t c;
        if (this->read_byte(&c)) {
          ESP_LOGI(TAG, "RX: 0x%02X", c);
        }
      }
    }

    if (millis() - state_ts > 6000) {
      ESP_LOGI(TAG, "Lese-Fenster geschlossen. Gehe zu IDLE.");
      state = UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
