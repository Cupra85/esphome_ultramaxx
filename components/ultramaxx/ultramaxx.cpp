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

static uart_port_t uart_port_used;

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");

  auto *p = reinterpret_cast<uart::ESP32UartComponent *>(this->get_parent());
  uart_port_used = (uart_port_t)p->get_uart_num();

  ESP_LOGI(TAG, "Using UART_NUM_%d", (int)uart_port_used);
}

//
// ⭐ Wird automatisch alle update_interval Sekunden aufgerufen!
//
void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START ===");

  uart_set_baudrate(uart_port_used, 2400);
  uart_set_parity(uart_port_used, UART_PARITY_DISABLE);

  wake_start = millis();
  last_send = 0;

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // ------------------------------------------------
  // Wakeup senden (~2.2s)
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
  // Pause
  // ------------------------------------------------
  if (state == UM_WAIT && now - state_ts > 100) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    uart_set_parity(uart_port_used, UART_PARITY_EVEN);

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
    state_ts = now;
  }

  // ------------------------------------------------
  // RX lesen
  // ------------------------------------------------
  if (state == UM_RX) {

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
