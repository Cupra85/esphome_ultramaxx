#pragma once

#include "esphome.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace wmz_mbus_custom {

class WMZMbusCustom : public Component, public PollingComponent {
 public:
  UARTComponent *uart_;
  WMZMbusCustom(UARTComponent *uart) : PollingComponent(86400 * 1000), uart_(uart) {}

  void setup() override {
    // initialisierung
  }

  void update() override {
    // wake-up + daten auslesen
    if(uart_->available()) {
      uint8_t c = uart_->read();
      ESP_LOGD("wmz_mbus", "Byte: %02X", c);
    }
  }

 protected:
  UARTComponent *uart_{nullptr};
  std::vector<std::pair<Sensor*, std::string>> sensors_;
};

}  // namespace wmz_mbus_custom
}  // namespace esphome
