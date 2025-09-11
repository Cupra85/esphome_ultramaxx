#pragma once

#include "esphome.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace ultramaxx {

class UltramaxxComponent : public Component, public PollingComponent {
 public:
  UARTComponent *uart_;
  UltramaxxComponent(UARTComponent *uart) : PollingComponent(86400 * 1000), uart_(uart) {}

  void setup() override {
    // Initialisierung
  }

  void update() override {
    // Wake-up + Daten auslesen
    if (uart_->available()) {
      uint8_t c = uart_->read();
      ESP_LOGD("ultramaxx", "Byte: %02X", c);
    }
  }

 protected:
  UARTComponent *uart_{nullptr};
  std::vector<std::pair<Sensor*, std::string>> sensors_;
};

}  // namespace ultramaxx
}  // namespace esphome
