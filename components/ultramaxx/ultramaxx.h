#pragma once

#include "esphome.h"

namespace esphome {
namespace ultramaxx {

class UltraMaXXComponent : public PollingComponent, public uart::UARTDevice {
 public:
  UltraMaXXComponent(uart::UARTComponent *parent)
      : PollingComponent(20000), uart::UARTDevice(parent) {}  // default 20s

  void setup() override;
  void update() override;
  void loop() override;
};

}  // namespace ultramaxx
}  // namespace esphome
