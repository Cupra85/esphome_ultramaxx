#pragma once

#include "esphome.h"

namespace esphome {
namespace ultramaxx {

class UltraMaXXComponent : public Component, public uart::UARTDevice {
 public:
  UltraMaXXComponent(uart::UARTComponent *parent)
      : uart::UARTDevice(parent) {}

  void setup() override;
  void loop() override;
};

}  // namespace ultramaxx
}  // namespace esphome
