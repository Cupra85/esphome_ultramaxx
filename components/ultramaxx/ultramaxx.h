#pragma once

#include "esphome.h"

namespace esphome {
namespace ultramaxx {

class UltraMaXXComponent : public Component, public uart::UARTDevice {
 public:
  UltraMaXXComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  void setup() override;
  void loop() override;

 protected:
  uint32_t last_wakeup_{0};
  bool waiting_for_frame_{false};

  void send_wakeup_();
  void send_init_frame_();
};

}  // namespace ultramaxx
}  // namespace esphome
