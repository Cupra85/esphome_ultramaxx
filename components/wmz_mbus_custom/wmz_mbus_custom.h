#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <driver/uart.h>
#include <vector>

namespace esphome {
namespace wmz_mbus_custom {

class WMZComponent : public PollingComponent {
 public:
  WMZComponent(int tx_pin, int rx_pin, uint32_t update_interval_ms);

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  sensor::Sensor heat_energy_kwh_;
  sensor::Sensor volume_m3_;
  sensor::Sensor flow_temp_c_;
  sensor::Sensor return_temp_c_;
  sensor::Sensor power_w_;
  sensor::Sensor flow_lh_;

 protected:
  int tx_pin_;
  int rx_pin_;
  uart_port_t uart_num_ = UART_NUM_1;

  void uart_configure(int baud, uart_parity_t parity);
  void uart_write(const uint8_t *data, size_t len, uint32_t wait_ms = 100);
  int  uart_read(uint8_t *data, size_t maxlen, uint32_t timeout_ms);
  void wake_up();
  bool send_init_expect_e5();
  void send_req_ud2();
  std::vector<uint8_t> read_frame(uint32_t window_ms);
  void parse_and_publish(const std::vector<uint8_t> &buf);

  static uint32_t u32le(const uint8_t *p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
  }
};

}  // namespace wmz_mbus_custom
}  // namespace esphome
