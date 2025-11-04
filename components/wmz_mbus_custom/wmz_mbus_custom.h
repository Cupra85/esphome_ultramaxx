#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include <driver/uart.h>
#include <vector>
#include "esphome/components/text_sensor/text_sensor.h"

sensor::TextSensor *debug_frame_ = new sensor::TextSensor();


namespace esphome {
namespace wmz_mbus_custom {

class WMZComponent : public PollingComponent {
 public:
  WMZComponent(int tx_pin, int rx_pin, uint32_t update_interval_ms);

  void setup() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Sensor-Pointer (wichtig für ESPHome)
  sensor::Sensor *heat_energy_kwh_ = new sensor::Sensor();
  sensor::Sensor *volume_m3_       = new sensor::Sensor();
  sensor::Sensor *flow_temp_c_     = new sensor::Sensor();
  sensor::Sensor *return_temp_c_   = new sensor::Sensor();
  sensor::Sensor *power_w_         = new sensor::Sensor();
  sensor::Sensor *flow_lh_         = new sensor::Sensor();

 protected:
  int tx_pin_;
  int rx_pin_;
  uart_port_t uart_num_ = UART_NUM_1;

  // UART Helper
  void uart_configure(int baud, uart_parity_t parity);
  void uart_write(const uint8_t *data, size_t len, uint32_t wait_ms = 100);
  int  uart_read(uint8_t *data, size_t maxlen, uint32_t timeout_ms);

  // Kommunikationsablauf
  void wake_up();                 // 0x55 für 2.2s @ 2400 Bd 8N1
  bool send_init_expect_e5();     // SND_NKE → E5
  void send_req_ud2();            // REQ_UD2 senden
  std::vector<uint8_t> read_frame(uint32_t window_ms);
  void parse_and_publish(const std::vector<uint8_t> &buf);

  // kleine Helfer
  static uint32_t u32le(const uint8_t *p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
  }
};

}  // namespace wmz_mbus_custom
}  // namespace esphome
