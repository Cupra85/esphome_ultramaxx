#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace ultramaxx {

class UltraMaXXComponent : public PollingComponent, public uart::UARTDevice {
 public:
  UltraMaXXComponent(uart::UARTComponent *parent) : PollingComponent(), uart::UARTDevice(parent) {}

  void setup() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_serial_number_sensor(sensor::Sensor *s) { serial_number_ = s; }
  void set_total_energy_sensor(sensor::Sensor *s) { total_energy_ = s; }
  void set_total_volume_sensor(sensor::Sensor *s) { total_volume_ = s; }
  void set_current_power_sensor(sensor::Sensor *s) { current_power_ = s; }
  void set_temp_flow_sensor(sensor::Sensor *s) { temp_flow_ = s; }
  void set_temp_return_sensor(sensor::Sensor *s) { temp_return_ = s; }
  void set_temp_diff_sensor(sensor::Sensor *s) { temp_diff_ = s; }
  void set_meter_time_sensor(text_sensor::TextSensor *s) { meter_time_ = s; }

 protected:
  // Decoder
  float decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const;
  uint32_t decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const;
  bool decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const;

  // Parser / publisher
  void parse_and_publish_(const std::vector<uint8_t> &buf);
  void reset_parse_flags_();

  // Sensor pointers
  sensor::Sensor *serial_number_{nullptr};
  sensor::Sensor *total_energy_{nullptr};
  sensor::Sensor *total_volume_{nullptr};
  sensor::Sensor *current_power_{nullptr};   // (falls später ergänzt)
  sensor::Sensor *temp_flow_{nullptr};
  sensor::Sensor *temp_return_{nullptr};
  sensor::Sensor *temp_diff_{nullptr};
  text_sensor::TextSensor *meter_time_{nullptr};

  // State machine timing
  uint32_t state_ts_{0};
  uint32_t wake_start_{0};
  uint32_t last_send_{0};

  // RX frame handling
  bool in_frame_{false};
  size_t expected_len_{0};
  uint32_t last_rx_ms_{0};
  bool fcb_toggle_{false};
  std::vector<uint8_t> rx_buffer_;

  // Publish guards per cycle (damit nicht 100x publish im selben Telegram)
  bool got_serial_{false};
  bool got_energy_{false};
  bool got_volume_{false};
  bool got_tflow_{false};
  bool got_tret_{false};
  bool got_tdiff_{false};
  bool got_time_{false};
};

}  // namespace ultramaxx
}  // namespace esphome
