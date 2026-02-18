#pragma once

#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace ultramaxx {

class UltraMaXXComponent : public PollingComponent, public uart::UARTDevice {
 public:
  explicit UltraMaXXComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  void set_serial_number_sensor(sensor::Sensor *s) { serial_number_ = s; }
  void set_total_energy_sensor(sensor::Sensor *s) { total_energy_ = s; }
  void set_total_volume_sensor(sensor::Sensor *s) { total_volume_ = s; }
  void set_current_power_sensor(sensor::Sensor *s) { current_power_ = s; }
  void set_temp_flow_sensor(sensor::Sensor *s) { temp_flow_ = s; }
  void set_temp_return_sensor(sensor::Sensor *s) { temp_return_ = s; }
  void set_temp_diff_sensor(sensor::Sensor *s) { temp_diff_ = s; }
  void set_meter_time_sensor(text_sensor::TextSensor *s) { meter_time_ = s; }

  void setup() override;
  void update() override;
  void loop() override;

 protected:
  enum UMState : uint8_t { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };

  float decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const;
  uint32_t decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const;
  bool decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const;
  bool extract_long_frame_(const std::vector<uint8_t> &buf, std::vector<uint8_t> &out_frame) const;

  sensor::Sensor *serial_number_{nullptr};
  sensor::Sensor *total_energy_{nullptr};
  sensor::Sensor *total_volume_{nullptr};
  sensor::Sensor *current_power_{nullptr};
  sensor::Sensor *temp_flow_{nullptr};
  sensor::Sensor *temp_return_{nullptr};
  sensor::Sensor *temp_diff_{nullptr};
  text_sensor::TextSensor *meter_time_{nullptr};

  UMState state_{UM_IDLE};
  uint32_t state_ts_{0};
  uint32_t wake_start_{0};
  uint32_t last_send_{0};
  uint32_t last_rx_byte_{0};
  bool fcb_toggle_{false};
  std::vector<uint8_t> rx_buffer_;
};

}  // namespace ultramaxx
}  // namespace esphome
