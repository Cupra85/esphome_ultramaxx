#pragma once

#include "esphome.h"

namespace esphome {
namespace ultramaxx {

class UltraMaXXComponent : public PollingComponent, public uart::UARTDevice {
 public:
  explicit UltraMaXXComponent(uart::UARTComponent *parent)
      : PollingComponent(20000), uart::UARTDevice(parent) {}

  void setup() override;
  void update() override;
  void loop() override;

  void set_serial_number_sensor(sensor::Sensor *s) { serial_number_ = s; }
  void set_total_energy_sensor(sensor::Sensor *s) { total_energy_ = s; }
  void set_total_volume_sensor(sensor::Sensor *s) { total_volume_ = s; }
  void set_current_power_sensor(sensor::Sensor *s) { current_power_ = s; }
  void set_temp_flow_sensor(sensor::Sensor *s) { temp_flow_ = s; }
  void set_temp_return_sensor(sensor::Sensor *s) { temp_return_ = s; }
  void set_temp_diff_sensor(sensor::Sensor *s) { temp_diff_ = s; }
  void set_meter_time_sensor(text_sensor::TextSensor *s) { meter_time_ = s; }

 protected:
  sensor::Sensor *serial_number_{nullptr};
  sensor::Sensor *total_energy_{nullptr};
  sensor::Sensor *total_volume_{nullptr};
  sensor::Sensor *current_power_{nullptr};
  sensor::Sensor *temp_flow_{nullptr};
  sensor::Sensor *temp_return_{nullptr};
  sensor::Sensor *temp_diff_{nullptr};
  text_sensor::TextSensor *meter_time_{nullptr};
};

}  
}
