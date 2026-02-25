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

  void set_serial_number_sensor(sensor::Sensor *s){serial_number_=s;}
  void set_total_energy_sensor(sensor::Sensor *s){total_energy_=s;}
  void set_total_volume_sensor(sensor::Sensor *s){total_volume_=s;}
  void set_current_power_sensor(sensor::Sensor *s){current_power_=s;}
  void set_flow_sensor(sensor::Sensor *s){flow_=s;}
  void set_temp_flow_sensor(sensor::Sensor *s){temp_flow_=s;}
  void set_temp_return_sensor(sensor::Sensor *s){temp_return_=s;}
  void set_temp_diff_sensor(sensor::Sensor *s){temp_diff_=s;}
  void set_operating_time_sensor(sensor::Sensor *s){operating_time_=s;}
  void set_firmware_version_sensor(sensor::Sensor *s){firmware_version_=s;}
  void set_software_version_sensor(sensor::Sensor *s){software_version_=s;}
  void set_access_counter_sensor(sensor::Sensor *s){access_counter_=s;}

  void set_meter_time_sensor(text_sensor::TextSensor *s){meter_time_=s;}
  void set_status_text_sensor(text_sensor::TextSensor *s){status_text_=s;}

 protected:
  float decode_bcd_(const std::vector<uint8_t>&,size_t,size_t) const;
  uint32_t decode_u_le_(const std::vector<uint8_t>&,size_t,size_t) const;
  bool decode_cp32_datetime_(const std::vector<uint8_t>&,size_t,std::string&) const;

  void parse_and_publish_(const std::vector<uint8_t>&);
  void reset_parse_flags_();
  std::string decode_status_text_(uint8_t) const;

  sensor::Sensor *serial_number_{};
  sensor::Sensor *total_energy_{};
  sensor::Sensor *total_volume_{};
  sensor::Sensor *current_power_{};
  sensor::Sensor *flow_{};
  sensor::Sensor *temp_flow_{};
  sensor::Sensor *temp_return_{};
  sensor::Sensor *temp_diff_{};
  sensor::Sensor *operating_time_{};
  sensor::Sensor *firmware_version_{};
  sensor::Sensor *software_version_{};
  sensor::Sensor *access_counter_{};

  text_sensor::TextSensor *meter_time_{};
  text_sensor::TextSensor *status_text_{};

  bool got_power_{false};
  bool got_flow_{false};
  bool got_fw_{false};
  bool got_sw_{false};
};

}
}
