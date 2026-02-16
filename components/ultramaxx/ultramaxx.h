#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace ultramaxx {

class UltraMaXXComponent : public PollingComponent, public uart::UARTDevice {
 public:
  // ⭐ update_interval kommt aus sensor.py → kein PollingComponent(...)
  UltraMaXXComponent(uart::UARTComponent *parent)
      : uart::UARTDevice(parent) {}
  UltraMaXXComponent(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  void setup() override;
  void update() override;
  void loop() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  // Setter (werden aus sensor.py aufgerufen)
  // Diese Namen müssen mit sensor.py übereinstimmen
  void set_serial_number_sensor(sensor::Sensor *s) { serial_number_ = s; }
  void set_total_energy_sensor(sensor::Sensor *s) { total_energy_ = s; }
  void set_total_volume_sensor(sensor::Sensor *s) { total_volume_ = s; }
@@ -31,7 +28,7 @@
  void set_meter_time_sensor(text_sensor::TextSensor *s) { meter_time_ = s; }

 protected:
  // ⭐ const reference (besser)
  // Ändere dies in der .cpp Datei ebenfalls zu 'const std::vector<uint8_t> &data'
  float decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len);

  sensor::Sensor *serial_number_{nullptr};
@@ -43,13 +40,13 @@
  sensor::Sensor *temp_diff_{nullptr};
  text_sensor::TextSensor *meter_time_{nullptr};

  uint8_t state_{0};
  uint32_t state_ts_{0};
  uint32_t wake_start_{0};
  uint32_t last_send_{0};
  uint32_t last_rx_byte_{0}; 

  std::vector<uint8_t> rx_buffer_;
};

}  
}
