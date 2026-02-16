#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace ultramaxx {

// Wir nutzen UARTDevice für die Kommunikation und PollingComponent für das Intervall
class UltraMaXXComponent : public PollingComponent, public uart::UARTDevice {
 public:
  explicit UltraMaXXComponent(uart::UARTComponent *parent)
      : PollingComponent(20000), uart::UARTDevice(parent) {}
  // Der Konstruktor bekommt den Parent-UART übergeben (wichtig für S3)
  UltraMaXXComponent(uart::UARTComponent *parent) : PollingComponent(), uart::UARTDevice(parent) {}

  void setup() override;
  void update() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Setter-Methoden für die sensor.py
  void set_serial_number_sensor(sensor::Sensor *s) { serial_number_ = s; }
  void set_total_energy_sensor(sensor::Sensor *s) { total_energy_ = s; }
  void set_total_volume_sensor(sensor::Sensor *s) { total_volume_ = s; }
@@ -24,6 +30,9 @@ class UltraMaXXComponent : public PollingComponent, public uart::UARTDevice {
  void set_meter_time_sensor(text_sensor::TextSensor *s) { meter_time_ = s; }

 protected:
  // Hilfsfunktion zum Dekodieren von BCD-Werten (typisch für Allmess/M-Bus)
  float decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len);

  sensor::Sensor *serial_number_{nullptr};
  sensor::Sensor *total_energy_{nullptr};
  sensor::Sensor *total_volume_{nullptr};
@@ -32,7 +41,14 @@ class UltraMaXXComponent : public PollingComponent, public uart::UARTDevice {
  sensor::Sensor *temp_return_{nullptr};
  sensor::Sensor *temp_diff_{nullptr};
  text_sensor::TextSensor *meter_time_{nullptr};

  // Status-Variablen für die State-Machine in der cpp
  uint8_t state_{0};
  uint32_t state_ts_{0};
  uint32_t wake_start_{0};
  uint32_t last_send_{0};
  std::vector<uint8_t> rx_buffer_;
};

}  
}
}  // namespace ultramaxx
}  // namespace esphome
