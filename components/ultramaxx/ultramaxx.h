#pragma once

#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
@@ -27,9 +30,9 @@
  void set_meter_time_sensor(text_sensor::TextSensor *s) { meter_time_ = s; }

 protected:
  float decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len);
  uint32_t decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len);
  bool decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out);
  float decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const;
  uint32_t decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const;
  bool decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const;

  sensor::Sensor *serial_number_{nullptr};
  sensor::Sensor *total_energy_{nullptr};
@@ -44,10 +47,11 @@
  uint32_t wake_start_{0};
  uint32_t last_send_{0};
  uint32_t last_rx_byte_{0};

  bool fcb_toggle_{false};

  std::vector<uint8_t> rx_buffer_;
};

}  // namespace ultramaxx
}  // namespace esphome
