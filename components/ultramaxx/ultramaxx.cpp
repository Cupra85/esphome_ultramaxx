#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// Korrektur: Namen an Header angepasst (mit Unterstrich)
float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) {
float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    if (start + i >= data.size()) break;
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) {
uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    if (start + i >= data.size()) break;
  for (size_t i = 0; i < len; i++)
    v |= ((uint32_t)data[start + i]) << (8 * i);
  }
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;
  char buf[32];
  // Typisches M-Bus CP32 Format: Min, Hour, Day/Month, Month/Year
  sprintf(buf, "%02u.%02u.20%02u %02u:%02u",
    data[start+2] & 0x1F, data[start+3] & 0x0F, data[start+3] >> 4 | (data[start+2] >> 5 << 4), 
    data[start+1] & 0x1F, data[start] & 0x3F);
  out = buf;
  return true;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX gestartet");
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();
@@ -53,6 +63,7 @@
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }
@@ -72,9 +83,10 @@
  if (state == UM_WAIT && now - state_ts_ > 350) {
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();
    uint8_t d; while (this->available()) this->read_byte(&d);
    uint8_t d;
    while (this->available()) this->read_byte(&d);
    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, 5);
    this->write_array(reset, sizeof(reset));
    state = UM_SEND;
    state_ts_ = now;
  }
@@ -83,83 +95,87 @@
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl + 0xFE) & 0xFF;
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req, 5);
    this->write_array(req, sizeof(req));
    fcb_toggle_ = !fcb_toggle_;
    rx_buffer_.clear();
    state = UM_RX;
    state_ts_ = now;
  }

  if (state == UM_RX) {
    if (rx_buffer_.size() >= 12) { 
    if (rx_buffer_.size() > 12) {
      for (size_t i = 0; i + 6 < rx_buffer_.size(); i++) {
        if (rx_buffer_[i] == 0x68 && rx_buffer_[i+3] == 0x68) {
        // M-Bus Long Frame Check: 0x68 L L 0x68
        if (rx_buffer_[i] == 0x68 && rx_buffer_[i+1] == rx_buffer_[i+2] && rx_buffer_[i+3] == 0x68) {
          uint8_t len = rx_buffer_[i+1];
          size_t total_len = len + 6;
          
          if (rx_buffer_.size() < i + total_len) return; 
          if (rx_buffer_[i + total_len - 1] != 0x16) continue;
          if (rx_buffer_.size() < i + len + 6) continue;
          if (rx_buffer_[i + len + 5] != 0x16) continue;

          ESP_LOGI(TAG, "Frame erkannt (%d Bytes)", (int)total_len);
          ESP_LOGI(TAG, "Frame empfangen, starte Parsing...");

          size_t p = i + 6; 
          size_t end = i + total_len - 2;
          // UltraMaXX spezifischer Offset zu den Daten (nach dem Header)
          size_t p = i + 19; 
          size_t end = i + len + 4;

          while (p + 2 < end) {
            uint8_t dib = rx_buffer_[p];
            uint8_t vib = rx_buffer_[p+1];
            size_t v_len = 0;

            // Längenlogik basierend auf DIB
            uint8_t dib_low = dib & 0x07;
            if (dib_low == 0) v_len = 0;
            else if (dib_low == 1) v_len = 1;
            else if (dib_low == 2) v_len = 2;
            else if (dib_low == 3) v_len = 3;
            else if (dib_low == 4) v_len = 4;
            else if (dib_low == 5) v_len = 4;
            else if (dib_low == 6) v_len = 6;
            else if (dib_low == 7) v_len = 8;
            
            // Korrektur für deine spezifischen Ultramaxx-Felder
            if (dib == 0x0C) v_len = 4;
            if (dib == 0x0B) v_len = 3;
            if (dib == 0x0A) v_len = 2;

            p += 2; 

            if (dib == 0x0C && vib == 0x78) {
              if (serial_number_) serial_number_->publish_state(decode_bcd_(rx_buffer_, p, 4));
            } else if (dib == 0x04 && vib == 0x06) {
              if (total_energy_) total_energy_->publish_state(decode_u_le_(rx_buffer_, p, 4) * 0.001f);
            } else if (dib == 0x0C && vib == 0x14) {
              if (total_volume_) total_volume_->publish_state(decode_bcd_(rx_buffer_, p, 4) * 0.01f);
            } else if (dib == 0x0A && vib == 0x5A) {
              if (temp_flow_) temp_flow_->publish_state(decode_bcd_(rx_buffer_, p, 2) * 0.1f);
            } else if (dib == 0x0A && vib == 0x5E) {
              if (temp_return_) temp_return_->publish_state(decode_bcd_(rx_buffer_, p, 2) * 0.1f);
            } else if (dib == 0x0B && vib == 0x61) {
              if (temp_diff_) temp_diff_->publish_state(decode_bcd_(rx_buffer_, p, 3) * 0.01f);
            }
            uint8_t dif = rx_buffer_[p];
            uint8_t vif = rx_buffer_[p+1];

            p += v_len;
            // Energie: 0x0C 0x06 (BCD 8-stellig, kWh)
            if (dif == 0x0C && vif == 0x06) {
              if (total_energy_) total_energy_->publish_state(decode_bcd_(rx_buffer_, p+2, 4));
              p += 6;
            }
            // Volumen: 0x0C 0x13 (BCD 8-stellig, m3 * 0.001) oder 0x14 (0.01)
            else if (dif == 0x0C && (vif == 0x13 || vif == 0x14)) {
              float mult = (vif == 0x13) ? 0.001f : 0.01f;
              if (total_volume_) total_volume_->publish_state(decode_bcd_(rx_buffer_, p+2, 4) * mult);
              p += 6;
            }
            // Temperaturen: 0x0A 0x5A (Vorlauf), 0x0A 0x5E (Rücklauf) - BCD 4-stellig, 0.1°C
            else if (dif == 0x0A && vif == 0x5A) {
              if (temp_flow_) temp_flow_->publish_state(decode_bcd_(rx_buffer_, p+2, 2) * 0.1f);
              p += 4;
            }
            else if (dif == 0x0A && vif == 0x5E) {
              if (temp_return_) temp_return_->publish_state(decode_bcd_(rx_buffer_, p+2, 2) * 0.1f);
              p += 4;
            }
            // Temperaturdifferenz: 0x0B 0x61 (BCD 6-stellig, 0.01K)
            else if (dif == 0x0B && vif == 0x61) {
              if (temp_diff_) temp_diff_->publish_state(decode_bcd_(rx_buffer_, p+2, 3) * 0.01f);
              p += 5;
            }
            // Leistung: 0x0B 0x2B (BCD 6-stellig, 0.001 kW)
            else if (dif == 0x0B && vif == 0x2B) {
              if (current_power_) current_power_->publish_state(decode_bcd_(rx_buffer_, p+2, 3) * 0.001f);
              p += 5;
            }
            // Datum/Zeit: 0x04 0x6D
            else if (dif == 0x04 && vif == 0x6D) {
              std::string ts;
              if (meter_time_ && decode_cp32_datetime_(rx_buffer_, p+2, ts)) meter_time_->publish_state(ts);
              p += 6;
            }
            else {
              p++; // Unbekanntes Byte überspringen
            }
          }

          rx_buffer_.clear();
          state = UM_IDLE;
          ESP_LOGI(TAG, "Parsing abgeschlossen.");
          return;
        }
      }
    }

    if (now - state_ts_ > 8000) {
      ESP_LOGW(TAG, "RX Timeout - Buffer: %d Bytes", (int)rx_buffer_.size());
    if (now - state_ts_ > 10000) {
      ESP_LOGW(TAG, "Timeout beim Warten auf Frame");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
