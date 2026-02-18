#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++)
    v |= ((uint32_t)data[start + i]) << (8 * i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;
  char buf[32];
  sprintf(buf, "%02u.%02u.20%02u %02u:%02u",
    data[start+2] & 0x1F, data[start+3] & 0x0F, 
    (data[start+3] >> 4) | ((data[start+2] >> 5) << 4), 
    data[start+1] & 0x1F, data[start] & 0x3F);
  out = buf;
  return true;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
  ESP_LOGI(TAG, "Starte Wakeup-Sequenz...");
}

void UltraMaXXComponent::loop() {
  uint32_t now = millis();

  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  if (state == UM_WAKEUP) {
    if (now - last_send_ > 20) {
      uint8_t buf[20]; memset(buf, 0x55, 20);
      this->write_array(buf, 20);
      last_send_ = now;
    }
    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup Ende -> Warte auf Umschaltung");
      state = UM_WAIT;
      state_ts_ = now;
    }
  }

  if (state == UM_WAIT && now - state_ts_ > 400) {
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();
    
    // Buffer leeren
    uint8_t dummy; while (this->available()) this->read_byte(&dummy);

    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    ESP_LOGI(TAG, "SND_NKE gesendet");
    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND && now - state_ts_ > 200) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl + 0xFE) & 0xFF;
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req, sizeof(req));
    ESP_LOGI(TAG, "REQ_UD2 gesendet, warte auf Daten...");
    fcb_toggle_ = !fcb_toggle_;
    rx_buffer_.clear();
    state = UM_RX;
    state_ts_ = now;
  }

  if (state == UM_RX) {
    // Wenn wir Daten haben und seit 200ms nichts mehr kam, ist der Frame wohl fertig
    if (!rx_buffer_.empty() && (now - last_rx_byte_ > 200)) {
      
      for (size_t i = 0; i + 10 < rx_buffer_.size(); i++) {
        // Suche nach 0x68 L L 0x68
        if (rx_buffer_[i] == 0x68 && rx_buffer_[i+3] == 0x68 && rx_buffer_[i+1] == rx_buffer_[i+2]) {
          
          uint8_t L = rx_buffer_[i+1];
          ESP_LOGI(TAG, "M-Bus Frame mit Länge %d gefunden!", L);

          size_t p = i + 19; 
          size_t end = i + L + 4; // Ende vor CS

          while (p + 2 < rx_buffer_.size() && p < end) {
            uint8_t dif = rx_buffer_[p];
            uint8_t vif = rx_buffer_[p+1];

            if (dif == 0x0C && vif == 0x78) {
              if (serial_number_) serial_number_->publish_state(decode_bcd_(rx_buffer_, p+2, 4));
              p += 6;
            }
            else if (dif == 0x04 && vif == 0x06) {
              if (total_energy_) total_energy_->publish_state(decode_u_le_(rx_buffer_, p+2, 4) * 0.001f);
              p += 6;
            }
            else if (dif == 0x0C && (vif == 0x14 || vif == 0x13)) {
              if (total_volume_) total_volume_->publish_state(decode_bcd_(rx_buffer_, p+2, 4) * 0.01f);
              p += 6;
            }
            else if (dif == 0x0A && vif == 0x5A) {
              if (temp_flow_) temp_flow_->publish_state(decode_bcd_(rx_buffer_, p+2, 2) * 0.1f);
              p += 4;
            }
            else if (dif == 0x0A && vif == 0x5E) {
              if (temp_return_) temp_return_->publish_state(decode_bcd_(rx_buffer_, p+2, 2) * 0.1f);
              p += 4;
            }
            else if (dif == 0x0B && vif == 0x61) {
              if (temp_diff_) temp_diff_->publish_state(decode_bcd_(rx_buffer_, p+2, 3) * 0.01f);
              p += 5;
            }
            else if (dif == 0x04 && vif == 0x6D) {
              std::string ts;
              if (meter_time_ && decode_cp32_datetime_(rx_buffer_, p+2, ts)) meter_time_->publish_state(ts);
              p += 6;
            }
            else { p++; }
          }
          rx_buffer_.clear();
          state = UM_IDLE;
          return;
        }
      }
    }

    if (now - state_ts_ > 5000) {
      ESP_LOGW(TAG, "RX Timeout. Buffer Größe: %d", rx_buffer_.size());
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
