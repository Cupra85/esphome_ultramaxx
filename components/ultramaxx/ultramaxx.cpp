#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &d, size_t s, size_t l) {
  if (s + l > d.size()) return 0;
  float v = 0;
  float m = 1;
  for (size_t i = 0; i < l; i++) {
    uint8_t b = d[s + i];
    v += (b & 0x0F) * m; m *= 10;
    v += ((b >> 4) & 0x0F) * m; m *= 10;
  }
  return v;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &d, size_t s, size_t l) {
  uint32_t v = 0;
  for (size_t i = 0; i < l; i++) v |= ((uint32_t)d[s+i]) << (8*i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &d, size_t s, std::string &out) {
  if (s + 4 > d.size()) return false;
  char buf[32];
  sprintf(buf,"%02u.%02u  %02u:%02u", d[s], d[s+1], d[s+2], d[s+3]);
  out = buf;
  return true;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG,"UltraMaXX started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG,"=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  wake_start_ = millis();
  last_send_ = 0;
}

void UltraMaXXComponent::loop() {

  // ========= STREAM PARSER =========

  while (this->available()) {

    uint8_t c;
    if (!this->read_byte(&c)) return;

    rx_buffer_.push_back(c);

    size_t n = rx_buffer_.size();
    if (n < 8) continue;

    auto &f = rx_buffer_;

    // -------- Seriennummer 0C78 --------
    if (f[n-6]==0x0C && f[n-5]==0x78 && serial_number_) {
      float sn = decode_bcd(f,n-4,4);
      serial_number_->publish_state(sn);
      ESP_LOGI(TAG,"Serial parsed: %.0f",sn);
    }

    // -------- Energie 0406 --------
    if (f[n-6]==0x04 && f[n-5]==0x06 && total_energy_) {
      uint32_t v = decode_u_le(f,n-4,4);
      total_energy_->publish_state(v*0.001f);
      ESP_LOGI(TAG,"Energy parsed");
    }

    // -------- Volumen 0C14 --------
    if (f[n-6]==0x0C && f[n-5]==0x14 && total_volume_) {
      total_volume_->publish_state(decode_bcd(f,n-4,4)*0.01f);
      ESP_LOGI(TAG,"Volume parsed");
    }

    // -------- Vorlauf 0A5A --------
    if (f[n-4]==0x0A && f[n-3]==0x5A && temp_flow_) {
      temp_flow_->publish_state(decode_bcd(f,n-2,2)*0.1f);
      ESP_LOGI(TAG,"Flow temp parsed");
    }

    // -------- Rücklauf 0A5E --------
    if (f[n-4]==0x0A && f[n-3]==0x5E && temp_return_) {
      temp_return_->publish_state(decode_bcd(f,n-2,2)*0.1f);
      ESP_LOGI(TAG,"Return temp parsed");
    }

    // -------- Delta T 0B61 --------
    if (f[n-5]==0x0B && f[n-4]==0x61 && temp_diff_) {
      temp_diff_->publish_state(decode_bcd(f,n-3,3)*0.01f);
      ESP_LOGI(TAG,"Delta T parsed");
    }

    // -------- Meter Time 046D --------
    if (f[n-6]==0x04 && f[n-5]==0x6D && meter_time_) {
      std::string t;
      if (decode_cp32_datetime_(f,n-4,t))
        meter_time_->publish_state(t);
      ESP_LOGI(TAG,"Meter time parsed");
    }

    // Buffer begrenzen
    if (rx_buffer_.size() > 300)
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin()+150);
  }
}

}  // namespace ultramaxx
}  // namespace esphome
