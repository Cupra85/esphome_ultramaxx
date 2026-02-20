#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const VERSION = "UltraMaXX Parser v5.15";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

//
// ------------------ DECODER ------------------
//

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++)
    v |= ((uint32_t)data[start + i]) << (8 * i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  if (start + 4 > data.size()) return false;
  char buf[32];
  sprintf(buf, "%02u.%02u %02u:%02u",
          data[start + 1], data[start],
          data[start + 3], data[start + 2]);
  out = buf;
  return true;
}

//
// ------------------ SETUP ------------------
//

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s", VERSION);
}

//
// ------------------ UPDATE ------------------
//

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START (%s) ===", VERSION);

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;

  state = UM_WAKEUP;
}

//
// ------------------ LOOP ------------------
//

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  //
  // ----------- RX STREAM PARSER -----------
  //
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {

      if (rx_buffer_.empty() && c != 0x68)
        continue;

      rx_buffer_.push_back(c);

      size_t i = rx_buffer_.size();

      if (i >= 6) {

        // SERIAL
        if (rx_buffer_[i-6] == 0x0C && rx_buffer_[i-5] == 0x78) {
          float sn = decode_bcd(rx_buffer_, i-4, 4);
          ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
          if (serial_number_) serial_number_->publish_state(sn);
        }

        // ENERGY
        if (rx_buffer_[i-6] == 0x04 && rx_buffer_[i-5] == 0x06) {
          float e = decode_u_le(rx_buffer_, i-4, 4) * 0.001f;
          ESP_LOGI(TAG, "ENERGY parsed: %.3f", e);
          if (total_energy_) total_energy_->publish_state(e);
        }

        // VOLUME
        if (rx_buffer_[i-6] == 0x0C && rx_buffer_[i-5] == 0x14) {
          float v = decode_bcd(rx_buffer_, i-4, 4) * 0.01f;
          ESP_LOGI(TAG, "VOLUME parsed: %.2f", v);
          if (total_volume_) total_volume_->publish_state(v);
        }

        // FLOW TEMP
        if (rx_buffer_[i-4] == 0x0A && rx_buffer_[i-3] == 0x5A) {
          float t = decode_bcd(rx_buffer_, i-2, 2) * 0.1f;
          ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f", t);
          if (temp_flow_) temp_flow_->publish_state(t);
        }

        // RETURN TEMP
        if (rx_buffer_[i-4] == 0x0A && rx_buffer_[i-3] == 0x5E) {
          float t = decode_bcd(rx_buffer_, i-2, 2) * 0.1f;
          ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f", t);
          if (temp_return_) temp_return_->publish_state(t);
        }

        // DELTA T
        if (rx_buffer_[i-5] == 0x0B && rx_buffer_[i-4] == 0x61) {
          float dt = decode_bcd(rx_buffer_, i-3, 3) * 0.01f;
          ESP_LOGI(TAG, "DELTA T parsed: %.2f", dt);
          if (temp_diff_) temp_diff_->publish_state(dt);
        }

        // TIME
        if (rx_buffer_[i-6] == 0x04 && rx_buffer_[i-5] == 0x6D) {
          std::string ts;
          if (decode_cp32_datetime_(rx_buffer_, i-4, ts)) {
            ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
            if (meter_time_) meter_time_->publish_state(ts);
          }
        }
      }

      // Frame End
      if (c == 0x16 && rx_buffer_.size() > 40) {
        ESP_LOGI(TAG, "Frame end detected (%d bytes)", rx_buffer_.size());
        rx_buffer_.clear();
      }
    }
  }

  //
  // ----------- STATE MACHINE -----------
  //

  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20];
      memset(buf, 0x55, 20);
      this->write_array(buf, 20);
      last_send_ = now;
    }
    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
    }
  }

  if (state == UM_WAIT && now - state_ts_ > 350) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();

    uint8_t reset[] = {0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {

    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl + 0xFE) & 0xFF;

    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req,sizeof(req));
    this->flush();

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;
    state = UM_RX;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
