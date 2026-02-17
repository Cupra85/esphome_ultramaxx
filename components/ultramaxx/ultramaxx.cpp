#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// ---------- Decoder ----------

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
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

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++)
    v |= (uint32_t)data[start + i] << (8 * i);
  return v;
}

// ---------- Setup ----------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

// ---------- Update ----------

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

// ---------- Loop ----------

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // RX sammeln
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // Wakeup
  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20]; memset(buf, 0x55, 20);
      this->write_array(buf, 20);
      last_send_ = now;
    }
    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
    }
  }

  // Switch UART
  if (state == UM_WAIT && now - state_ts_ > 350) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while (this->available()) this->read_byte(&d);

    uint8_t reset[] = {0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  // REQ_UD2
  if (state == UM_SEND && now - state_ts_ > 150) {

    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl + 0xFE) & 0xFF;
    uint8_t req[] = {0x10,ctrl,0xFE,cs,0x16};

    this->write_array(req,sizeof(req));
    this->flush();

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    rx_buffer_.clear();
    state = UM_RX;
    state_ts_ = now;
    last_rx_byte_ = now;
  }

  // ----------- NEUER ROBUSTER PARSER -----------

  if (state == UM_RX) {

    if (!rx_buffer_.empty() && (now - last_rx_byte_ > 300)) {

      auto &f = rx_buffer_;

      ESP_LOGI(TAG,"Parsing Frame (%d bytes)", f.size());

      size_t ptr = 7; // Header überspringen

      while (ptr + 2 < f.size()) {

        uint8_t DIF = f[ptr++];

        // DIF Extensions überspringen
        while (DIF & 0x80) DIF = f[ptr++];

        uint8_t VIF = f[ptr++];

        // VIF Extensions überspringen
        while (VIF == 0xFD || VIF == 0xFB) {
          VIF = f[ptr++];
        }

        int len = 0;
        switch (DIF & 0x0F) {
          case 0x01: len = 1; break;
          case 0x02: len = 2; break;
          case 0x03: len = 3; break;
          case 0x04: len = 4; break;
          default: break;
        }

        if (ptr + len > f.size()) break;

        // ---------- MATCHING ----------

        if (serial_number_ && VIF == 0x78 && len == 4)
          serial_number_->publish_state(decode_bcd(f,ptr,4));

        else if (total_energy_ && VIF == 0x06 && len == 4)
          total_energy_->publish_state(decode_u_le(f,ptr,4) * 0.001f);

        else if (total_volume_ && VIF == 0x14 && len == 4)
          total_volume_->publish_state(decode_bcd(f,ptr,4) * 0.01f);

        else if (temp_flow_ && VIF == 0x5A && len == 2)
          temp_flow_->publish_state(decode_bcd(f,ptr,2) * 0.1f);

        else if (temp_return_ && VIF == 0x5E && len == 2)
          temp_return_->publish_state(decode_bcd(f,ptr,2) * 0.1f);

        else if (temp_diff_ && VIF == 0x61 && len == 3)
          temp_diff_->publish_state(decode_bcd(f,ptr,3) * 0.01f);

        ptr += len;
      }

      rx_buffer_.clear();
      state = UM_IDLE;

      ESP_LOGI(TAG,"Update finished.");
    }

    if (now - state_ts_ > 10000) {
      ESP_LOGW(TAG,"RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
