#include "ultramaxx.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState : uint8_t {
  UM_IDLE = 0,
  UM_WAKEUP,
  UM_WAIT,
  UM_SEND,
  UM_RX
};

static inline uint32_t le_u32(const std::vector<uint8_t> &d, size_t i) {
  return (uint32_t)d[i] | ((uint32_t)d[i + 1] << 8) | ((uint32_t)d[i + 2] << 16) | ((uint32_t)d[i + 3] << 24);
}

static inline uint16_t le_u16(const std::vector<uint8_t> &d, size_t i) {
  return (uint16_t)d[i] | ((uint16_t)d[i + 1] << 8);
}

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return value;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
  this->state_ = UM_IDLE;
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  this->rx_buffer_.clear();
  this->wake_start_ = millis();
  this->last_send_ = 0;
  this->state_ = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // 1) Wakeup: ~2.2s 0x55
  if (this->state_ == UM_WAKEUP) {
    if (now - this->last_send_ > 15) {
      uint8_t b = 0x55;
      this->write_array(&b, 1);
      this->last_send_ = now;
    }
    if (now - this->wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      this->state_ = UM_WAIT;
      this->state_ts_ = now;
    }
    return;
  }

  // 2) Switch to 2400 8E1 + SND_NKE
  if (this->state_ == UM_WAIT && now - this->state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // clear RX
    uint8_t dummy;
    while (this->available()) this->read_byte(&dummy);

    // SND_NKE (broadcast FE)
    const uint8_t snd_nke[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(snd_nke, sizeof(snd_nke));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    this->state_ = UM_SEND;
    this->state_ts_ = now;
    return;
  }

  // 3) REQ_UD2 (full user data)
  if (this->state_ == UM_SEND && now - this->state_ts_ > 100) {
    const uint8_t req_ud2[] = {0x10, 0x7B, 0xFE, 0x79, 0x16};
    this->write_array(req_ud2, sizeof(req_ud2));
    this->flush();
    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    this->state_ = UM_RX;
    this->state_ts_ = now;
    this->last_rx_byte_ = now;
    return;
  }

  // 4) RX sammeln + nach 1s Stille parsen
  if (this->state_ == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (this->read_byte(&c)) {
        this->rx_buffer_.push_back(c);
        this->last_rx_byte_ = millis();
      }
    }

    // Frame-Ende: 1s keine neuen Bytes und Mindestlänge
    if (this->rx_buffer_.size() > 20 && (millis() - this->last_rx_byte_) > 1000) {
      auto &f = this->rx_buffer_;

      ESP_LOGD(TAG, "RX bytes: %u", (unsigned) f.size());

      // Scan basierend auf deinem echten Dump:
      // 0C 78 (SN BCD)
      // 04 06 (Energy u32 LE)
      // 0C 14 (Volume u32 LE)
      // 3B 2D (Power u24 LE)
      // 0A 5A (FlowTemp u16 LE, scale TBD)
      // 0A 5E (ReturnTemp u16 LE, scale TBD)
      // 0B 61 (DeltaT u24 LE)
      // 04 6D (Date/Time payload, publish as hex for now)
      for (size_t i = 0; i + 6 < f.size(); i++) {

        if (this->serial_number_ && f[i] == 0x0C && f[i + 1] == 0x78) {
          this->serial_number_->publish_state(this->decode_bcd(f, i + 2, 4));
        }

        if (this->total_energy_ && f[i] == 0x04 && f[i + 1] == 0x06) {
          // Achtung: Skalierung ist geräteabhängig. Wir publishen erstmal roh/1000 als kWh-Näherung.
          const float raw = (float) le_u32(f, i + 2);
          this->total_energy_->publish_state(raw / 1000.0f);
        }

        if (this->total_volume_ && f[i] == 0x0C && f[i + 1] == 0x14) {
          const float raw = (float) le_u32(f, i + 2);
          this->total_volume_->publish_state(raw / 1000.0f);
        }

        if (this->current_power_ && f[i] == 0x3B && f[i + 1] == 0x2D) {
          uint32_t p = (uint32_t) f[i + 2] | ((uint32_t) f[i + 3] << 8) | ((uint32_t) f[i + 4] << 16);
          if (p != 0x999999) this->current_power_->publish_state((float) p);
        }

        if (this->temp_flow_ && f[i] == 0x0A && f[i + 1] == 0x5A) {
          // Dein Dump: 0A 5A 47 02 -> 0x0247 = 583
          // Oft: 0.1°C oder 0.01°C. Wir nehmen 0.1°C als übliche Allmess-Skalierung.
          const float t = (float) le_u16(f, i + 2);
          this->temp_flow_->publish_state(t / 10.0f);
        }

        if (this->temp_return_ && f[i] == 0x0A && f[i + 1] == 0x5E) {
          const float t = (float) le_u16(f, i + 2);
          this->temp_return_->publish_state(t / 10.0f);
        }

        if (this->temp_diff_ && f[i] == 0x0B && f[i + 1] == 0x61) {
          uint32_t td = (uint32_t) f[i + 2] | ((uint32_t) f[i + 3] << 8) | ((uint32_t) f[i + 4] << 16);
          this->temp_diff_->publish_state((float) td / 100.0f);
        }

        if (this->meter_time_ && f[i] == 0x04 && f[i + 1] == 0x6D) {
          // publish 4 bytes as hex (device-specific time encoding)
          char buf[16];
          snprintf(buf, sizeof(buf), "%02X%02X%02X%02X", f[i + 2], f[i + 3], f[i + 4], f[i + 5]);
          this->meter_time_->publish_state(buf);
        }
      }

      this->rx_buffer_.clear();
      this->state_ = UM_IDLE;
      return;
    }

    // Hard timeout RX
    if (now - this->state_ts_ > 12000) {
      ESP_LOGW(TAG, "RX Timeout");
      this->rx_buffer_.clear();
      this->state_ = UM_IDLE;
      return;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
