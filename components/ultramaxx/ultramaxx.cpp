#include "ultramaxx.h"

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size())
    return 0.0f;

  float value = 0.0f;
  float mul = 1.0f;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul;
    mul *= 10.0f;
    value += ((b >> 4) & 0x0F) * mul;
    mul *= 10.0f;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size())
    return 0;

  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    v |= (uint32_t) data[start + i] << (8 * i);
  }
  return v;
}

// CP32 date/time (common M-Bus layout):
// minute = b0 & 0x3F
// hour   = b1 & 0x1F
// day    = b2 & 0x1F
// month  = ((b2 & 0xE0) >> 5) | ((b3 & 0x01) << 3)
// year   = (b3 >> 1) + 2000
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size())
    return false;

  const uint8_t b0 = data[start + 0];
  const uint8_t b1 = data[start + 1];
  const uint8_t b2 = data[start + 2];
  const uint8_t b3 = data[start + 3];

  const int minute = b0 & 0x3F;
  const int hour = b1 & 0x1F;
  const int day = b2 & 0x1F;
  const int month = ((b2 & 0xE0) >> 5) | ((b3 & 0x01) << 3);
  const int year = (b3 >> 1) + 2000;

  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12)
    return false;

  char tmp[32];
  std::snprintf(tmp, sizeof(tmp), "%04d-%02d-%02d %02d:%02d", year, month, day, hour, minute);
  out = tmp;
  return true;
}

// Extract long frame: 68 L L 68 ... CS 16
bool UltraMaXXComponent::extract_long_frame_(const std::vector<uint8_t> &buf, std::vector<uint8_t> &out_frame) const {
  for (size_t start = 0; start + 6 <= buf.size(); start++) {
    if (buf[start] != 0x68)
      continue;

    if (start + 4 > buf.size())
      return false;

    const uint8_t l1 = buf[start + 1];
    const uint8_t l2 = buf[start + 2];
    if (l1 != l2)
      continue;

    if (buf[start + 3] != 0x68)
      continue;

    const size_t total = (size_t) l1 + 6;  // 68 L L 68 + (L bytes) + CS + 16
    if (start + total > buf.size())
      return false;

    if (buf[start + total - 1] != 0x16)
      continue;

    const uint8_t cs = buf[start + total - 2];
    uint32_t sum = 0;
    for (size_t i = start + 4; i <= start + total - 3; i++)
      sum += buf[i];

    if (((sum)&0xFF) != cs)
      continue;

    out_frame.assign(buf.begin() + start, buf.begin() + start + total);
    return true;
  }
  return false;
}

// ------------------------------------------------------------
// Component
// ------------------------------------------------------------
void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup: 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  last_rx_byte_ = millis();

  state_ = UM_WAKEUP;
  state_ts_ = millis();
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // Always read bytes non-blocking
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state_ == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;

        if (rx_buffer_.size() > 512) {
          rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 256);
        }
      }
    }
  }

  // WAKEUP: send 0x55 pattern for ~2.2s at 2400 8N1
  if (state_ == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20];
      std::memset(buf, 0x55, sizeof(buf));
      this->write_array(buf, sizeof(buf));
      last_send_ = now;
    }

    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state_ = UM_WAIT;
      state_ts_ = now;
    }
    return;
  }

  // WAIT then switch to 2400 8E1
  if (state_ == UM_WAIT && (now - state_ts_ > 350)) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // flush input (remove wakeup garbage)
    uint8_t d;
    while (this->available())
      this->read_byte(&d);
    rx_buffer_.clear();

    // SND_NKE
    const uint8_t snd_nke[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(snd_nke, sizeof(snd_nke));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE sent");
    state_ = UM_SEND;
    state_ts_ = now;
    return;
  }

  // SEND request
  if (state_ == UM_SEND && (now - state_ts_ > 120)) {
    const uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;          // toggle FCB
    const uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);     // short frame checksum
    const uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 sent");
    fcb_toggle_ = !fcb_toggle_;

    rx_buffer_.clear();
    state_ = UM_RX;
    state_ts_ = now;
    last_rx_byte_ = now;
    return;
  }

  // RX / PARSE long frame
  if (state_ == UM_RX) {
    // wait until no more bytes for a moment
    if (rx_buffer_.size() > 10 && (now - last_rx_byte_) > 200) {
      std::vector<uint8_t> frame;
      if (!extract_long_frame_(rx_buffer_, frame)) {
        // keep collecting, but guard timeout below
      } else {
        ESP_LOGI(TAG, "Parsing Frame (%d bytes)", (int) frame.size());

        // Minimal fixed header skip:
        // 68 L L 68 C A CI  => 7 bytes (0..6)
        // then: ID(4) MAN(2) VER(1) MED(1) ACC(1) STAT(1) SIG(2) => 12 bytes
        // records start at 7+12=19
        if (frame.size() < 25) {
          ESP_LOGW(TAG, "Frame too short");
          rx_buffer_.clear();
          state_ = UM_IDLE;
          return;
        }

        size_t rec = 19;
        const size_t end = frame.size() - 2;  // exclude CS + 0x16

        while (rec + 2 <= end) {
          uint8_t dif = frame[rec++];

          // skip DIFE(s)
          if (dif & 0x80) {
            while (rec < end) {
              uint8_t die = frame[rec++];
              if ((die & 0x80) == 0)
                break;
            }
          }
          if (rec >= end)
            break;

          uint8_t vif = frame[rec++];

          // skip VIFE(s)
          if (vif & 0x80) {
            while (rec < end) {
              uint8_t vife = frame[rec++];
              if ((vife & 0x80) == 0)
                break;
            }
            if (rec >= end)
              break;
          }

          // data length from DIF low nibble
          int len = 0;
          switch (dif & 0x0F) {
            case 0x00: len = 0; break;
            case 0x01: len = 1; break;
            case 0x02: len = 2; break;
            case 0x03: len = 3; break;
            case 0x04: len = 4; break;
            case 0x05: len = 4; break;  // REAL
            case 0x06: len = 6; break;
            case 0x07: len = 8; break;
            case 0x09: len = 1; break;  // BCD2
            case 0x0A: len = 2; break;  // BCD4
            case 0x0B: len = 3; break;  // BCD6
            case 0x0C: len = 4; break;  // BCD8
            case 0x0E: len = 6; break;  // BCD12
            default: len = 0; break;
          }

          if (rec + len > end)
            break;

          // --- Interpret typical VIFs from README ---
          // Serial: VIF 0x78 (BCD8 / 4 bytes)
          if (serial_number_ && ((dif & 0x0F) == 0x0C) && vif == 0x78 && len == 4) {
            serial_number_->publish_state(decode_bcd_(frame, rec, 4));
          }

          // Energy: VIF 0x06 (U32 LE) -> kWh scaling depends on meter; common: 0.001 MWh -> kWh = *1.0
          if (total_energy_ && ((dif & 0x0F) == 0x04) && vif == 0x06 && len == 4) {
            const uint32_t v = decode_u_le_(frame, rec, 4);
            // Keep your previous scaling (0.001) if you used MWh:
            total_energy_->publish_state(v * 0.001f);
          }

          // Volume: README says 0x13; some frames show 0x14. Accept both (BCD8 / 4 bytes) -> *0.01 m³
          if (total_volume_ && ((dif & 0x0F) == 0x0C) && (vif == 0x13 || vif == 0x14) && len == 4) {
            total_volume_->publish_state(decode_bcd_(frame, rec, 4) * 0.01f);
          }

          // Flow temp: commonly 0x5A (BCD4/2 bytes) -> *0.1 °C
          if (temp_flow_ && ((dif & 0x0F) == 0x0A) && vif == 0x5A && len == 2) {
            temp_flow_->publish_state(decode_bcd_(frame, rec, 2) * 0.1f);
          }

          // Return temp: commonly 0x5E (BCD4/2 bytes) -> *0.1 °C
          if (temp_return_ && ((dif & 0x0F) == 0x0A) && vif == 0x5E && len == 2) {
            temp_return_->publish_state(decode_bcd_(frame, rec, 2) * 0.1f);
          }

          // Power: README says 0x2B; some variants show 0x2D. Common: BCD6/3 bytes -> *0.1 (kW) or W scaling
          if (current_power_ && ((dif & 0x0F) == 0x0B) && (vif == 0x2B || vif == 0x2D) && len == 3) {
            current_power_->publish_state(decode_bcd_(frame, rec, 3) * 0.1f);
          }

          // Flow: README says 0x3B; you may need scaling depending on unit; here: U32 LE * 0.001 as a safe default
          if (flow_rate_ && ((dif & 0x0F) == 0x04) && vif == 0x3B && len == 4) {
            const uint32_t v = decode_u_le_(frame, rec, 4);
            flow_rate_->publish_state(v * 0.001f);
          }

          // Meter time: VIF 0x6D CP32 (4 bytes)
          if (meter_time_ && ((dif & 0x0F) == 0x04) && vif == 0x6D && len == 4) {
            std::string dt;
            if (decode_cp32_datetime_(frame, rec, dt)) {
              meter_time_->publish_state(dt);
            }
          }

          rec += len;
        }

        rx_buffer_.clear();
        state_ = UM_IDLE;
        ESP_LOGI(TAG, "Update finished.");
        return;
      }
    }

    // RX timeout
    if (now - state_ts_ > 4000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state_ = UM_IDLE;
      return;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
