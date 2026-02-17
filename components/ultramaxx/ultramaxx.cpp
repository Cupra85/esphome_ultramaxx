#include "ultramaxx.h"

#include <algorithm>
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// ------------------------------------------------------------
// Helpers: Long-frame extraction (68 L L 68 ... CS 16)
// ------------------------------------------------------------
static bool extract_long_frame(const std::vector<uint8_t> &buf, std::vector<uint8_t> &out_frame) {
  // Search for 0x68
  for (size_t start = 0; start + 6 <= buf.size(); start++) {
    if (buf[start] != 0x68)
      continue;

    if (start + 4 > buf.size())
      return false;

    uint8_t l1 = buf[start + 1];
    uint8_t l2 = buf[start + 2];
    if (l1 != l2)
      continue;

    if (buf[start + 3] != 0x68)
      continue;

    size_t total = static_cast<size_t>(l1) + 6;  // 68 L L 68 + (L bytes C..DATA) + CS + 16
    if (start + total > buf.size())
      return false;  // not enough data yet

    if (buf[start + total - 1] != 0x16)
      continue;

    // Optional checksum validation (recommended)
    // CS is second last byte; sum from C (start+4) to last data (start+total-3)
    uint8_t cs = buf[start + total - 2];
    uint32_t sum = 0;
    for (size_t i = start + 4; i <= start + total - 3; i++) sum += buf[i];
    if ((sum & 0xFF) != cs) {
      // If checksum fails, ignore this candidate and continue searching
      continue;
    }

    out_frame.assign(buf.begin() + start, buf.begin() + start + total);
    return true;
  }
  return false;
}

// ------------------------------------------------------------
// UltraMaXXComponent
// ------------------------------------------------------------
float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0.0f;
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
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= (uint32_t)data[start + i] << (8 * i);
  return v;
}

// CP32 date/time decode (common M-Bus layout):
// minute = b0 & 0x3F
// hour   = b1 & 0x1F
// day    = b2 & 0x1F
// month  = ((b2 & 0xE0) >> 5) | ((b3 & 0x01) << 3)
// year   = (b3 >> 1) + 2000
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;
  uint8_t b0 = data[start + 0];
  uint8_t b1 = data[start + 1];
  uint8_t b2 = data[start + 2];
  uint8_t b3 = data[start + 3];

  int minute = b0 & 0x3F;
  int hour = b1 & 0x1F;
  int day = b2 & 0x1F;
  int month = ((b2 & 0xE0) >> 5) | ((b3 & 0x01) << 3);
  int year = (b3 >> 1) + 2000;

  if (minute < 0 || minute > 59 || hour < 0 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12)
    return false;

  char tmp[32];
  std::snprintf(tmp, sizeof(tmp), "%04d-%02d-%02d %02d:%02d", year, month, day, hour, minute);
  out = tmp;
  return true;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup mode: 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  last_rx_byte_ = millis();
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // --- always read bytes (non-blocking) ---
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;

        // prevent unbounded growth
        if (rx_buffer_.size() > 512) {
          rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 256);
        }
      }
    }
  }

  // --- Wakeup: send 0x55 for ~2.2s at 2400 8N1 ---
  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20];
      std::memset(buf, 0x55, sizeof(buf));
      this->write_array(buf, sizeof(buf));
      last_send_ = now;
    }
    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
    }
    return;
  }

  // --- Wait then switch to 2400 8E1 ---
  if (state == UM_WAIT && (now - state_ts_ > 350)) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // hard flush input so no 0x55/E5 leftovers disturb the next RX state
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    // SND_NKE
    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
    return;
  }

  // --- Send request (REQ_UD2 with FCB toggle) ---
  if (state == UM_SEND && (now - state_ts_ > 120)) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;     // toggle FCB
    uint8_t cs = (uint8_t)((ctrl + 0xFE) & 0xFF); // checksum for short frame
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();
    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    rx_buffer_.clear();
    last_rx_byte_ = now;
    state = UM_RX;
    state_ts_ = now;
    return;
  }

  // --- RX: wait for complete long frame (ignore E5 etc.) ---
  if (state == UM_RX) {
    std::vector<uint8_t> frame;
    if (extract_long_frame(rx_buffer_, frame)) {
      ESP_LOGI(TAG, "Parsing Frame (%d bytes)", (int) frame.size());

      // ---- locate records start (CI=0x72 response): skip fixed header ----
      // 68 L L 68 C A CI  (index 0..6)
      // then: ID(4) MAN(2) VER(1) MED(1) ACC(1) STAT(1) SIG(2)  => 12 bytes
      // records start at index 7+12 = 19
      if (frame.size() < 25) {
        ESP_LOGW(TAG, "Frame too short");
        rx_buffer_.clear();
        state = UM_IDLE;
        return;
      }

      size_t rec = 19;
      const size_t end = frame.size() - 2; // exclude CS + 0x16

      while (rec + 2 <= end) {
        uint8_t dif = frame[rec++];
        // handle DIF extensions (we only need to skip them)
        while ((dif & 0x80) && rec < end) {
          // DIFE present, skip
          uint8_t die = frame[rec++];
          if ((die & 0x80) == 0) break;
        }

        if (rec >= end) break;
        uint8_t vif = frame[rec++];

        // handle VIF extensions (skip)
        if (vif & 0x80) {
          while (rec < end) {
            uint8_t vife = frame[rec++];
            if ((vife & 0x80) == 0) break;
          }
          // we don't interpret VIFE in this minimal parser
          if (rec >= end) break;
        }

        // determine data length from DIF low nibble
        int len = 0;
        switch (dif & 0x0F) {
          case 0x00: len = 0; break;
          case 0x01: len = 1; break;
          case 0x02: len = 2; break;
          case 0x03: len = 3; break;
          case 0x04: len = 4; break;
          case 0x05: len = 4; break; // REAL
          case 0x06: len = 6; break;
          case 0x07: len = 8; break;
          case 0x09: len = 1; break; // 2 digit BCD
          case 0x0A: len = 2; break; // 4 digit BCD
          case 0x0B: len = 3; break; // 6 digit BCD
          case 0x0C: len = 4; break; // 8 digit BCD
          case 0x0E: len = 6; break; // 12 digit BCD
          default:
            // variable/special not handled here
            len = 0;
            break;
        }

        if (rec + len > end) break;

        // ---- interpret known VIFs from your frame ----
        // Serial number: DIF low nibble 0x0C (8-digit BCD / 4 bytes), VIF 0x78
        if (serial_number_ && ((dif & 0x0F) == 0x0C) && vif == 0x78 && len == 4) {
          serial_number_->publish_state(this->decode_bcd_(frame, rec, 4));
        }

        // Total energy: DIF 0x04 (U32 LE), VIF 0x06 => *0.001 = MWh
        if (total_energy_ && ((dif & 0x0F) == 0x04) && vif == 0x06 && len == 4) {
          uint32_t v = this->decode_u_le_(frame, rec, 4);
          total_energy_->publish_state(v * 0.001f);
        }

        // Total volume: DIF 0x0C (BCD8), VIF 0x14 => *0.01 = m³
        if (total_volume_ && ((dif & 0x0F) == 0x0C) && vif == 0x14 && len == 4) {
          total_volume_->publish_state(this->decode_bcd_(frame, rec, 4) * 0.01f);
        }

        // Current power: DIF 0x0B (BCD6), VIF 0x2D => /10 = kW
        if (current_power_ && ((dif & 0x0F) == 0x0B) && vif == 0x2D && len == 3) {
          current_power_->publish_state(this->decode_bcd_(frame, rec, 3) * 0.1f);
        }

        // Flow temp: DIF 0x0A (BCD4), VIF 0x5A => *0.1 °C
        if (temp_flow_ && ((dif & 0x0F) == 0x0A) && vif == 0x5A && len == 2) {
          temp_flow_->publish_state(this->decode_bcd_(frame, rec, 2) * 0.1f);
        }

        // Return temp: DIF 0x0A (BCD4), VIF 0x5E => *0.1 °C
        if (temp_return_ && ((dif & 0x0F) == 0x0A) && vif == 0x5E && len == 2) {
          temp_return_->publish_state(this->decode_bcd_(frame, rec, 2) * 0.1f);
        }

        // Delta T: DIF 0x0B (BCD6), VIF 0x61 => /100 K
        if (temp_diff_ && ((dif & 0x0F) == 0x0B) && vif == 0x61 && len == 3) {
          temp_diff_->publish_state(this->decode_bcd_(frame, rec, 3) * 0.01f);
        }

        // Meter time: DIF 0x04, VIF 0x6D (CP32)
        if (meter_time_ && ((dif & 0x0F) == 0x04) && vif == 0x6D && len == 4) {
          std::string dt;
          if (this->decode_cp32_datetime_(frame, rec, dt)) {
            meter_time_->publish_state(dt);
          }
        }

        rec += len;
      }

      rx_buffer_.clear();
      state = UM_IDLE;
      ESP_LOGI(TAG, "Update finished.");
      return;
    }

    // RX timeout: only if we never managed to extract a valid long frame
    if (now - state_ts_ > 4000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
      return;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
