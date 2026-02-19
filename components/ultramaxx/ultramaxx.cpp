#include "ultramaxx.h"

#include <cstring>
#include <string>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_SEND,
  UM_RX
};

static UMState state = UM_IDLE;

// ------------------------- Helpers -------------------------

static uint8_t mbus_checksum_(const uint8_t *data, size_t len) {
  // checksum = sum(data bytes) mod 256
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return (uint8_t) (sum & 0xFF);
}

static bool extract_mbus_longframe_(const std::vector<uint8_t> &buf,
                                   std::vector<uint8_t> &out_frame) {
  // Long frame: 68 L L 68 C A CI DATA... CS 16
  // Total length = L + 5 bytes
  if (buf.size() < 6) return false;

  // find first 0x68
  size_t start = 0;
  while (start < buf.size() && buf[start] != 0x68) start++;
  if (start >= buf.size()) return false;

  // need at least header
  if (start + 6 > buf.size()) return false;

  uint8_t L1 = buf[start + 1];
  uint8_t L2 = buf[start + 2];
  if (L1 != L2) return false;
  if (buf[start + 3] != 0x68) return false;

  const size_t total_len = (size_t) L1 + 5;  // 68 L L 68 + (L bytes) + 16
  if (start + total_len > buf.size()) return false;

  // stop byte must be 0x16
  if (buf[start + total_len - 1] != 0x16) return false;

  // checksum byte is before 0x16
  const size_t cs_index = start + total_len - 2;

  // checksum is over C..DATA (L bytes), which begin at start+4 and have length L
  const uint8_t cs_calc = mbus_checksum_(&buf[start + 4], (size_t) L1);
  const uint8_t cs_recv = buf[cs_index];
  if (cs_calc != cs_recv) return false;

  out_frame.assign(buf.begin() + start, buf.begin() + start + total_len);
  return true;
}

// ------------------------- Decoders -------------------------

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0;
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul;
    mul *= 10;
    value += ((b >> 4) & 0x0F) * mul;
    mul *= 10;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    v |= (uint32_t) data[start + i] << (8 * i);
  }
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  // CP32 per MBDOC48 (EN13757-3):
  // min  : bits 1..6
  // IV   : bit 8
  // hour : bits 9..13
  // SU   : bit 16
  // day  : bits 17..21
  // year : bits 22..24 and 29..32  (7 bit total)
  // month: bits 25..28
  if (start + 4 > data.size()) return false;

  uint32_t v = decode_u_le(data, start, 4);

  uint32_t minute = (v >> 0) & 0x3F;
  uint32_t iv     = (v >> 7) & 0x01;
  uint32_t hour   = (v >> 8) & 0x1F;
  uint32_t su     = (v >> 15) & 0x01;
  uint32_t day    = (v >> 16) & 0x1F;
  uint32_t year_l = (v >> 21) & 0x07;   // bits 22..24
  uint32_t month  = (v >> 24) & 0x0F;   // bits 25..28
  uint32_t year_h = (v >> 28) & 0x0F;   // bits 29..32

  uint32_t year = (year_h << 3) | year_l;  // 0..99 typical

  if (iv) return false;  // time invalid

  // Basic sanity checks
  if (month < 1 || month > 12) return false;
  if (day < 1 || day > 31) return false;
  if (hour > 23) return false;
  if (minute > 59) return false;

  uint32_t full_year = 2000 + year;

  char buf[32];
  // include SU flag only as suffix if you want; keep clean:
  // "YYYY-MM-DD HH:MM"
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u", full_year, month, day, hour, minute);
  out = std::string(buf);

  (void) su;  // available if you later want to expose summer-time info
  return true;
}

// ------------------------- Component -------------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup phase: 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // Always drain UART; but only buffer in UM_RX
  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c)) break;

    if (state == UM_RX) {
      rx_buffer_.push_back(c);
      last_rx_byte_ = now;
      // keep buffer bounded (avoid runaway if line noise)
      if (rx_buffer_.size() > 512) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 256);
      }
    }
  }

  // --- WAKEUP: send 0x55 for ~2.2s at 2400 8N1 ---
  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20];
      memset(buf, 0x55, sizeof(buf));
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

  // --- WAIT then switch to 2400 8E1 ---
  if (state == UM_WAIT) {
    if (now - state_ts_ > 350) {
      ESP_LOGI(TAG, "Switch to 2400 8E1");

      this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
      this->parent_->load_settings();

      // HARD flush of any leftovers (55/echo/etc.)
      uint8_t d;
      while (this->available()) this->read_byte(&d);

      rx_buffer_.clear();

      // SND_NKE broadcast FE
      uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
      this->write_array(reset, sizeof(reset));
      this->flush();
      ESP_LOGI(TAG, "SND_NKE gesendet");

      state = UM_SEND;
      state_ts_ = now;
    }
    return;
  }

  // --- SEND request (REQ_UD2 alternating FCB) ---
  if (state == UM_SEND) {
    // small delay after reset (meter needs it; Tasmota used ~350ms earlier, we already did that;
    // keep an additional short guard)
    if (now - state_ts_ < 120) return;

    // clear again to avoid parsing E5 or anything before frame
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();

    // REQ_UD2: 0x5B / 0x7B toggle (FCB)
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    state = UM_RX;
    state_ts_ = now;
    last_rx_byte_ = now;
    return;
  }

  // --- RX: collect and parse exactly one valid longframe ---
  if (state == UM_RX) {
    // Try extract a valid longframe from rx_buffer_ at any time
    std::vector<uint8_t> frame;
    if (extract_mbus_longframe_(rx_buffer_, frame)) {
      ESP_LOGI(TAG, "Parsing Frame (%u bytes)", (unsigned) frame.size());

      // frame layout:
      // 0:68 1:L 2:L 3:68 4:C 5:A 6:CI 7..data.. (until CS) (last-2:CS last-1:16)
      const size_t data_start = 7;
      const size_t data_end   = frame.size() - 2;  // exclusive of CS and 16 => CS is at size-2

      // scan for DIF/VIF patterns inside data section
      for (size_t i = data_start; i + 6 <= data_end; i++) {
        uint8_t dif = frame[i];
        uint8_t vif = frame[i + 1];

        // Serial number: 0C 78 + 4B BCD
        if (dif == 0x0C && vif == 0x78) {
          if (serial_number_) {
            float sn = decode_bcd(frame, i + 2, 4);
            serial_number_->publish_state(sn);
          }
          continue;
        }

        // Energy: 04 06 + 4B uint32 LE => *0.001 MWh
        if (dif == 0x04 && vif == 0x06) {
          if (total_energy_) {
            uint32_t raw = decode_u_le(frame, i + 2, 4);
            total_energy_->publish_state((float) raw * 0.001f);
          }
          continue;
        }

        // Volume: 0C 14 + 4B BCD => *0.01 m³
        if (dif == 0x0C && vif == 0x14) {
          if (total_volume_) {
            float v = decode_bcd(frame, i + 2, 4) * 0.01f;
            total_volume_->publish_state(v);
          }
          continue;
        }

        // Flow temp: 0A 5A + 2B BCD => *0.1 °C
        if (dif == 0x0A && vif == 0x5A) {
          if (temp_flow_) {
            float t = decode_bcd(frame, i + 2, 2) * 0.1f;
            temp_flow_->publish_state(t);
          }
          continue;
        }

        // Return temp: 0A 5E + 2B BCD => *0.1 °C
        if (dif == 0x0A && vif == 0x5E) {
          if (temp_return_) {
            float t = decode_bcd(frame, i + 2, 2) * 0.1f;
            temp_return_->publish_state(t);
          }
          continue;
        }

        // Delta T: 0B 61 + 3B uint24 LE => *0.01 K
        if (dif == 0x0B && vif == 0x61) {
          if (temp_diff_) {
            uint32_t raw = decode_u_le(frame, i + 2, 3);
            temp_diff_->publish_state((float) raw * 0.01f);
          }
          continue;
        }

        // Meter time: 04 6D + 4B CP32 => "YYYY-MM-DD HH:MM"
        if (dif == 0x04 && vif == 0x6D) {
          if (meter_time_) {
            std::string ts;
            if (decode_cp32_datetime_(frame, i + 2, ts)) {
              meter_time_->publish_state(ts);
            }
          }
          continue;
        }
      }

      ESP_LOGI(TAG, "Update finished.");

      rx_buffer_.clear();
      state = UM_IDLE;
      return;
    }

    // timeout if no full valid frame arrived
    if (now - state_ts_ > 10000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
