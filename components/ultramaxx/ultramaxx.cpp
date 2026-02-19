#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_SEND,   // send SND_NKE + REQ_UD2
  UM_RX
};

static UMState state = UM_IDLE;

// -------------------- helpers --------------------

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0.0f;
  float value = 0.0f;
  float mul = 1.0f;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10.0f;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10.0f;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    v |= ((uint32_t) data[start + i]) << (8 * i);
  }
  return v;
}

// M-Bus F format (date+time) per: byte1 minutes (n5..n0), byte2 hours (h4..h0),
// byte3/byte4 are date in G format (day/month/year)
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  if (start + 4 > data.size()) return false;

  uint8_t b1 = data[start + 0];
  uint8_t b2 = data[start + 1];
  uint8_t b3 = data[start + 2];
  uint8_t b4 = data[start + 3];

  int minute = b1 & 0x3F;        // 0 0 n5..n0
  int hour   = b2 & 0x1F;        // 0 0 0 h4..h0

  int day   = b3 & 0x1F;         // j4..j0
  int month = b4 & 0x0F;         // M3..M0

  int year_low3 = (b3 >> 5) & 0x07;      // a2..a0
  int year_high4 = (b4 >> 4) & 0x0F;     // a6..a3
  int year = (year_high4 << 3) | year_low3; // 0..99

  // Basic sanity
  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) return false;

  int full_year = (year < 70) ? (2000 + year) : (1900 + year);

  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d", full_year, month, day, hour, minute);
  out = buf;
  return true;
}

// Extract one valid M-Bus long frame from rx_buffer_.
// Frame: 68 L L 68 [L bytes from C..(before CS)] CS 16  => total = L + 6
static bool extract_mbus_frame(std::vector<uint8_t> &buf, std::vector<uint8_t> &frame_out) {
  // drop leading garbage until 0x68
  while (!buf.empty() && buf[0] != 0x68) buf.erase(buf.begin());
  if (buf.size() < 6) return false;

  // need 68 L L 68
  if (buf[0] != 0x68 || buf[3] != 0x68) {
    // resync: drop first byte
    buf.erase(buf.begin());
    return false;
  }

  uint8_t L1 = buf[1];
  uint8_t L2 = buf[2];
  if (L1 != L2) {
    buf.erase(buf.begin());
    return false;
  }

  size_t total_len = (size_t) L1 + 6;
  if (buf.size() < total_len) return false;
  if (buf[total_len - 1] != 0x16) {
    // not a proper end marker; resync
    buf.erase(buf.begin());
    return false;
  }

  // verify checksum (optional but helps resync)
  // checksum is sum of L bytes starting at C-field (index 4) mod 256
  uint8_t cs = buf[4 + L1];
  uint8_t sum = 0;
  for (size_t i = 0; i < L1; i++) sum = (uint8_t) (sum + buf[4 + i]);
  if (sum != cs) {
    // bad frame, resync
    buf.erase(buf.begin());
    return false;
  }

  frame_out.assign(buf.begin(), buf.begin() + total_len);
  buf.erase(buf.begin(), buf.begin() + total_len);
  return true;
}

// -------------------- component --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup uses 2400 8N1
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

  // ---------- STATE MACHINE FIRST (so we don't drop fast replies) ----------

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
  }

  if (state == UM_WAIT && (now - state_ts_ > 350)) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // HARD FLUSH: remove leftover 0x55 etc.
    uint8_t dummy;
    while (this->available()) this->read_byte(&dummy);
    rx_buffer_.clear();

    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND) {
    // Send SND_NKE immediately once
    // (guard with time: only do once at entry)
    if (now - state_ts_ < 5) {
      uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
      this->write_array(reset, sizeof(reset));
      this->flush();
      ESP_LOGI(TAG, "SND_NKE gesendet");
    }

    // Send REQ_UD2 after a short delay (as in known-good scripts)
    if (now - state_ts_ > 120) {
      // flush again right before request
      uint8_t dummy;
      while (this->available()) this->read_byte(&dummy);
      rx_buffer_.clear();

      uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;     // toggle FCB
      uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
      uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

      this->write_array(req, sizeof(req));
      this->flush();

      ESP_LOGI(TAG, "REQ_UD2 gesendet");
      fcb_toggle_ = !fcb_toggle_;

      // NOW switch to RX immediately
      state = UM_RX;
      state_ts_ = now;
      last_rx_byte_ = now;
    }
  }

  // ---------- THEN READ UART (ONLY IN RX) ----------
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (this->read_byte(&c)) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }

    // Try to extract & parse frames whenever buffer grows
    std::vector<uint8_t> frame;
    bool parsed_any = false;

    // Extract potentially multiple frames if present
    while (extract_mbus_frame(rx_buffer_, frame)) {
      parsed_any = true;
      ESP_LOGI(TAG, "Parsing Frame (%u bytes)", (unsigned) frame.size());

      // Frame layout:
      // 0:68 1:L 2:L 3:68 4:C 5:A 6:CI 7.. : body
      // For RSP_UD, CI usually 0x72. Records start after fixed header fields.
      // Your frames show records starting at index 19.
      if (frame.size() < 24) continue;

      size_t ptr = 19;  // verified from your telegrams: after access/status/sig1/sig2

      // walk records
      while (ptr + 2 < frame.size()) {
        uint8_t dif = frame[ptr++];
        if (dif == 0x16) break; // just in case

        // skip DIFE(s)
        while (dif & 0x80) {
          if (ptr >= frame.size()) break;
          dif = frame[ptr++];  // last read was actually DIFE; keep looping if it has extension too
        }

        if (ptr >= frame.size()) break;
        uint8_t vif = frame[ptr++];

        // skip VIFE(s)
        while (vif & 0x80) {
          if (ptr >= frame.size()) break;
          vif = frame[ptr++];  // last read was actually VIFE; keep looping if extension too
        }

        uint8_t lsn = dif & 0x0F;
        int len = 0;

        // data length per DIF low nibble (subset we need)
        if (lsn <= 0x04) {
          // 0: none, 1:1, 2:2, 3:3, 4:4
          len = lsn;
        } else if (lsn >= 0x09 && lsn <= 0x0C) {
          // BCD 2/4/6/8 digits => 1/2/3/4 bytes
          len = (lsn - 0x08);
        } else {
          // other types not needed for our target fields
          // try to stop safely if unknown
          // (prevents walking off if we can't determine length)
          break;
        }

        if (ptr + (size_t) len > frame.size()) break;

        // --- publish fields by VIF (and expected DIF) ---
        // Serial number: DIF=0x0C (BCD8 => 4 bytes), VIF=0x78
        if (serial_number_ && vif == 0x78 && len == 4) {
          float sn = decode_bcd(frame, ptr, 4);
          serial_number_->publish_state(sn);
        }

        // Total energy: DIF=0x04 (4 bytes), VIF=0x06 (uint32 LE, /1000 => MWh)
        if (total_energy_ && vif == 0x06 && len == 4) {
          uint32_t raw = decode_u_le(frame, ptr, 4);
          total_energy_->publish_state(((float) raw) / 1000.0f);
        }

        // Total volume: DIF=0x0C (BCD8 => 4 bytes), VIF=0x14 (BCD, /100 => m³)
        if (total_volume_ && vif == 0x14 && len == 4) {
          float vol = decode_bcd(frame, ptr, 4) / 100.0f;
          total_volume_->publish_state(vol);
        }

        // Current power (in your telegrams appears as DIF=0x3B (BCD6 =>3 bytes), VIF=0x2D, /10 => kW)
        if (current_power_ && vif == 0x2D && len == 3) {
          float p = decode_bcd(frame, ptr, 3) / 10.0f;
          current_power_->publish_state(p);
        }

        // Flow temp: DIF=0x0A (BCD4 =>2 bytes), VIF=0x5A, /10 => °C
        if (temp_flow_ && vif == 0x5A && len == 2) {
          float t = decode_bcd(frame, ptr, 2) / 10.0f;
          temp_flow_->publish_state(t);
        }

        // Return temp: DIF=0x0A (BCD4 =>2 bytes), VIF=0x5E, /10 => °C
        if (temp_return_ && vif == 0x5E && len == 2) {
          float t = decode_bcd(frame, ptr, 2) / 10.0f;
          temp_return_->publish_state(t);
        }

        // Temp diff: DIF=0x0B (BCD6 =>3 bytes), VIF=0x61, /100 => K
        if (temp_diff_ && vif == 0x61 && len == 3) {
          float dt = decode_bcd(frame, ptr, 3) / 100.0f;
          temp_diff_->publish_state(dt);
        }

        // Meter time: DIF=0x04 (4 bytes), VIF=0x6D (F format)
        if (meter_time_ && vif == 0x6D && len == 4) {
          std::string ts;
          if (decode_cp32_datetime_(frame, ptr, ts)) {
            meter_time_->publish_state(ts);
          }
        }

        ptr += (size_t) len;
      }
    }

    if (parsed_any) {
      ESP_LOGI(TAG, "Update finished.");
      state = UM_IDLE;
      rx_buffer_.clear();
      return;
    }

    // RX timeout (no valid frame extracted)
    if (now - state_ts_ > 10000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
