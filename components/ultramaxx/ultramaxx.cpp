#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

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
  if (start + len > data.size() || len == 0 || len > 4) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= (uint32_t)data[start + i] << (8 * i);
  return v;
}

// CP32 (Type F) nach EN13757-3: min(1..6), hour(9..13), day(17..21), month(25..28),
// year(UI7: 22..24 + 29..32), hundred year(14..15), invalid bit(8), summer bit(16)
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  if (start + 4 > data.size()) return false;
  uint32_t v = decode_u_le(data, start, 4);

  uint32_t minute = (v >> 0) & 0x3F;
  bool invalid = ((v >> 7) & 0x01) != 0;
  uint32_t hour = (v >> 8) & 0x1F;
  uint32_t hundred = (v >> 13) & 0x03;
  bool summer = ((v >> 15) & 0x01) != 0;
  uint32_t day = (v >> 16) & 0x1F;
  uint32_t year_low = (v >> 21) & 0x07;
  uint32_t month = (v >> 24) & 0x0F;
  uint32_t year_high = (v >> 28) & 0x0F;
  uint32_t year = (year_high << 3) | year_low;

  int full_year = 1900 + (int)hundred * 100 + (int)year;
  // Kompatibilitäts-Empfehlung (00..80 => 2000..2080)
  if (hundred == 0 && year <= 80) full_year = 2000 + (int)year;

  char buf[64];
  snprintf(buf, sizeof(buf), "%04d-%02u-%02u %02u:%02u%s%s",
           full_year, (unsigned)month, (unsigned)day,
           (unsigned)hour, (unsigned)minute,
           summer ? " DST" : "",
           invalid ? " INVALID" : "");
  out = buf;
  return true;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup-Phase (8N1)
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  uint32_t now = millis();

  // RX immer einsammeln
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // ---- WAKEUP (2.2s 0x55) ----
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

  // ---- Switch auf 8E1, Reset ----
  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // Echo/Altbytes raus
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();

    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
    return;
  }

  // ---- REQ/SND_UD (FCB toggeln) ----
  if (state == UM_SEND && now - state_ts_ > 150) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;  // REQ_UD2 / SND_UD2 mit toggled FCB
    uint8_t cs = (uint8_t)((ctrl + 0xFE) & 0xFF);
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();
    fcb_toggle_ = !fcb_toggle_;

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    rx_buffer_.clear();
    state = UM_RX;
    state_ts_ = now;
    last_rx_byte_ = now;
    return;
  }

  // ---- RX / Parsing ----
  if (state == UM_RX) {
    // Frame completion:
    // Long frame: 68 L L 68 ... (L bytes from C..CS) ... 16
    bool complete = false;
    if (rx_buffer_.size() >= 6 && rx_buffer_[0] == 0x68 && rx_buffer_[3] == 0x68) {
      uint8_t L = rx_buffer_[1];
      size_t total = 4 + (size_t)L + 1;  // 4 header + L bytes + stop(16)
      if (rx_buffer_.size() >= total && rx_buffer_[total - 1] == 0x16) {
        complete = true;
        rx_buffer_.resize(total);
      }
    }

    // Fallback: Stille nach Daten
    if (!complete && !rx_buffer_.empty() && (now - last_rx_byte_ > 500)) {
      // wenn es schon wie ein Long-Frame aussieht, aber evtl. Längenbyte nicht zuverlässig kam
      if (rx_buffer_.back() == 0x16 && rx_buffer_.size() > 20) complete = true;
    }

    if (complete) {
      auto &f = rx_buffer_;

      // Grundchecks
      if (f.size() < 25 || f[0] != 0x68 || f[3] != 0x68) {
        ESP_LOGW(TAG, "Frame invalid (len=%u)", (unsigned)f.size());
        rx_buffer_.clear();
        state = UM_IDLE;
        return;
      }

      // Layout: 68 L L 68 C A CI [12 bytes fixed header] [records...] CS 16
      uint8_t ci = f[6];

      // Nur CI72 (variable data) unterstützt hier
      if (ci != 0x72 && ci != 0x73) {
        ESP_LOGW(TAG, "Unsupported CI: 0x%02X", ci);
        rx_buffer_.clear();
        state = UM_IDLE;
        return;
      }

      size_t ptr = 7 + 12;        // <-- FIX: records start AFTER fixed header
      size_t end = f.size() - 2;  // stop before CS+16

      ESP_LOGI(TAG, "Parsing Frame (%u Bytes), records @%u..%u", (unsigned)f.size(), (unsigned)ptr, (unsigned)end);

      while (ptr < end) {
        uint8_t dif = f[ptr++];

        // Special: manufacturer specific / no more records
        if (dif == 0x2F || dif == 0x0F) break;

        // Skip DIFE extensions
        while ((dif & 0x80) && ptr < end) {
          uint8_t dife = f[ptr++];
          (void)dife;
          dif &= 0x7F;
        }

        if (ptr >= end) break;
        uint8_t vif = f[ptr++];

        // Skip VIFE extensions
        while ((vif & 0x80) && ptr < end) {
          uint8_t vife = f[ptr++];
          (void)vife;
          vif &= 0x7F;
        }

        int len = 0;
        switch (dif & 0x0F) {
          case 0x00: len = 0; break;
          case 0x01: len = 1; break;
          case 0x02: len = 2; break;
          case 0x03: len = 3; break;
          case 0x04: len = 4; break;
          case 0x06: len = 6; break;
          case 0x07: len = 8; break;
          case 0x0A: len = 2; break;  // BCD4 (2 bytes)
          case 0x0B: len = 3; break;  // BCD6 (3 bytes)
          case 0x0C: len = 4; break;  // BCD8 (4 bytes)
          default: len = 0; break;
        }

        if (ptr + (size_t)len > end) break;

        // --- RECORDS (passend zu deinem Telegramm / Thread) ---
        // Serial: DIF 0x0C, VIF 0x78, 4 bytes BCD
        if (serial_number_ && (dif & 0x0F) == 0x0C && vif == 0x78 && len == 4) {
          serial_number_->publish_state(decode_bcd(f, ptr, 4));
        }

        // Total energy: DIF 0x04, VIF 0x06, 4 bytes (u32 LE) -> *0.001 = MWh (wie in Thread/Tasmota)
        if (total_energy_ && (dif & 0x0F) == 0x04 && vif == 0x06 && len == 4) {
          uint32_t raw = decode_u_le(f, ptr, 4);
          total_energy_->publish_state((float)raw * 0.001f);
        }

        // Total volume: DIF 0x0C, VIF 0x14, 4 bytes BCD -> *0.01 = m³ (34 39 16 00 => 163934 => 1639.34)
        if (total_volume_ && (dif & 0x0F) == 0x0C && vif == 0x14 && len == 4) {
          float raw_bcd = decode_bcd(f, ptr, 4);
          total_volume_->publish_state(raw_bcd * 0.01f);
        }

        // Current power: häufig DIF 0x3B (storage bits) => low nibble 0x0B, VIF 0x2D, 3 bytes BCD
        // 99 99 99 bedeutet "invalid" (Thread)
        if (current_power_ && (dif & 0x0F) == 0x0B && vif == 0x2D && len == 3) {
          if (!(f[ptr] == 0x99 && f[ptr + 1] == 0x99 && f[ptr + 2] == 0x99)) {
            float p = decode_bcd(f, ptr, 3) * 0.1f;  // @10 => kW
            current_power_->publish_state(p);
          }
        }

        // Flow temp: DIF 0x0A, VIF 0x5A, 2 bytes BCD -> *0.1 °C
        if (temp_flow_ && (dif & 0x0F) == 0x0A && vif == 0x5A && len == 2) {
          temp_flow_->publish_state(decode_bcd(f, ptr, 2) * 0.1f);
        }

        // Return temp: DIF 0x0A, VIF 0x5E, 2 bytes BCD -> *0.1 °C
        if (temp_return_ && (dif & 0x0F) == 0x0A && vif == 0x5E && len == 2) {
          temp_return_->publish_state(decode_bcd(f, ptr, 2) * 0.1f);
        }

        // Delta T: DIF 0x0B, VIF 0x61, 3 bytes BCD -> /100 K
        if (temp_diff_ && (dif & 0x0F) == 0x0B && vif == 0x61 && len == 3) {
          temp_diff_->publish_state(decode_bcd(f, ptr, 3) * 0.01f);
        }

        // Meter time: DIF 0x04, VIF 0x6D, 4 bytes CP32
        if (meter_time_ && (dif & 0x0F) == 0x04 && vif == 0x6D && len == 4) {
          std::string ts;
          if (decode_cp32_datetime_(f, ptr, ts)) {
            meter_time_->publish_state(ts);
          }
        }

        ptr += (size_t)len;
      }

      rx_buffer_.clear();
      state = UM_IDLE;
      ESP_LOGI(TAG, "Update finished.");
      return;
    }

    // Timeout
    if (now - state_ts_ > 10000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
      return;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
