#include "ultramaxx.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "esphome/core/log.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v10.0";

// --------------------------------------------------------------------------------------
// Decoder
// --------------------------------------------------------------------------------------

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return NAN;
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
  for (size_t i = 0; i < len; i++) v |= ((uint32_t) data[start + i]) << (8 * i);
  return v;
}

// CP32 (Type F) = 4 Bytes, byteweise Dekodierung (Minute/Hour/Day/Month/Year + Flags)
bool UltraMaXXComponent::decode_cp32_datetime_(
    const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;

  const uint8_t b0 = data[start + 0];
  const uint8_t b1 = data[start + 1];
  const uint8_t b2 = data[start + 2];
  const uint8_t b3 = data[start + 3];

  // Bit7 in b0: time invalid
  if (b0 & 0x80) return false;

  const uint8_t minute = (uint8_t) (b0 & 0x3F);  // 0..59
  const uint8_t hour = (uint8_t) (b1 & 0x1F);    // 0..23
  // const bool dst = (b1 & 0x80) != 0;          // DST flag (optional)

  const uint8_t day = (uint8_t) (b2 & 0x1F);    // 1..31
  const uint8_t month = (uint8_t) (b3 & 0x0F);  // 1..12

  // year bits: (b2 bits5..7) + (b3 bits4..7 shifted)
  const uint8_t year = (uint8_t) (((b2 & 0xE0) >> 5) | ((b3 & 0xF0) >> 1));  // 0..127

  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) return false;

  const int full_year = 2000 + (int) year;

  char buf[40];
  std::snprintf(buf, sizeof(buf), "%02u.%02u.%04d %02u:%02u", day, month, full_year, hour, minute);
  out = buf;
  return true;
}

// --------------------------------------------------------------------------------------
// Status Decoder (Byte-Pos 16)
// --------------------------------------------------------------------------------------
// Bitmask:
// 0x00 = All OK
// 0x04 = Low Battery
// 0x08 = Permanent Error
// 0x10 = Temporary Error
std::string UltraMaXXComponent::decode_status_text_(uint8_t status) const {
  if (status == 0x00) return "All OK";

  std::string s;
  auto add = [&](const char *part) {
    if (!s.empty()) s += " + ";
    s += part;
  };

  if (status & 0x04) add("Low Battery");
  if (status & 0x08) add("Permanent Error");
  if (status & 0x10) add("Temporary Error");

  const uint8_t known = 0x04 | 0x08 | 0x10;
  const uint8_t unknown = status & ~known;
  if (unknown) {
    char buf[24];
    std::snprintf(buf, sizeof(buf), "Unknown(0x%02X)", unknown);
    add(buf);
  }

  return s.empty() ? "Unknown" : s;
}

// --------------------------------------------------------------------------------------
// Flags
// --------------------------------------------------------------------------------------

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = false;
  got_energy_ = false;
  got_volume_ = false;
  got_tflow_ = false;
  got_tret_ = false;
  got_tdiff_ = false;
  got_time_ = false;
  got_operating_ = false;
  got_access_counter_ = false;
  got_status_ = false;

  got_power_ = false;
  got_flow_ = false;
  got_fw_ = false;
  got_sw_ = false;
}

// --------------------------------------------------------------------------------------
// Parser
// --------------------------------------------------------------------------------------

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  const size_t n = buf.size();
  if (n < 20) return;

  // ---------------- FIXED DATA HEADER ----------------
  if (n > 16) {
    if (!got_access_counter_) {
      uint8_t access = buf[15];
      got_access_counter_ = true;
      ESP_LOGI(TAG, "ACCESS COUNTER parsed: %u", (unsigned) access);
      if (access_counter_) access_counter_->publish_state((float) access);
    }

    if (!got_status_) {
      uint8_t status = buf[16];
      got_status_ = true;
      std::string st = decode_status_text_(status);
      ESP_LOGI(TAG, "STATUS parsed: %s", st.c_str());
      if (status_text_) status_text_->publish_state(st);
    }
  }

  // ---------------- VARIABLE DATA RECORDS ----------------
  size_t i = 19;
  const size_t end = n - 2;

  while (i < end) {
    uint8_t dif = buf[i++];
    if (dif == 0x2F) continue;

    bool dif_ext = dif & 0x80;
    bool dif_error = dif & 0x40;
    uint8_t dif_len_code = dif & 0x0F;

    if (dif_ext) {
      while (i < end) {
        uint8_t dife = buf[i++];
        if (!(dife & 0x80)) break;
      }
    }

    if (i >= end) break;

    uint8_t vif = buf[i++];
    bool vif_ext = vif & 0x80;
    uint8_t vif_base = vif & 0x7F;

    // ---- VIFE sauber hier lesen (WICHTIG) ----
    uint8_t last_vife = 0;
    if (vif_ext) {
      while (i < end) {
        last_vife = buf[i++];
        if (!(last_vife & 0x80)) break;
      }
    }

    size_t dlen = 0;
    switch (dif_len_code) {
      case 0x01:
        dlen = 1;
        break;
      case 0x02:
        dlen = 2;
        break;
      case 0x03:
        dlen = 3;
        break;
      case 0x04:
        dlen = 4;
        break;
      case 0x09:
        dlen = 1;
        break;
      case 0x0A:
        dlen = 2;
        break;
      case 0x0B:
        dlen = 3;
        break;
      case 0x0C:
        dlen = 4;
        break;
      default:
        return;
    }

    if (i + dlen > end) break;

    // -------- INVALID CHECK --------
    bool invalid_99 = false;
    if (dlen >= 2) {
      invalid_99 = true;
      for (size_t k = 0; k < dlen; k++) {
        if (buf[i + k] != 0x99) {
          invalid_99 = false;
          break;
        }
      }
    }

    bool invalid_value = dif_error || invalid_99;

    // ================= SERIAL =================
    if (!got_serial_ && dif_len_code == 0x0C && vif_base == 0x78) {
      if (invalid_value) {
        ESP_LOGI(TAG, "SERIAL parsed: unknown");
        if (serial_number_) serial_number_->publish_state(NAN);
      } else {
        float sn = decode_bcd_(buf, i, 4);
        ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
        if (serial_number_) serial_number_->publish_state(sn);
      }
      got_serial_ = true;
    }

    // ================= ENERGY =================
    if (!got_energy_ && dif_len_code == 0x04 && vif_base == 0x06) {
      if (invalid_value) {
        ESP_LOGI(TAG, "ENERGY parsed: unknown");
        if (total_energy_) total_energy_->publish_state(NAN);
      } else {
        uint32_t raw = decode_u_le_(buf, i, 4);
        ESP_LOGI(TAG, "ENERGY parsed: %u kWh", (unsigned) raw);
        if (total_energy_) total_energy_->publish_state((float) raw);
      }
      got_energy_ = true;
    }

    // ================= VOLUME =================
    if (!got_volume_ && dif_len_code == 0x0C && vif_base == 0x14) {
      if (invalid_value) {
        ESP_LOGI(TAG, "VOLUME parsed: unknown");
        if (total_volume_) total_volume_->publish_state(NAN);
      } else {
        float v = decode_bcd_(buf, i, 4) * 0.01f;
        ESP_LOGI(TAG, "VOLUME parsed: %.2f m³", v);
        if (total_volume_) total_volume_->publish_state(v);
      }
      got_volume_ = true;
    }

    // ================= FLOW TEMP =================
    if (!got_tflow_ && dif_len_code == 0x0A && vif_base == 0x5A) {
      if (invalid_value) {
        ESP_LOGI(TAG, "FLOW TEMP parsed: unknown");
        if (temp_flow_) temp_flow_->publish_state(NAN);
      } else {
        float t = decode_bcd_(buf, i, 2) * 0.1f;
        ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f °C", t);
        if (temp_flow_) temp_flow_->publish_state(t);
      }
      got_tflow_ = true;
    }

    // ================= RETURN TEMP =================
    if (!got_tret_ && dif_len_code == 0x0A && vif_base == 0x5E) {
      if (invalid_value) {
        ESP_LOGI(TAG, "RETURN TEMP parsed: unknown");
        if (temp_return_) temp_return_->publish_state(NAN);
      } else {
        float t = decode_bcd_(buf, i, 2) * 0.1f;
        ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f °C", t);
        if (temp_return_) temp_return_->publish_state(t);
      }
      got_tret_ = true;
    }

    // ================= DELTA T =================
    if (!got_tdiff_ && dif_len_code == 0x0B && vif_base == 0x61) {
      if (invalid_value) {
        ESP_LOGI(TAG, "DELTA T parsed: unknown");
        if (temp_diff_) temp_diff_->publish_state(NAN);
      } else {
        float dt = decode_bcd_(buf, i, 3) * 0.01f;
        ESP_LOGI(TAG, "DELTA T parsed: %.2f °C", dt);
        if (temp_diff_) temp_diff_->publish_state(dt);
      }
      got_tdiff_ = true;
    }

    // ================= POWER =================
    if (!got_power_ && dif_len_code == 0x0B && (vif_base == 0x2D || vif_base == 0x2E)) {
      if (invalid_value) {
        ESP_LOGI(TAG, "POWER parsed: unknown");
        if (current_power_) current_power_->publish_state(NAN);
      } else {
        float p = decode_bcd_(buf, i, 3) * 0.1f;
        ESP_LOGI(TAG, "POWER parsed: %.1f kW", p);
        if (current_power_) current_power_->publish_state(p);
      }
      got_power_ = true;
    }

    // ================= FLOW =================
    if (!got_flow_ && dif_len_code == 0x0B && vif_base == 0x3B) {
      if (invalid_value) {
        ESP_LOGI(TAG, "FLOW parsed: unknown");
        if (flow_) flow_->publish_state(NAN);
      } else {
        float f = decode_bcd_(buf, i, 3);
        ESP_LOGI(TAG, "FLOW parsed: %.0f l/h", f);
        if (flow_) flow_->publish_state(f);
      }
      got_flow_ = true;
    }

    // ================= OPERATING TIME =================
    if (!got_operating_ && dif_len_code == 0x02 && vif_base == 0x27) {
      if (invalid_value) {
        ESP_LOGI(TAG, "OPERATING TIME parsed: unknown");
        if (operating_time_) operating_time_->publish_state(NAN);
      } else {
        uint32_t days = decode_u_le_(buf, i, 2);
        ESP_LOGI(TAG, "OPERATING TIME parsed: %u days", (unsigned) days);
        if (operating_time_) operating_time_->publish_state((float) days);
      }
      got_operating_ = true;
    }

    // ================= TIME =================
    if (!got_time_ && dif_len_code == 0x04 && vif_base == 0x6D) {
      if (dif_error) {
        ESP_LOGI(TAG, "TIME parsed: unknown");
        if (meter_time_) meter_time_->publish_state("unknown");
      } else {
        std::string ts;
        if (decode_cp32_datetime_(buf, i, ts)) {
          ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
          if (meter_time_) meter_time_->publish_state(ts);
        } else {
          ESP_LOGI(TAG, "TIME parsed: unknown");
          if (meter_time_) meter_time_->publish_state("unknown");
        }
      }
      got_time_ = true;
    }

    // ================= FIRMWARE VERSION =================
    if (!got_fw_ && dif_len_code == 0x09 && vif_base == 0x7D && last_vife == 0x0E) {
      if (invalid_value) {
        ESP_LOGI(TAG, "FIRMWARE VERSION parsed: unknown");
        if (firmware_version_) firmware_version_->publish_state(NAN);
      } else {
        float fw = decode_bcd_(buf, i, 1);
        ESP_LOGI(TAG, "FIRMWARE VERSION parsed: %.0f", fw);
        if (firmware_version_) firmware_version_->publish_state(fw);
      }
      got_fw_ = true;
    }

    // ================= SOFTWARE VERSION =================
    if (!got_sw_ && dif_len_code == 0x09 && vif_base == 0x7D && last_vife == 0x0F) {
      if (invalid_value) {
        ESP_LOGI(TAG, "SOFTWARE VERSION parsed: unknown");
        if (software_version_) software_version_->publish_state(NAN);
      } else {
        float sw = decode_bcd_(buf, i, 1);
        ESP_LOGI(TAG, "SOFTWARE VERSION parsed: %.0f", sw);
        if (software_version_) software_version_->publish_state(sw);
      }
      got_sw_ = true;
    }

    i += dlen;
  }
}

// --------------------------------------------------------------------------------------
// Component lifecycle
// --------------------------------------------------------------------------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // Wakeup: 2400 8N1 + 0x55 bursts
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  in_frame_ = false;
  expected_len_ = 0;
  last_rx_ms_ = 0;

  reset_parse_flags_();

  wake_start_ = millis();
  last_send_ = 0;
  state_ = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // -------------------- RX loop --------------------
  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c)) break;

    last_rx_ms_ = now;

    if (!in_frame_) {
      if (c != 0x68) continue;  // wait start
      in_frame_ = true;
      rx_buffer_.clear();
      expected_len_ = 0;
      rx_buffer_.push_back(c);
      continue;
    }

    rx_buffer_.push_back(c);

    // After 4 bytes: 68 L L 68
    if (rx_buffer_.size() == 4) {
      const uint8_t L1 = rx_buffer_[1];
      const uint8_t L2 = rx_buffer_[2];
      const uint8_t S2 = rx_buffer_[3];
      if (S2 != 0x68 || L1 != L2 || L1 < 3) {
        in_frame_ = false;
        rx_buffer_.clear();
        expected_len_ = 0;
        continue;
      }
      // total = 4(header) + L(data) + 2(CS + 0x16)
      expected_len_ = (size_t) (4 + L1 + 2);
    }

    // Frame complete?
    if (expected_len_ > 0 && rx_buffer_.size() >= expected_len_) {
      if (rx_buffer_.back() == 0x16) {
        std::string hex;
        char tmp[4];
        for (uint8_t b : rx_buffer_) {
          std::snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "FRAME HEX: %s", hex.c_str());

        parse_and_publish_(rx_buffer_);
      } else {
        ESP_LOGW(TAG, "Frame length reached (%u) but last byte != 0x16 (got %02X) -> resync",
                 (unsigned) rx_buffer_.size(), rx_buffer_.back());
      }

      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
    }
  }

  // -------------------- RX timeout handling --------------------
  if (state_ == UM_RX) {
    if (!in_frame_ && (now - last_rx_ms_ > 2200)) {
      ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
      state_ = UM_IDLE;
    }
    if (in_frame_ && (now - last_rx_ms_ > 2200)) {
      ESP_LOGW(TAG, "RX Timeout (in frame). buf=%u exp=%u", (unsigned) rx_buffer_.size(),
               (unsigned) expected_len_);
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      state_ = UM_IDLE;
    }
  }

  // -------------------- State machine --------------------
  if (state_ == UM_WAKEUP) {
    // send 0x55 for ~2.2s
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

  if (state_ == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // flush input
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    last_rx_ms_ = now;

    // SND_NKE (reset/link)
    const uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE sent");

    state_ = UM_SEND;
    state_ts_ = now;
    return;
  }

  if (state_ == UM_SEND && now - state_ts_ > 150) {
    // REQ_UD2 with FCB toggle
    const uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    const uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    const uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();
    ESP_LOGI(TAG, "REQ_UD2 sent");

    fcb_toggle_ = !fcb_toggle_;

    state_ = UM_RX;
    last_rx_ms_ = now;
    return;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
