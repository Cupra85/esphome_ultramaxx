#include "ultramaxx.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "esphome/core/log.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v8.2";

// --------------------------------------------------------------------------------------
// Hinweis:
// Dein ultramaxx.h deklariert keine RX-/State-Member (rx_buffer_, in_frame_, ...).
// Damit es OHNE Header-Änderung kompiliert, halten wir den Runtime-State hier static.
// Das setzt (wie bei den meisten Installationen) genau 1 Instanz voraus.
// --------------------------------------------------------------------------------------

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState g_state = UM_IDLE;

// UART / Frame RX state
static std::vector<uint8_t> g_rx_buffer;
static bool g_in_frame = false;
static size_t g_expected_len = 0;
static uint32_t g_last_rx_ms = 0;

// Wakeup / timing
static uint32_t g_wake_start = 0;
static uint32_t g_last_send = 0;
static uint32_t g_state_ts = 0;

// M-Bus FCB toggle for REQ_UD2
static bool g_fcb_toggle = false;

// Parse-once flags (pro update() Reset)
static bool g_got_serial = false;
static bool g_got_energy = false;
static bool g_got_volume = false;
static bool g_got_tflow = false;
static bool g_got_tret = false;
static bool g_got_tdiff = false;
static bool g_got_time = false;
static bool g_got_operating = false;
static bool g_got_access_counter = false;
static bool g_got_status = false;

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
  g_got_serial = false;
  g_got_energy = false;
  g_got_volume = false;
  g_got_tflow = false;
  g_got_tret = false;
  g_got_tdiff = false;
  g_got_time = false;
  g_got_operating = false;
  g_got_access_counter = false;
  g_got_status = false;

  // Diese vier Flags existieren im Header -> nutzen wir auch sauber als "parsed once"
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
  if (n < 10) return;

  // ---------- Fixed Data Header Felder (Byte-Pos 15/16) ----------
  // 15 = Zugriffszähler
  // 16 = Status
  if (n > 16) {
    if (!g_got_access_counter) {
      const uint8_t access = buf[15];
      g_got_access_counter = true;
      ESP_LOGI(TAG, "ACCESS COUNTER parsed: %u", (unsigned) access);
      if (access_counter_) access_counter_->publish_state((float) access);
    }

    if (!g_got_status) {
      const uint8_t status = buf[16];
      g_got_status = true;
      const std::string st = decode_status_text_(status);
      ESP_LOGI(TAG, "STATUS parsed: %s", st.c_str());
      if (status_text_) status_text_->publish_state(st);
    }
  }

  // ---------- Variable Data Blocks (Marker-Scan) ----------
  for (size_t i = 0; i + 2 < n; i++) {
    // SERIAL: 0x0C 0x78 + 4B BCD
    if (!g_got_serial && i + 6 <= n && buf[i] == 0x0C && buf[i + 1] == 0x78) {
      float sn = decode_bcd_(buf, i + 2, 4);
      g_got_serial = true;
      ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
      if (serial_number_) serial_number_->publish_state(sn);
    }

    // ENERGY: 0x04 0x06 + 4B little endian (kWh)
    if (!g_got_energy && i + 6 <= n && buf[i] == 0x04 && buf[i + 1] == 0x06) {
      uint32_t raw = decode_u_le_(buf, i + 2, 4);
      float kwh = (float) raw;
      g_got_energy = true;
      ESP_LOGI(TAG, "ENERGY parsed: %.0f kWh (raw=%u)", kwh, (unsigned) raw);
      if (total_energy_) total_energy_->publish_state(kwh);
    }

    // VOLUME: 0x0C 0x14 + 4B BCD (x0.01 m³)
    if (!g_got_volume && i + 6 <= n && buf[i] == 0x0C && buf[i + 1] == 0x14) {
      float v = decode_bcd_(buf, i + 2, 4) * 0.01f;
      g_got_volume = true;
      ESP_LOGI(TAG, "VOLUME parsed: %.2f m³", v);
      if (total_volume_) total_volume_->publish_state(v);
    }

    // FLOW TEMP: 0x0A 0x5A + 2B BCD (x0.1 °C)
    if (!g_got_tflow && i + 4 <= n && buf[i] == 0x0A && buf[i + 1] == 0x5A) {
      float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
      g_got_tflow = true;
      ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f °C", t);
      if (temp_flow_) temp_flow_->publish_state(t);
    }

    // RETURN TEMP: 0x0A 0x5E + 2B BCD (x0.1 °C)
    if (!g_got_tret && i + 4 <= n && buf[i] == 0x0A && buf[i + 1] == 0x5E) {
      float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
      g_got_tret = true;
      ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f °C", t);
      if (temp_return_) temp_return_->publish_state(t);
    }

    // DELTA T: 0x0B 0x61 + 3B BCD (x0.01 K)
    if (!g_got_tdiff && i + 5 <= n && buf[i] == 0x0B && buf[i + 1] == 0x61) {
      float dt = decode_bcd_(buf, i + 2, 3) * 0.01f;
      g_got_tdiff = true;
      ESP_LOGI(TAG, "DELTA T parsed: %.2f K", dt);
      if (temp_diff_) temp_diff_->publish_state(dt);
    }

    // TIME: 0x04 0x6D + 4B CP32
    if (!g_got_time && i + 6 <= n && buf[i] == 0x04 && buf[i + 1] == 0x6D) {
      std::string ts;
      if (decode_cp32_datetime_(buf, i + 2, ts)) {
        g_got_time = true;
        ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
        if (meter_time_) meter_time_->publish_state(ts);
      }
    }

    // OPERATING TIME: 0x02 0x27 + 2B LE (days)
    if (!g_got_operating && i + 4 <= n && buf[i] == 0x02 && buf[i + 1] == 0x27) {
      uint32_t days = decode_u_le_(buf, i + 2, 2);
      g_got_operating = true;
      ESP_LOGI(TAG, "OPERATING TIME parsed: %u days", (unsigned) days);
      if (operating_time_) operating_time_->publish_state((float) days);
    }

    // POWER: 0x0B 0x2D/0x2E + 3B BCD (x0.1 kW)  (Annahme wie bisher)
    if (!got_power_ && i + 5 <= n && buf[i] == 0x0B && (buf[i + 1] == 0x2D || buf[i + 1] == 0x2E)) {
      float p = decode_bcd_(buf, i + 2, 3) * 0.1f;
      got_power_ = true;
      ESP_LOGI(TAG, "POWER parsed: %.1f kW", p);
      if (current_power_) current_power_->publish_state(p);
    }

    // FLOW: 0x0B 0x3B + 3B BCD (l/h)  (Annahme wie bisher)
    if (!got_flow_ && i + 5 <= n && buf[i] == 0x0B && buf[i + 1] == 0x3B) {
      float f = decode_bcd_(buf, i + 2, 3);
      got_flow_ = true;
      ESP_LOGI(TAG, "FLOW parsed: %.0f l/h", f);
      if (flow_) flow_->publish_state(f);
    }

    // FIRMWARE: 0x09 0xFD 0x0E + 1B (hier als unsigned int -> float)
    if (!got_fw_ && i + 4 <= n && buf[i] == 0x09 && buf[i + 1] == 0xFD && buf[i + 2] == 0x0E) {
      uint8_t fw_raw = buf[i + 3];
      float fw = (float) fw_raw;
      got_fw_ = true;
      ESP_LOGI(TAG, "FIRMWARE VERSION parsed: %.0f", fw);
      if (firmware_version_) firmware_version_->publish_state(fw);
    }

    // SOFTWARE: 0x09 0xFD 0x0F + 1B (hier als unsigned int -> float)
    if (!got_sw_ && i + 4 <= n && buf[i] == 0x09 && buf[i + 1] == 0xFD && buf[i + 2] == 0x0F) {
      uint8_t sw_raw = buf[i + 3];
      float sw = (float) sw_raw;
      got_sw_ = true;
      ESP_LOGI(TAG, "SOFTWARE VERSION parsed: %.0f", sw);
      if (software_version_) software_version_->publish_state(sw);
    }
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

  g_rx_buffer.clear();
  g_in_frame = false;
  g_expected_len = 0;
  g_last_rx_ms = 0;

  reset_parse_flags_();

  g_wake_start = millis();
  g_last_send = 0;
  g_state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // -------------------- RX loop --------------------
  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c)) break;

    g_last_rx_ms = now;

    if (!g_in_frame) {
      if (c != 0x68) continue;  // wait start
      g_in_frame = true;
      g_rx_buffer.clear();
      g_expected_len = 0;
      g_rx_buffer.push_back(c);
      continue;
    }

    g_rx_buffer.push_back(c);

    // After 4 bytes: 68 L L 68
    if (g_rx_buffer.size() == 4) {
      const uint8_t L1 = g_rx_buffer[1];
      const uint8_t L2 = g_rx_buffer[2];
      const uint8_t S2 = g_rx_buffer[3];
      if (S2 != 0x68 || L1 != L2 || L1 < 3) {
        g_in_frame = false;
        g_rx_buffer.clear();
        g_expected_len = 0;
        continue;
      }
      // total = 4(header) + L(data) + 2(CS + 0x16)
      g_expected_len = (size_t) (4 + L1 + 2);
    }

    // Frame complete?
    if (g_expected_len > 0 && g_rx_buffer.size() >= g_expected_len) {
      if (g_rx_buffer.back() == 0x16) {
        // optional: hex dump once per full frame
        std::string hex;
        char tmp[4];
        for (uint8_t b : g_rx_buffer) {
          std::snprintf(tmp, sizeof(tmp), "%02X ", b);
          hex += tmp;
        }
        ESP_LOGI(TAG, "FRAME HEX: %s", hex.c_str());

        parse_and_publish_(g_rx_buffer);
      } else {
        ESP_LOGW(TAG,
                 "Frame length reached (%u) but last byte != 0x16 (got %02X) -> resync",
                 (unsigned) g_rx_buffer.size(), g_rx_buffer.back());
      }

      g_in_frame = false;
      g_rx_buffer.clear();
      g_expected_len = 0;
    }
  }

  // -------------------- RX timeout handling --------------------
  if (g_state == UM_RX) {
    if (!g_in_frame && (now - g_last_rx_ms > 2200)) {
      ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) g_rx_buffer.size());
      g_state = UM_IDLE;
    }
    if (g_in_frame && (now - g_last_rx_ms > 2200)) {
      ESP_LOGW(TAG, "RX Timeout (in frame). buf=%u exp=%u", (unsigned) g_rx_buffer.size(), (unsigned) g_expected_len);
      g_in_frame = false;
      g_rx_buffer.clear();
      g_expected_len = 0;
      g_state = UM_IDLE;
    }
  }

  // -------------------- State machine --------------------
  if (g_state == UM_WAKEUP) {
    // send 0x55 for ~2.2s
    if (now - g_last_send > 15) {
      uint8_t buf[20];
      std::memset(buf, 0x55, sizeof(buf));
      this->write_array(buf, sizeof(buf));
      g_last_send = now;
    }
    if (now - g_wake_start > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      g_state = UM_WAIT;
      g_state_ts = now;
    }
    return;
  }

  if (g_state == UM_WAIT && now - g_state_ts > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // flush input
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    g_rx_buffer.clear();
    g_in_frame = false;
    g_expected_len = 0;
    g_last_rx_ms = now;

    // SND_NKE (reset/link)
    const uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE sent");

    g_state = UM_SEND;
    g_state_ts = now;
    return;
  }

  if (g_state == UM_SEND && now - g_state_ts > 150) {
    // REQ_UD2 with FCB toggle
    const uint8_t ctrl = g_fcb_toggle ? 0x7B : 0x5B;
    const uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    const uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();
    ESP_LOGI(TAG, "REQ_UD2 sent");

    g_fcb_toggle = !g_fcb_toggle;

    g_state = UM_RX;
    g_last_rx_ms = now;
    return;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
