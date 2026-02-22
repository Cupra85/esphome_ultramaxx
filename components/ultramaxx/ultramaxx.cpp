#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v7.0";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------- Decoder --------------------

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
  for (size_t i = 0; i < len; i++) v |= ((uint32_t)data[start + i]) << (8 * i);
  return v;
}

// CP32 (Type F) = 4 Bytes, byteweise Dekodierung (Minute/Hour/Day/Month/Year + Flags)
bool UltraMaXXComponent::decode_cp32_datetime_(
    const std::vector<uint8_t> &data,
    size_t start,
    std::string &out) const {

  if (start + 4 > data.size()) return false;

  const uint8_t b0 = data[start + 0];
  const uint8_t b1 = data[start + 1];
  const uint8_t b2 = data[start + 2];
  const uint8_t b3 = data[start + 3];

  // Bit7 in b0: time invalid
  if (b0 & 0x80) return false;

  const uint8_t minute = (uint8_t)(b0 & 0x3F);      // 0..59
  const uint8_t hour   = (uint8_t)(b1 & 0x1F);      // 0..23
  const bool dst       = (b1 & 0x80) != 0;          // DST flag (Sommerzeit aktiv)

  const uint8_t day    = (uint8_t)(b2 & 0x1F);      // 1..31
  const uint8_t month  = (uint8_t)(b3 & 0x0F);      // 1..12

  // year bits: (b2 bits5..7) + (b3 bits4..7 shifted)
  const uint8_t year =
      (uint8_t)(((b2 & 0xE0) >> 5) | ((b3 & 0xF0) >> 1)); // 0..127

  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) return false;

  const int full_year = 2000 + (int) year;

  char buf[40];
  std::snprintf(buf, sizeof(buf), "%02u.%02u.%04d %02u:%02u", day, month, full_year, hour, minute);

  (void) dst; // aktuell nur optional genutzt
  out = buf;
  return true;
}

// -------------------- Status Decoder (Byte-Pos 16) --------------------
// Bitmask-fähig gemäß deiner Tabelle:
// 0x00 = All OK
// 0x04 = Low Battery
// 0x08 = Permanent Error
// 0x10 = Temporary Error
std::string UltraMaXXComponent::decode_status_text_(uint8_t status) const {
  // ✅ Explizit: 0x00 => "All OK"
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

  char head[16];
  std::snprintf(head, sizeof(head), "0x%02X: ", status);
  return std::string(head) + s;
}

// -------------------- Flags --------------------

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = false;
  got_energy_ = false;
  got_volume_ = false;
  got_tflow_  = false;
  got_tret_   = false;
  got_tdiff_  = false;
  got_time_   = false;
  got_operating_ = false;
  got_error_ = false;
  got_access_counter_ = false;
  got_status_ = false;
}

// -------------------- Parser --------------------

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  const size_t n = buf.size();
  if (n < 10) return;

  // -------- Fixed Data Header Felder (Byte-Pos 15/16) --------
  // Byte-Pos laut deiner Tabelle:
  // 15 = Zugriffszähler
  // 16 = Status
  if (n > 16) {
    if (!got_access_counter_) {
      const uint8_t access = buf[15];
      got_access_counter_ = true;
      ESP_LOGI(TAG, "ACCESS COUNTER parsed: %u", (unsigned) access);
      if (access_counter_) access_counter_->publish_state((float) access);
    }

    if (!got_status_) {
      const uint8_t status = buf[16];
      got_status_ = true;

      // ✅ Status wird IMMER verarbeitet, auch wenn status == 0x00
      const std::string st = decode_status_text_(status);
      ESP_LOGI(TAG, "STATUS parsed: %s", st.c_str());
      if (status_text_) status_text_->publish_state(st);
    }
  }

  // -------- Variable Data Blocks (Marker-Scan wie bisher) --------
  for (size_t i = 0; i + 2 < n; i++) {

    if (!got_serial_ && i + 6 <= n && buf[i]==0x0C && buf[i+1]==0x78) {
      float sn = decode_bcd_(buf,i+2,4);
      got_serial_ = true;
      ESP_LOGI(TAG,"SERIAL parsed: %.0f",sn);
      if(serial_number_) serial_number_->publish_state(sn);
    }

    if (!got_energy_ && i + 6 <= n && buf[i]==0x04 && buf[i+1]==0x06) {
      uint32_t raw = decode_u_le_(buf,i+2,4);
      float kwh = (float)raw;
      got_energy_=true;
      ESP_LOGI(TAG,"ENERGY parsed: %.0f kWh (raw=%u)",kwh,(unsigned)raw);
      if(total_energy_) total_energy_->publish_state(kwh);
    }

    if (!got_volume_ && i + 6 <= n && buf[i]==0x0C && buf[i+1]==0x14) {
      float v = decode_bcd_(buf,i+2,4)*0.01f;
      got_volume_=true;
      ESP_LOGI(TAG,"VOLUME parsed: %.2f m³",v);
      if(total_volume_) total_volume_->publish_state(v);
    }

    if (!got_tflow_ && i + 4 <= n && buf[i]==0x0A && buf[i+1]==0x5A) {
      float t = decode_bcd_(buf,i+2,2)*0.1f;
      got_tflow_=true;
      ESP_LOGI(TAG,"FLOW TEMP parsed: %.1f °C",t);
      if(temp_flow_) temp_flow_->publish_state(t);
    }

    if (!got_tret_ && i + 4 <= n && buf[i]==0x0A && buf[i+1]==0x5E) {
      float t = decode_bcd_(buf,i+2,2)*0.1f;
      got_tret_=true;
      ESP_LOGI(TAG,"RETURN TEMP parsed: %.1f °C",t);
      if(temp_return_) temp_return_->publish_state(t);
    }

    if (!got_tdiff_ && i + 5 <= n && buf[i]==0x0B && buf[i+1]==0x61) {
      float dt = decode_bcd_(buf,i+2,3)*0.01f;
      got_tdiff_=true;
      ESP_LOGI(TAG,"DELTA T parsed: %.2f K",dt);
      if(temp_diff_) temp_diff_->publish_state(dt);
    }

    if (!got_time_ && i + 6 <= n && buf[i]==0x04 && buf[i+1]==0x6D) {
      std::string ts;
      if(decode_cp32_datetime_(buf,i+2,ts)) {
        got_time_=true;
        ESP_LOGI(TAG,"TIME parsed: %s",ts.c_str());
        if(meter_time_) meter_time_->publish_state(ts);
      }
    }

    if (!got_operating_ && i + 4 <= n && buf[i]==0x02 && buf[i+1]==0x27) {
      uint32_t days = decode_u_le_(buf,i+2,2);
      got_operating_=true;
      ESP_LOGI(TAG,"OPERATING TIME parsed: %u days",(unsigned)days);
      if(operating_time_) operating_time_->publish_state(days);
    }

    if (!got_error_ && i + 4 <= n && buf[i]==0x09 && buf[i+1]==0xFD && buf[i+2]==0x0E) {
      uint32_t hours = buf[i+3];
      got_error_=true;
      ESP_LOGI(TAG,"ERROR TIME parsed: %u h",(unsigned)hours);
      if(error_time_) error_time_->publish_state(hours);
    }
  }
}

// -------------------- Component lifecycle --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

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
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c)) break;

    last_rx_ms_ = now;

    if (!in_frame_) {
      if (c != 0x68) continue;
      in_frame_ = true;
      rx_buffer_.clear();
      expected_len_ = 0;
      rx_buffer_.push_back(c);
      continue;
    }

    rx_buffer_.push_back(c);

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
      expected_len_ = (size_t) (4 + L1 + 2);
    }

    this->parse_and_publish_(rx_buffer_);

    if (expected_len_ > 0 && rx_buffer_.size() >= expected_len_) {
      if (rx_buffer_.back() != 0x16) {
        ESP_LOGW(TAG, "Frame length reached (%u) but last byte != 0x16 (got %02X) -> resync",
                 (unsigned) rx_buffer_.size(), rx_buffer_.back());
      }
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
    }
  }

  if (state == UM_RX) {
    if (!in_frame_ && (now - last_rx_ms_ > 2200)) {
      ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
      state = UM_IDLE;
    }
    if (in_frame_ && (now - last_rx_ms_ > 2200)) {
      ESP_LOGW(TAG, "RX Timeout (in frame). buf=%u exp=%u", (unsigned) rx_buffer_.size(), (unsigned) expected_len_);
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      state = UM_IDLE;
    }
  }

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

  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    last_rx_ms_ = now;

    const uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
    return;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {
    const uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    const uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    const uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();
    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    state = UM_RX;
    last_rx_ms_ = now;
    return;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
