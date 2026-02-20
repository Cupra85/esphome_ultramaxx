#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

// ============================================================
// Version / Build
// ============================================================
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v6.4";

// ============================================================
// State machine
// ============================================================
enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// ============================================================
// Helpers
// ============================================================

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return NAN;
  uint32_t value = 0;
  uint32_t mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul;
    mul *= 10;
    value += ((b >> 4) & 0x0F) * mul;
    mul *= 10;
  }
  return (float) value;
}

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= ((uint32_t) data[start + i]) << (8 * i);
  return v;
}

// CP32 date/time used by your meter looks like: DD MM HH mm (as seen in your logs)
// Your prior formatting matched: day=data[start+1], month=data[start], hour=data[start+3], min=data[start+2]
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start,
                                              std::string &out) const {
  if (start + 4 > data.size()) return false;
  char buf[32];
  snprintf(buf, sizeof(buf), "%02u.%02u %02u:%02u", data[start + 1], data[start], data[start + 3], data[start + 2]);
  out = buf;
  return true;
}

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = false;
  got_energy_ = false;
  got_volume_ = false;
  got_tflow_ = false;
  got_tret_ = false;
  got_tdiff_ = false;
  got_time_ = false;
}

// ============================================================
// Streaming parser (works on growing rx_buffer_)
// We do *not* rely on complete frame end. We just look for markers.
// BUT: to prevent the energy “Ausreißer”, we verify the expected next marker.
// ============================================================
void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  const size_t n = buf.size();
  if (n < 8) return;

  // Scan the whole buffer (cheap at ~80 bytes)
  for (size_t i = 0; i + 8 <= n; i++) {
    // SERIAL: 0C 78 + 4 BCD bytes
    if (!got_serial_ && buf[i] == 0x0C && buf[i + 1] == 0x78) {
      float sn_f = decode_bcd_(buf, i + 2, 4);
      if (!isnan(sn_f)) {
        uint32_t sn = (uint32_t) sn_f;
        ESP_LOGI(TAG, "SERIAL parsed: %u", (unsigned) sn);
        if (serial_number_) serial_number_->publish_state((float) sn);
        got_serial_ = true;
      }
    }

    // ENERGY: 04 06 + 4 bytes little endian, then we expect 0C 14 (Volume marker)
    // This avoids misaligned reads that produced values like 0x0C000014 (=201326612).
    if (!got_energy_ && buf[i] == 0x04 && buf[i + 1] == 0x06) {
      if (i + 2 + 4 + 2 <= n) {
        // Verify next marker after 4 data bytes
        const uint8_t next0 = buf[i + 2 + 4];
        const uint8_t next1 = buf[i + 2 + 4 + 1];
        if (next0 == 0x0C && next1 == 0x14) {
          uint32_t raw = decode_u_le_(buf, i + 2, 4);
          // kWh as integer
          ESP_LOGI(TAG, "ENERGY parsed: %u kWh (raw=%u)", (unsigned) raw, (unsigned) raw);
          if (total_energy_) total_energy_->publish_state((float) raw);
          got_energy_ = true;
        } else {
          // Misaligned hit: ignore silently (or enable debug if you want)
          // ESP_LOGD(TAG, "ENERGY marker hit but next bytes not 0C 14 (got %02X %02X) -> ignore", next0, next1);
        }
      }
    }

    // VOLUME: 0C 14 + 2?4? bytes -> in your frames it looks like 34 39 (2 bytes) right after 0C 14.
    // But you asked: "nach 0C 14 Gesamtvolumen" and earlier you used BCD 4 bytes *0.01.
    // Your sample: 0C 14 34 39 16 00 ... => looks like BCD "3934" (i.e. 39.34?) if 2 bytes.
    // We'll parse *2 bytes* BCD with factor 0.01 (=> 39.34 m³) to match the frame.
    if (!got_volume_ && buf[i] == 0x0C && buf[i + 1] == 0x14) {
      if (i + 2 + 2 <= n) {
        float v = decode_bcd_(buf, i + 2, 2) * 0.01f;
        if (!isnan(v)) {
          ESP_LOGI(TAG, "VOLUME parsed: %.2f m³", v);
          if (total_volume_) total_volume_->publish_state(v);
          got_volume_ = true;
        }
      }
    }

    // FLOW TEMP: 0A 5A + 2 BCD *0.1
    if (!got_tflow_ && buf[i] == 0x0A && buf[i + 1] == 0x5A) {
      if (i + 2 + 2 <= n) {
        float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
        if (!isnan(t)) {
          ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f °C", t);
          if (temp_flow_) temp_flow_->publish_state(t);
          got_tflow_ = true;
        }
      }
    }

    // RETURN TEMP: 0A 5E + 2 BCD *0.1
    if (!got_tret_ && buf[i] == 0x0A && buf[i + 1] == 0x5E) {
      if (i + 2 + 2 <= n) {
        float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
        if (!isnan(t)) {
          ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f °C", t);
          if (temp_return_) temp_return_->publish_state(t);
          got_tret_ = true;
        }
      }
    }

    // DELTA T: 0B 61 + 3 BCD *0.01
    if (!got_tdiff_ && buf[i] == 0x0B && buf[i + 1] == 0x61) {
      if (i + 2 + 3 <= n) {
        float dt = decode_bcd_(buf, i + 2, 3) * 0.01f;
        if (!isnan(dt)) {
          ESP_LOGI(TAG, "DELTA T parsed: %.2f K", dt);
          if (temp_diff_) temp_diff_->publish_state(dt);
          got_tdiff_ = true;
        }
      }
    }

    // METER TIME: 04 6D + 4 bytes CP32
    if (!got_time_ && buf[i] == 0x04 && buf[i + 1] == 0x6D) {
      if (i + 2 + 4 <= n) {
        std::string ts;
        if (decode_cp32_datetime_(buf, i + 2, ts)) {
          ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
          if (meter_time_) meter_time_->publish_state(ts);
          got_time_ = true;
        }
      }
    }
  }
}

// ============================================================
// ESPHome component methods
// ============================================================

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s --- BUILD %s %s", ULTRAMAXX_VERSION, __DATE__, __TIME__);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // wakeup at 2400 8N1 (as in your logs)
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

  // ---------------- RX stream ----------------
  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c)) break;

    last_rx_ms_ = now;

    // Wait for start of long frame 0x68; ignore noise (55 wake bytes etc.)
    if (!in_frame_) {
      if (c != 0x68) continue;
      in_frame_ = true;
      rx_buffer_.clear();
      expected_len_ = 0;
    }

    rx_buffer_.push_back(c);

    // Determine expected length once we have: 68 L L 68 ...
    if (expected_len_ == 0 && rx_buffer_.size() >= 4) {
      if (rx_buffer_[0] == 0x68 && rx_buffer_[3] == 0x68 && rx_buffer_[1] == rx_buffer_[2]) {
        // total frame bytes = 4(header) + L + 2(checksum+end)
        expected_len_ = 4 + rx_buffer_[1] + 2;
      }
    }

    // Parse opportunistically (your request: don't wait for full frame)
    this->parse_and_publish_(rx_buffer_);

    // If full frame arrived, reset for next cycle
    if (expected_len_ > 0 && rx_buffer_.size() >= expected_len_) {
      // Optional: sanity check last byte should be 0x16
      if (rx_buffer_.back() == 0x16) {
        // done
      }
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      break;
    }

    // Guard against runaway buffer
    if (rx_buffer_.size() > 256) {
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      break;
    }
  }

  // RX timeout (only meaningful if we're currently in a frame)
  if (state == UM_RX) {
    if (in_frame_) {
      if (now - last_rx_ms_ > 1200) {
        ESP_LOGW(TAG, "RX Timeout (in frame). buf=%u", (unsigned) rx_buffer_.size());
        in_frame_ = false;
        rx_buffer_.clear();
        expected_len_ = 0;
      }
    } else {
      // if we never saw header 0x68 after request
      if (now - state_ts_ > 2500) {
        ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
        rx_buffer_.clear();
        expected_len_ = 0;
      }
    }
  }

  // ---------------- State machine (TX) ----------------
  if (state == UM_WAKEUP) {
    // send 0x55 wake bytes periodically
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

  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // flush pending bytes
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    reset_parse_flags_();

    // SND_NKE
    const uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
    return;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {
    // REQ_UD2 with FCB toggling (as you had)
    const uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    const uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);

    const uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    // start RX window
    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    last_rx_ms_ = now;
    state = UM_RX;
    state_ts_ = now;
    return;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
