#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v5.20";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------- Decoder --------------------

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return NAN;
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

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    v |= ((uint32_t) data[start + i]) << (8 * i);
  }
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;

  // Dein bisheriges Mapping:
  //  b0 = Tag? / b1 = Monat? / b2 = Minute? / b3 = Stunde?
  // Ausgabe: "MM.TT HH:MM" (wie vorher bei dir)
  char buf[32];
  snprintf(buf, sizeof(buf), "%02u.%02u %02u:%02u",
           data[start + 1], data[start + 0],
           data[start + 3], data[start + 2]);
  out = buf;
  return true;
}

// -------------------- Publish guards --------------------

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = false;
  got_energy_ = false;
  got_volume_ = false;
  got_tflow_  = false;
  got_tret_   = false;
  got_tdiff_  = false;
  got_time_   = false;
}

// -------------------- Streaming Parser --------------------
// Scannt den aktuellen Buffer nach deinen Marker-Bytes und published sobald genügend Daten da sind.
// OHNE auf kompletten Frame zu warten.

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  const size_t n = buf.size();
  if (n < 6) return;

  // Optional: zum Debuggen einmalig pro RX-Fenster
  // ESP_LOGD(TAG, "parse_and_publish_: buf=%u", (unsigned) n);

  for (size_t i = 0; i + 1 < n; i++) {
    const uint8_t b0 = buf[i];
    const uint8_t b1 = buf[i + 1];

    // SERIAL: 0C 78 + 4 BCD
    if (!got_serial_ && b0 == 0x0C && b1 == 0x78) {
      if (i + 2 + 4 <= n) {
        float sn = decode_bcd_(buf, i + 2, 4);
        if (!isnan(sn)) {
          got_serial_ = true;
          ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
          if (serial_number_) serial_number_->publish_state(sn);
        }
      }
    }

    // ENERGY: 04 06 + 4 LE  -> *0.001 => MWh (wie du wolltest)
    if (!got_energy_ && b0 == 0x04 && b1 == 0x06) {
      if (i + 2 + 4 <= n) {
        uint32_t raw = decode_u_le_(buf, i + 2, 4);
        float e = ((float) raw) * 0.001f;
        got_energy_ = true;
        ESP_LOGI(TAG, "ENERGY parsed: %.3f MWh (raw=%u)", e, (unsigned) raw);
        if (total_energy_) total_energy_->publish_state(e);
      }
    }

    // VOLUME: 0C 14 + 4 BCD -> *0.01 => m³
    if (!got_volume_ && b0 == 0x0C && b1 == 0x14) {
      if (i + 2 + 4 <= n) {
        float v = decode_bcd_(buf, i + 2, 4) * 0.01f;
        if (!isnan(v)) {
          got_volume_ = true;
          ESP_LOGI(TAG, "VOLUME parsed: %.2f m3", v);
          if (total_volume_) total_volume_->publish_state(v);
        }
      }
    }

    // FLOW TEMP: 0A 5A + 2 BCD -> *0.1 => °C
    if (!got_tflow_ && b0 == 0x0A && b1 == 0x5A) {
      if (i + 2 + 2 <= n) {
        float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
        if (!isnan(t)) {
          got_tflow_ = true;
          ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f C", t);
          if (temp_flow_) temp_flow_->publish_state(t);
        }
      }
    }

    // RETURN TEMP: 0A 5E + 2 BCD -> *0.1 => °C
    if (!got_tret_ && b0 == 0x0A && b1 == 0x5E) {
      if (i + 2 + 2 <= n) {
        float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
        if (!isnan(t)) {
          got_tret_ = true;
          ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f C", t);
          if (temp_return_) temp_return_->publish_state(t);
        }
      }
    }

    // DELTA T: 0B 61 + 3 BCD -> *0.01 => K
    if (!got_tdiff_ && b0 == 0x0B && b1 == 0x61) {
      if (i + 2 + 3 <= n) {
        float dt = decode_bcd_(buf, i + 2, 3) * 0.01f;
        if (!isnan(dt)) {
          got_tdiff_ = true;
          ESP_LOGI(TAG, "DELTA T parsed: %.2f K", dt);
          if (temp_diff_) temp_diff_->publish_state(dt);
        }
      }
    }

    // METER TIME: 04 6D + 4 bytes (dein cp32 mapping)
    if (!got_time_ && b0 == 0x04 && b1 == 0x6D) {
      if (i + 2 + 4 <= n) {
        std::string ts;
        if (decode_cp32_datetime_(buf, i + 2, ts)) {
          got_time_ = true;
          ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
          if (meter_time_) meter_time_->publish_state(ts);
        }
      }
    }
  }
}

// -------------------- ESPHome hooks --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // Reset flags for this polling cycle
  this->reset_parse_flags_();

  // Wakeup mit 2400 8N1 (wie in deinen Logs)
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // RX state reset
  rx_buffer_.clear();
  in_frame_ = false;
  expected_len_ = 0;
  last_rx_ms_ = millis();

  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // -------------------- RX: nur im UM_RX lesen --------------------
  // (damit die 0x55-Wakeup-Orgie nicht den Buffer verstopft)
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (!this->read_byte(&c)) break;

      last_rx_ms_ = now;

      // Start eines Long-Frames finden
      if (!in_frame_) {
        if (c != 0x68) {
          continue;  // ignoriere alles bis 0x68
        }
        in_frame_ = true;
        rx_buffer_.clear();
        rx_buffer_.push_back(c);
        continue;
      }

      rx_buffer_.push_back(c);

      // Header auswerten sobald genug da ist: 68 L L 68
      if (rx_buffer_.size() == 4) {
        if (rx_buffer_[0] == 0x68 && rx_buffer_[3] == 0x68 && rx_buffer_[1] == rx_buffer_[2]) {
          expected_len_ = (size_t) rx_buffer_[1] + 6;  // 68 L L 68 + data(L) + CS + 16
          // ESP_LOGI(TAG, "Longframe header ok: L=%u => expected=%u", rx_buffer_[1], (unsigned) expected_len_);
        } else {
          // Kein gültiger Longframe-Header -> resync
          in_frame_ = false;
          rx_buffer_.clear();
          expected_len_ = 0;
        }
      }

      // Streaming-Parse (ohne vollständigen Frame abzuwarten)
      this->parse_and_publish_(rx_buffer_);

      // Fertig wenn komplette Länge erreicht ODER Endbyte 0x16 kommt und wir schon "groß genug" sind
      if (expected_len_ > 0 && rx_buffer_.size() >= expected_len_) {
        ESP_LOGI(TAG, "Frame complete (%u bytes)", (unsigned) rx_buffer_.size());
        in_frame_ = false;
        rx_buffer_.clear();
        expected_len_ = 0;
        break;
      } else if (c == 0x16 && rx_buffer_.size() > 40) {
        ESP_LOGI(TAG, "Frame end detected (%u bytes)", (unsigned) rx_buffer_.size());
        in_frame_ = false;
        rx_buffer_.clear();
        expected_len_ = 0;
        break;
      }

      // Buffer begrenzen (Safety)
      if (rx_buffer_.size() > 256) {
        ESP_LOGW(TAG, "RX buffer overflow safety reset (%u)", (unsigned) rx_buffer_.size());
        in_frame_ = false;
        rx_buffer_.clear();
        expected_len_ = 0;
        break;
      }
    }

    // Timeout wenn nach Start kein Byte mehr kommt
    if (in_frame_ && (now - last_rx_ms_ > 1200)) {
      ESP_LOGW(TAG, "RX Timeout (in_frame). buf=%u expected=%u", (unsigned) rx_buffer_.size(), (unsigned) expected_len_);
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      state = UM_IDLE;
    }

    // Timeout wenn gar kein Header je kam
    if (!in_frame_ && (now - last_rx_ms_ > 2500)) {
      ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
      rx_buffer_.clear();
      expected_len_ = 0;
      state = UM_IDLE;
    }
  }

  // -------------------- STATE MACHINE --------------------

  if (state == UM_WAKEUP) {
    // send 0x55 burst alle 15ms
    if (now - last_send_ > 15) {
      uint8_t buf[20];
      memset(buf, 0x55, sizeof(buf));
      this->write_array(buf, sizeof(buf));
      last_send_ = now;
    }

    // nach ~2.2s beenden
    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
    }
  }

  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // UART RX leer saugen (altes Zeug / 0x55 / Reste)
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;

    // SND_NKE: 10 40 FE CS 16 (CS=0x3E)
    const uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {
    // REQ_UD2 mit FCB toggling (0x5B/0x7B), CS = (C + A) & 0xFF bei Shortframe
    const uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    const uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);

    const uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    // RX starten
    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    last_rx_ms_ = now;

    state = UM_RX;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
