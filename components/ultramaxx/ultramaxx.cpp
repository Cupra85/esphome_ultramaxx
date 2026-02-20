#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

// Versionsstring (wird im setup() und bei READ START ausgegeben)
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v5.14";

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
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    v |= ((uint32_t) data[start + i]) << (8 * i);
  }
  return v;
}

// CP32 Time Stamp (EN 13757-3):
// bits: 0..5 minute, 6..10 hour, 11..15 day, 16..19 month, 20..26 year (0..99 -> 2000..2099),
// 27..29 weekday, 30 summer time, 31 invalid
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  if (start + 4 > data.size()) return false;

  uint32_t v = decode_u_le(data, start, 4);

  uint32_t minute = (v >> 0) & 0x3F;
  uint32_t hour   = (v >> 6) & 0x1F;
  uint32_t day    = (v >> 11) & 0x1F;
  uint32_t month  = (v >> 16) & 0x0F;
  uint32_t year   = (v >> 20) & 0x7F;   // 0..99 (üblich: +2000)

  // Plausibilität (sehr locker, manche Geräte sind falsch eingestellt)
  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) {
    // trotzdem ausgeben, aber markiert
  }

  char buf[32];
  // Ausgabeformat: YYYY-MM-DD HH:MM
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u",
           (unsigned) (2000 + year), (unsigned) month, (unsigned) day,
           (unsigned) hour, (unsigned) minute);
  out = buf;
  return true;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "%s", ULTRAMAXX_LOADED_LINE);
  ESP_LOGI(TAG, "UltraMaXX STARTED --- VERSION: %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // Wakeup-Protokoll bleibt unverändert
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

// Hilfsflags pro Zyklus: damit nicht zigmal published wird, wenn Muster mehrfach auftauchen
struct ParsedFlags {
  bool sn = false;
  bool energy = false;
  bool volume = false;
  bool flow = false;
  bool ret = false;
  bool dt = false;
  bool time = false;
};

void UltraMaXXComponent::loop() {
  uint32_t now = millis();

  // Flags pro RX-Phase
  static ParsedFlags parsed;
  static bool in_frame = false;

  // ----------------------------------------
  // RX STREAM (nur wenn wir wirklich im RX sind)
  // ----------------------------------------
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (!this->read_byte(&c)) break;

      last_rx_byte_ = now;

      // Solange wir noch keinen Longframe-Header gesehen haben, ignorieren wir alles bis 0x68
      if (!in_frame) {
        if (c != 0x68) {
          continue;
        }
        in_frame = true;
        rx_buffer_.clear();
      }

      rx_buffer_.push_back(c);

      // Buffer begrenzen (wir brauchen nur „Fenster“ um die Marker zu erkennen)
      // Longframe ist ~80 bytes, aber wir halten 200 als Sicherheitsfenster.
      if (rx_buffer_.size() > 200) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + (rx_buffer_.size() - 200));
      }

      // ----------------------------
      // Streaming Pattern Matching
      // ----------------------------
      const size_t n = rx_buffer_.size();

      // Wir prüfen jeweils nur, wenn genug bytes da sind
      // Marker+Payload: 0C78 + 4, 0406 + 4, 0C14 + 4, 0A5A +2, 0A5E +2, 0B61 +3, 046D +4

      // SERIAL: 0C 78 [4 bytes BCD]
      if (!parsed.sn && n >= 2 + 4) {
        for (size_t i = 0; i + 6 <= n; i++) {
          if (rx_buffer_[i] == 0x0C && rx_buffer_[i + 1] == 0x78) {
            float sn = decode_bcd(rx_buffer_, i + 2, 4);
            ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
            if (serial_number_) serial_number_->publish_state(sn);
            parsed.sn = true;
            break;
          }
        }
      }

      // ENERGY: 04 06 [4 bytes U32 LE] => *0.001 = MWh
      if (!parsed.energy && n >= 2 + 4) {
        for (size_t i = 0; i + 6 <= n; i++) {
          if (rx_buffer_[i] == 0x04 && rx_buffer_[i + 1] == 0x06) {
            uint32_t raw = decode_u_le(rx_buffer_, i + 2, 4);
            float e = raw * 0.001f;
            ESP_LOGI(TAG, "ENERGY parsed: %.3f MWh (raw=%u)", e, (unsigned) raw);
            if (total_energy_) total_energy_->publish_state(e);
            parsed.energy = true;
            break;
          }
        }
      }

      // VOLUME: 0C 14 [4 bytes BCD] => *0.01 = m³
      if (!parsed.volume && n >= 2 + 4) {
        for (size_t i = 0; i + 6 <= n; i++) {
          if (rx_buffer_[i] == 0x0C && rx_buffer_[i + 1] == 0x14) {
            float v = decode_bcd(rx_buffer_, i + 2, 4) * 0.01f;
            ESP_LOGI(TAG, "VOLUME parsed: %.2f m³", v);
            if (total_volume_) total_volume_->publish_state(v);
            parsed.volume = true;
            break;
          }
        }
      }

      // FLOW TEMP: 0A 5A [2 bytes BCD] => *0.1 °C
      if (!parsed.flow && n >= 2 + 2) {
        for (size_t i = 0; i + 4 <= n; i++) {
          if (rx_buffer_[i] == 0x0A && rx_buffer_[i + 1] == 0x5A) {
            float t = decode_bcd(rx_buffer_, i + 2, 2) * 0.1f;
            ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f °C", t);
            if (temp_flow_) temp_flow_->publish_state(t);
            parsed.flow = true;
            break;
          }
        }
      }

      // RETURN TEMP: 0A 5E [2 bytes BCD] => *0.1 °C
      if (!parsed.ret && n >= 2 + 2) {
        for (size_t i = 0; i + 4 <= n; i++) {
          if (rx_buffer_[i] == 0x0A && rx_buffer_[i + 1] == 0x5E) {
            float t = decode_bcd(rx_buffer_, i + 2, 2) * 0.1f;
            ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f °C", t);
            if (temp_return_) temp_return_->publish_state(t);
            parsed.ret = true;
            break;
          }
        }
      }

      // DELTA T: 0B 61 [3 bytes BCD] => *0.01 K
      if (!parsed.dt && n >= 2 + 3) {
        for (size_t i = 0; i + 5 <= n; i++) {
          if (rx_buffer_[i] == 0x0B && rx_buffer_[i + 1] == 0x61) {
            float dt = decode_bcd(rx_buffer_, i + 2, 3) * 0.01f;
            ESP_LOGI(TAG, "DELTA T parsed: %.2f K", dt);
            if (temp_diff_) temp_diff_->publish_state(dt);
            parsed.dt = true;
            break;
          }
        }
      }

      // METER TIME: 04 6D [4 bytes CP32]
      if (!parsed.time && n >= 2 + 4) {
        for (size_t i = 0; i + 6 <= n; i++) {
          if (rx_buffer_[i] == 0x04 && rx_buffer_[i + 1] == 0x6D) {
            std::string ts;
            if (decode_cp32_datetime_(rx_buffer_, i + 2, ts)) {
              ESP_LOGI(TAG, "METER TIME parsed: %s", ts.c_str());
              if (meter_time_) meter_time_->publish_state(ts);
              parsed.time = true;
              break;
            }
          }
        }
      }

      // Frame-Ende
      if (c == 0x16) {
        ESP_LOGD(TAG, "Frame end 0x16 detected. buf=%u", (unsigned) rx_buffer_.size());
        // Zyklus beenden
        rx_buffer_.clear();
        in_frame = false;
        state = UM_IDLE;
        ESP_LOGI(TAG, "Update finished.");
        break;
      }
    }

    // Timeout, wenn wir zwar RX sind, aber nie einen Header (0x68) gesehen haben
    if (state == UM_RX && (now - state_ts_ > 8000)) {
      ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
      rx_buffer_.clear();
      in_frame = false;
      state = UM_IDLE;
    }

    // Wenn wir im Frame sind, aber nichts mehr kommt (Stille), Zyklus beenden
    if (state == UM_RX && in_frame && (now - last_rx_byte_ > 1200)) {
      ESP_LOGI(TAG, "RX idle gap -> finish. buf=%u", (unsigned) rx_buffer_.size());
      rx_buffer_.clear();
      in_frame = false;
      state = UM_IDLE;
    }
  }

  // ----------------------------------------
  // STATE MACHINE (Wakeup/Request unverändert)
  // ----------------------------------------
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

  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // RX-Puffer leeren (Echo/Wakeup-Reste)
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();

    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);

    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    // RX vorbereiten
    fcb_toggle_ = !fcb_toggle_;
    rx_buffer_.clear();
    parsed = ParsedFlags{};
    in_frame = false;
    last_rx_byte_ = now;
    state = UM_RX;
    state_ts_ = now;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
