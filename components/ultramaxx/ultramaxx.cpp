#include "ultramaxx.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v6.2";

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

// CP32 (EN13757/OMS-style): 4 bytes = 2 bytes time (CP16) + 2 bytes date (CP16), little-endian.
// time: minute = bits 0..5, hour = bits 8..12
// date: day = bits 0..4, month = bits 5..8, year = bits 9..15 + 2000
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;

  uint16_t t = (uint16_t) data[start] | ((uint16_t) data[start + 1] << 8);
  uint16_t d = (uint16_t) data[start + 2] | ((uint16_t) data[start + 3] << 8);

  uint8_t minute = t & 0x3F;
  uint8_t hour = (t >> 8) & 0x1F;

  uint8_t day = d & 0x1F;
  uint8_t month = (d >> 5) & 0x0F;
  uint16_t year = 2000 + ((d >> 9) & 0x7F);

  if (minute > 59 || hour > 23 || day == 0 || day > 31 || month == 0 || month > 12) {
    // Fallback: raw hex, damit du zumindest was siehst
    char buf[32];
    snprintf(buf, sizeof(buf), "raw:%02X%02X%02X%02X", data[start], data[start + 1], data[start + 2], data[start + 3]);
    out = buf;
    return true;
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "%02u.%02u.%04u %02u:%02u", day, month, year, hour, minute);
  out = buf;
  return true;
}

// -------------------- Parser helpers --------------------

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = false;
  got_energy_ = false;
  got_volume_ = false;
  got_tflow_ = false;
  got_tret_ = false;
  got_tdiff_ = false;
  got_time_ = false;
}

// Stream-scan: findet Marker überall im Buffer, sobald genug Payload da ist.
// Marker-Regeln laut deinem Wunsch:
// 0C 78 -> Seriennummer (4 BCD bytes)
// 04 06 -> Gesamtenergie (4 LE bytes) -> kWh
// 0C 14 -> Gesamtvolumen (4 BCD bytes) *0.01 m³
// 0A 5A -> Vorlauf (2 BCD bytes) *0.1 °C
// 0A 5E -> Rücklauf (2 BCD bytes) *0.1 °C
// 0B 61 -> ΔT (3 BCD bytes) *0.01 K
// 04 6D -> Zählerzeit (4 bytes CP32)
void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  const size_t n = buf.size();
  if (n < 4) return;

  // Debug: wenn du sehen willst, ob wir überhaupt scannen, kurz aktivieren:
  // ESP_LOGD(TAG, "parse_and_publish_ scan n=%u", (unsigned) n);

  for (size_t i = 0; i + 1 < n; i++) {
    const uint8_t a = buf[i];
    const uint8_t b = buf[i + 1];

    // SERIAL: 0C 78 + 4 bytes BCD
    if (!got_serial_ && a == 0x0C && b == 0x78 && i + 2 + 4 <= n) {
      float sn = this->decode_bcd_(buf, i + 2, 4);
      if (!isnan(sn)) {
        ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
        if (serial_number_) serial_number_->publish_state(sn);
        got_serial_ = true;
      }
    }

    // ENERGY: 04 06 + 4 bytes LE -> kWh (raw)
    if (!got_energy_ && a == 0x04 && b == 0x06 && i + 2 + 4 <= n) {
      uint32_t raw = this->decode_u_le_(buf, i + 2, 4);
      float kwh = (float) raw;  // raw ist bei dir offenbar kWh
      ESP_LOGI(TAG, "ENERGY parsed: %.0f kWh (raw=%u)", kwh, (unsigned) raw);
      if (total_energy_) total_energy_->publish_state(kwh);
      got_energy_ = true;
    }

    // VOLUME: 0C 14 + 4 bytes BCD * 0.01 -> m³
    if (!got_volume_ && a == 0x0C && b == 0x14 && i + 2 + 4 <= n) {
      float v = this->decode_bcd_(buf, i + 2, 4) * 0.01f;
      if (!isnan(v)) {
        ESP_LOGI(TAG, "VOLUME parsed: %.2f m³", v);
        if (total_volume_) total_volume_->publish_state(v);
        got_volume_ = true;
      }
    }

    // FLOW TEMP: 0A 5A + 2 bytes BCD * 0.1 -> °C
    if (!got_tflow_ && a == 0x0A && b == 0x5A && i + 2 + 2 <= n) {
      float t = this->decode_bcd_(buf, i + 2, 2) * 0.1f;
      if (!isnan(t)) {
        ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f °C", t);
        if (temp_flow_) temp_flow_->publish_state(t);
        got_tflow_ = true;
      }
    }

    // RETURN TEMP: 0A 5E + 2 bytes BCD * 0.1 -> °C
    if (!got_tret_ && a == 0x0A && b == 0x5E && i + 2 + 2 <= n) {
      float t = this->decode_bcd_(buf, i + 2, 2) * 0.1f;
      if (!isnan(t)) {
        ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f °C", t);
        if (temp_return_) temp_return_->publish_state(t);
        got_tret_ = true;
      }
    }

    // DELTA T: 0B 61 + 3 bytes BCD * 0.01 -> K
    if (!got_tdiff_ && a == 0x0B && b == 0x61 && i + 2 + 3 <= n) {
      float dt = this->decode_bcd_(buf, i + 2, 3) * 0.01f;
      if (!isnan(dt)) {
        ESP_LOGI(TAG, "DELTA T parsed: %.2f K", dt);
        if (temp_diff_) temp_diff_->publish_state(dt);
        got_tdiff_ = true;
      }
    }

    // METER TIME: 04 6D + 4 bytes CP32 -> Text
    if (!got_time_ && a == 0x04 && b == 0x6D && i + 2 + 4 <= n) {
      std::string ts;
      if (this->decode_cp32_datetime_(buf, i + 2, ts)) {
        ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
        if (meter_time_) meter_time_->publish_state(ts);
        got_time_ = true;
      }
    }
  }
}

// -------------------- ESPHome lifecycle --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX Parser v6.0 LOADED");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // Wake-up Phase: 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  in_frame_ = false;
  expected_len_ = 0;
  last_rx_ms_ = 0;

  this->reset_parse_flags_();

  wake_start_ = millis();
  last_send_ = 0;
  state_ts_ = wake_start_;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // -------------------- RX: stream collect & parse --------------------
  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c)) break;

    last_rx_ms_ = now;

    // Wir ignorieren alles bis zur Longframe-Startkennung 0x68.
    if (!in_frame_) {
      if (c != 0x68) continue;
      in_frame_ = true;
      rx_buffer_.clear();
      expected_len_ = 0;
    }

    rx_buffer_.push_back(c);

    // Sobald wir wenigstens Header haben, expected_len berechnen:
    // Longframe: 68 L L 68 ... DATA ... CS 16
    if (expected_len_ == 0 && rx_buffer_.size() >= 4) {
      if (rx_buffer_[0] == 0x68 && rx_buffer_[3] == 0x68) {
        uint8_t l1 = rx_buffer_[1];
        uint8_t l2 = rx_buffer_[2];
        if (l1 == l2) {
          // Gesamtlänge inkl. 68 L L 68 + (L bytes) + CS + 16
          expected_len_ = 4 + l1 + 2;
        }
      }
    }

    // Stream-Scan IMMER, sobald genug Bytes da sind:
    this->parse_and_publish_(rx_buffer_);

    // Frame-Ende: wenn erwartete Länge erreicht oder 0x16 am Ende (als Sicherheitsnetz)
    const bool len_complete = (expected_len_ > 0 && rx_buffer_.size() >= expected_len_);
    const bool ends_with_16 = (!rx_buffer_.empty() && rx_buffer_.back() == 0x16 && rx_buffer_.size() >= 6);

    if (len_complete || ends_with_16) {
      // Frame abgeschlossen -> für nächsten Zyklus wieder sauber
      rx_buffer_.clear();
      in_frame_ = false;
      expected_len_ = 0;
      // Flags NICHT resetten: pro update-Zyklus nur einmal publishen
    }

    // Schutz: Buffer nicht unendlich wachsen lassen
    if (rx_buffer_.size() > 300) {
      ESP_LOGW(TAG, "RX buffer overflow guard, clearing");
      rx_buffer_.clear();
      in_frame_ = false;
      expected_len_ = 0;
    }
  }

  // -------------------- STATE MACHINE (TX) --------------------

  if (state == UM_WAKEUP) {
    // 0x55 stream for ~2.2s
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

    // UART RX FIFO leeren (wichtig, damit kein 0x55-Müll reinfunkt)
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;

    // SND_NKE
    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
    return;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {
    // REQ_UD2 mit FCB toggling
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();
    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;
    state = UM_RX;
    state_ts_ = now;
    return;
  }

  if (state == UM_RX) {
    // Timeout: wenn wir gar keinen Header gesehen haben
    if (!in_frame_ && (now - state_ts_ > 2500)) {
      ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
      rx_buffer_.clear();
      expected_len_ = 0;
      state = UM_IDLE;
      return;
    }

    // Timeout: wenn Frame angefangen hat, aber dann nichts mehr kommt
    if (in_frame_ && (now - last_rx_ms_ > 1200)) {
      ESP_LOGW(TAG, "RX Timeout (stalled). buf=%u", (unsigned) rx_buffer_.size());
      rx_buffer_.clear();
      in_frame_ = false;
      expected_len_ = 0;
      state = UM_IDLE;
      return;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
