#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v5.13";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------- Decoder helpers --------------------

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= ((uint32_t) data[start + i]) << (8 * i);
  return v;
}

// CP32 “Date and Time” (VIF 0x6D, DIF 0x04) – siehe EN13757-3 Annex A.
// Byte0: minute (bits 0..5)
// Byte1: hour (bits 0..4), HY bits (5..6), SU bit (7)
// Byte2: day (bits 0..4), year low 3 bits (5..7)
// Byte3: month (bits 0..3), year high 4 bits (4..7)
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;

  uint8_t b0 = data[start + 0];
  uint8_t b1 = data[start + 1];
  uint8_t b2 = data[start + 2];
  uint8_t b3 = data[start + 3];

  // IV (invalid) ist typischerweise Bit7 in b0 bei manchen Implementierungen – wir checken konservativ:
  // Wenn minute > 59 oder hour > 23 -> verwerfen.
  uint8_t minute = b0 & 0x3F;
  uint8_t hour   = b1 & 0x1F;
  uint8_t day    = b2 & 0x1F;
  uint8_t month  = b3 & 0x0F;

  uint8_t year_lo = (b2 >> 5) & 0x07;
  uint8_t year_hi = (b3 >> 4) & 0x0F;
  uint16_t year_2d = (uint16_t) (year_lo | (year_hi << 3));  // 0..99 typ.
  uint8_t hy = (b1 >> 5) & 0x03;  // century hint

  if (minute > 59 || hour > 23 || day == 0 || day > 31 || month == 0 || month > 12) return false;

  uint16_t year = 2000 + year_2d;
  // falls HY was anderes liefert, trotzdem robust bleiben:
  if (hy == 0) year = 1900 + year_2d;

  char buf[32];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u", year, month, day, hour, minute);
  out = buf;
  return true;
}

// -------------------- Parser logic --------------------

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = got_energy_ = got_volume_ = false;
  got_tflow_  = got_tret_   = got_tdiff_  = false;
  got_time_   = false;
}

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  // Wir suchen nach den von dir gewünschten Marker-Paaren:
  // 0C 78 -> Serial (4B BCD)
  // 04 06 -> Energy (4B u32 LE) *0.001 => MWh
  // 0C 14 -> Volume (4B BCD) *0.01 => m³
  // 0A 5A -> Flow temp (2B BCD) *0.1 => °C
  // 0A 5E -> Return temp (2B BCD) *0.1 => °C
  // 0B 61 -> DeltaT (3B u24 LE) *0.01 => K
  // 04 6D -> Meter time (4B CP32)

  // Scan whole buffer (klein genug), publish nur einmal pro Frame/Update.
  for (size_t i = 0; i + 2 < buf.size(); i++) {
    // SERIAL
    if (!got_serial_ && i + 6 <= buf.size() && buf[i] == 0x0C && buf[i + 1] == 0x78) {
      float sn = decode_bcd_(buf, i + 2, 4);
      got_serial_ = true;
      ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
      if (serial_number_) serial_number_->publish_state(sn);
      continue;
    }

    // ENERGY
    if (!got_energy_ && i + 6 <= buf.size() && buf[i] == 0x04 && buf[i + 1] == 0x06) {
      uint32_t raw = decode_u_le_(buf, i + 2, 4);
      float e_mwh = raw * 0.001f;
      got_energy_ = true;
      ESP_LOGI(TAG, "ENERGY parsed: %.3f MWh (raw=%u)", e_mwh, raw);
      if (total_energy_) total_energy_->publish_state(e_mwh);
      continue;
    }

    // VOLUME
    if (!got_volume_ && i + 6 <= buf.size() && buf[i] == 0x0C && buf[i + 1] == 0x14) {
      float v = decode_bcd_(buf, i + 2, 4) * 0.01f;
      got_volume_ = true;
      ESP_LOGI(TAG, "VOLUME parsed: %.2f m³", v);
      if (total_volume_) total_volume_->publish_state(v);
      continue;
    }

    // FLOW TEMP
    if (!got_tflow_ && i + 4 <= buf.size() && buf[i] == 0x0A && buf[i + 1] == 0x5A) {
      float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
      got_tflow_ = true;
      ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f °C", t);
      if (temp_flow_) temp_flow_->publish_state(t);
      continue;
    }

    // RETURN TEMP
    if (!got_tret_ && i + 4 <= buf.size() && buf[i] == 0x0A && buf[i + 1] == 0x5E) {
      float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
      got_tret_ = true;
      ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f °C", t);
      if (temp_return_) temp_return_->publish_state(t);
      continue;
    }

    // DELTA T (3 bytes u24 LE, *0.01)
    if (!got_tdiff_ && i + 5 <= buf.size() && buf[i] == 0x0B && buf[i + 1] == 0x61) {
      uint32_t raw = decode_u_le_(buf, i + 2, 3);
      float dt = raw * 0.01f;
      got_tdiff_ = true;
      ESP_LOGI(TAG, "DELTA T parsed: %.2f K (raw=%u)", dt, raw);
      if (temp_diff_) temp_diff_->publish_state(dt);
      continue;
    }

    // METER TIME (CP32)
    if (!got_time_ && i + 6 <= buf.size() && buf[i] == 0x04 && buf[i + 1] == 0x6D) {
      std::string ts;
      if (decode_cp32_datetime_(buf, i + 2, ts)) {
        got_time_ = true;
        ESP_LOGI(TAG, "METER TIME parsed: %s", ts.c_str());
        if (meter_time_) meter_time_->publish_state(ts);
      }
      continue;
    }
  }
}

// -------------------- Component lifecycle --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX Parser v6 LOADED");
  ESP_LOGI(TAG, "UltraMaXX STARTED --- VERSION: %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // UART wakeup mode: 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // reset RX state
  rx_buffer_.clear();
  in_frame_ = false;
  expected_len_ = 0;
  reset_parse_flags_();

  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // -------------------- RX capture (nur in UM_RX) --------------------
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (!this->read_byte(&c)) break;

      last_rx_ms_ = now;

      // Frame-Start finden: 68 L L 68
      if (!in_frame_) {
        if (c != 0x68) {
          // Ignoriere alles vor Start (z.B. 0x55 Echo)
          continue;
        }
        rx_buffer_.clear();
        rx_buffer_.push_back(c);
        continue;
      } else {
        rx_buffer_.push_back(c);
      }

      // Wenn wir gerade erst begonnen haben, prüfen wir Header sobald genug Bytes da sind
      if (!in_frame_ && rx_buffer_.size() >= 4) {
        // unreachable wegen Logik oben, aber lassen wir’s robust
      }

      // Wenn wir noch nicht “in_frame_” sind, müssen wir Header komplettieren:
      if (!in_frame_ && rx_buffer_.size() >= 4) {
        // nicht genutzt
      }
    }

    // Wir brauchen eine kleine Header-State-Maschine:
    // Falls wir bei Start 0x68 gepusht haben, müssen wir weiter Bytes lesen,
    // darum ergänzen wir hier (falls rx_buffer_ nur Start hat) ebenfalls:
    while (this->available() && rx_buffer_.size() > 0 && rx_buffer_[0] == 0x68 && !in_frame_) {
      uint8_t c;
      if (!this->read_byte(&c)) break;
      last_rx_ms_ = now;
      rx_buffer_.push_back(c);

      if (rx_buffer_.size() >= 4) {
        // 68 L L 68
        uint8_t L1 = rx_buffer_[1];
        uint8_t L2 = rx_buffer_[2];
        if (L1 == L2 && rx_buffer_[3] == 0x68) {
          expected_len_ = (size_t) L1 + 6;  // siehe Frame extraction Logik
          in_frame_ = true;

          // jetzt sind wir in einem gültigen Frame – Flags resetten
          reset_parse_flags_();

          ESP_LOGD(TAG, "Frame header OK: L=%u expected_len=%u", (unsigned) L1, (unsigned) expected_len_);
        } else {
          // falscher Start – wieder auf Startsuche
          rx_buffer_.clear();
          in_frame_ = false;
          expected_len_ = 0;
        }
      }
    }

    // Falls wir in_frame_ sind, sammeln wir weiter
    if (in_frame_) {
      while (this->available()) {
        uint8_t c;
        if (!this->read_byte(&c)) break;
        last_rx_ms_ = now;

        rx_buffer_.push_back(c);

        // “early publish”: sofort aus dem Buffer extrahieren
        this->parse_and_publish_(rx_buffer_);

        // Buffer begrenzen (sicher)
        if (rx_buffer_.size() > 256) {
          rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + (rx_buffer_.size() - 256));
        }

        // Wenn wir erwartete Länge erreicht haben: fertig (auch wenn 0x16 evtl. später wäre)
        if (expected_len_ > 0 && rx_buffer_.size() >= expected_len_) {
          ESP_LOGI(TAG, "Frame complete (%u bytes).", (unsigned) rx_buffer_.size());
          // final scan (falls etwas erst jetzt vollständig ist)
          this->parse_and_publish_(rx_buffer_);
          rx_buffer_.clear();
          in_frame_ = false;
          expected_len_ = 0;
          state = UM_IDLE;
          break;
        }

        // Optional: wenn echtes Endbyte kommt und wir schon "groß genug" sind
        if (c == 0x16 && rx_buffer_.size() > 20) {
          ESP_LOGI(TAG, "Frame end 0x16 (%u bytes).", (unsigned) rx_buffer_.size());
          this->parse_and_publish_(rx_buffer_);
          rx_buffer_.clear();
          in_frame_ = false;
          expected_len_ = 0;
          state = UM_IDLE;
          break;
        }
      }

      // Timeout: wenn keine neuen Bytes kommen
      if (state == UM_RX && (now - state_ts_) > 8000) {
        ESP_LOGW(TAG, "RX Timeout (no full frame). buf=%u in_frame=%d exp=%u",
                 (unsigned) rx_buffer_.size(), (int) in_frame_, (unsigned) expected_len_);
        rx_buffer_.clear();
        in_frame_ = false;
        expected_len_ = 0;
        state = UM_IDLE;
      }
    } else {
      // wir haben Start gesehen aber Header nicht komplett -> kurzer Timeout
      if (state == UM_RX && (now - state_ts_) > 8000) {
        ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
        rx_buffer_.clear();
        expected_len_ = 0;
        in_frame_ = false;
        state = UM_IDLE;
      }
    }
  }

  // -------------------- State machine (Wakeup unverändert) --------------------

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

    // UART RX leeren (Echo vom Wakeup / Altlasten)
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    reset_parse_flags_();

    // SND_NKE
    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {
    // REQ_UD2 (toggle FCB)
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    // Vor Senden nochmal leeren
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    // RX vorbereiten
    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    reset_parse_flags_();

    state = UM_RX;
    state_ts_ = now;
    last_rx_ms_ = now;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
