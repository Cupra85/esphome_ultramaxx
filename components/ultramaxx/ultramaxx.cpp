#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v5.17";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------- Decoder --------------------

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size())
    return 0.0f;

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
  if (start + len > data.size())
    return 0;

  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    v |= ((uint32_t) data[start + i]) << (8 * i);
  }
  return v;
}

// CP32 DateTime (EN13757-3 typisch): minute/hour/day/month + year bits
// Wir decodieren robust: Minute/Hour/Day/Month sicher; Year heuristisch.
// Wenn das nicht plausibel ist, geben wir trotzdem ein Format + Raw aus.
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size())
    return false;

  const uint8_t b0 = data[start + 0];
  const uint8_t b1 = data[start + 1];
  const uint8_t b2 = data[start + 2];
  const uint8_t b3 = data[start + 3];

  const uint8_t minute = b0 & 0x3F;
  const uint8_t hour   = b1 & 0x1F;
  const uint8_t day    = b2 & 0x1F;
  const uint8_t month  = b3 & 0x0F;

  // Heuristik für Jahr (verschiedene Zähler kodieren es leicht anders)
  // Variante A: (b3 high nibble)*10 + (b2 high bits)
  const uint8_t yA = (uint8_t)(((b3 >> 4) & 0x0F) * 10 + ((b2 >> 5) & 0x07));
  uint16_t year = 2000 + yA;

  bool plausible = true;
  if (minute > 59 || hour > 23 || day == 0 || day > 31 || month == 0 || month > 12)
    plausible = false;

  char buf[64];
  if (plausible) {
    snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u", year, month, day, hour, minute);
    out = buf;
    return true;
  }

  // Fallback: Raw anzeigen
  snprintf(buf, sizeof(buf), "RAW_CP32_%02X%02X%02X%02X", b0, b1, b2, b3);
  out = buf;
  return true;
}

// -------------------- Parser helpers --------------------

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = false;
  got_energy_ = false;
  got_volume_ = false;
  got_tflow_  = false;
  got_tret_   = false;
  got_tdiff_  = false;
  got_time_   = false;
}

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  if (buf.size() < 10)
    return;

  // Wir suchen im gesamten Buffer nach den "Marker"-Sequenzen
  // und publishen pro Marker nur 1x pro Zyklus (Flags).
  for (size_t i = 0; i + 6 <= buf.size(); i++) {

    // 0C 78 + 4B BCD -> Seriennummer
    if (!got_serial_ && i + 2 + 4 <= buf.size() && buf[i] == 0x0C && buf[i + 1] == 0x78) {
      float sn = decode_bcd_(buf, i + 2, 4);
      got_serial_ = true;
      ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
      if (serial_number_) serial_number_->publish_state(sn);
      continue;
    }

    // 04 06 + 4B U32 LE -> Energie (MWh = raw*0.001)
    if (!got_energy_ && i + 2 + 4 <= buf.size() && buf[i] == 0x04 && buf[i + 1] == 0x06) {
      uint32_t raw = decode_u_le_(buf, i + 2, 4);
      float e = raw * 0.001f;
      got_energy_ = true;
      ESP_LOGI(TAG, "ENERGY parsed: %.3f MWh (raw=%u)", e, raw);
      if (total_energy_) total_energy_->publish_state(e);
      continue;
    }

    // 0C 14 + 4B BCD -> Volumen (m³ = bcd*0.01)
    if (!got_volume_ && i + 2 + 4 <= buf.size() && buf[i] == 0x0C && buf[i + 1] == 0x14) {
      float v = decode_bcd_(buf, i + 2, 4) * 0.01f;
      got_volume_ = true;
      ESP_LOGI(TAG, "VOLUME parsed: %.2f m3", v);
      if (total_volume_) total_volume_->publish_state(v);
      continue;
    }

    // 0A 5A + 2B BCD -> Vorlauf (*0.1)
    if (!got_tflow_ && i + 2 + 2 <= buf.size() && buf[i] == 0x0A && buf[i + 1] == 0x5A) {
      float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
      got_tflow_ = true;
      ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f C", t);
      if (temp_flow_) temp_flow_->publish_state(t);
      continue;
    }

    // 0A 5E + 2B BCD -> Rücklauf (*0.1)
    if (!got_tret_ && i + 2 + 2 <= buf.size() && buf[i] == 0x0A && buf[i + 1] == 0x5E) {
      float t = decode_bcd_(buf, i + 2, 2) * 0.1f;
      got_tret_ = true;
      ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f C", t);
      if (temp_return_) temp_return_->publish_state(t);
      continue;
    }

    // 0B 61 + 3B BCD -> DeltaT (*0.01)
    if (!got_tdiff_ && i + 2 + 3 <= buf.size() && buf[i] == 0x0B && buf[i + 1] == 0x61) {
      float dt = decode_bcd_(buf, i + 2, 3) * 0.01f;
      got_tdiff_ = true;
      ESP_LOGI(TAG, "DELTA T parsed: %.2f K", dt);
      if (temp_diff_) temp_diff_->publish_state(dt);
      continue;
    }

    // 04 6D + 4B CP32 -> time
    if (!got_time_ && i + 2 + 4 <= buf.size() && buf[i] == 0x04 && buf[i + 1] == 0x6D) {
      std::string ts;
      if (decode_cp32_datetime_(buf, i + 2, ts)) {
        got_time_ = true;
        ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
        if (meter_time_) meter_time_->publish_state(ts);
      }
      continue;
    }
  }
}

// -------------------- ESPHome hooks --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "%s", ULTRAMAXX_LOADED_LINE);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // UART auf 2400 8N1 für Wakeup
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // RX/Flags für neuen Zyklus
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

  // ---------------- RX STREAM (nur wenn wir im RX-State sind) ----------------
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (!this->read_byte(&c))
        break;

      last_rx_ms_ = now;

      // Start finden: erst ab 0x68 anfangen zu sammeln
      if (!in_frame_) {
        if (c != 0x68)
          continue;
        in_frame_ = true;
        rx_buffer_.clear();
        expected_len_ = 0;
      }

      rx_buffer_.push_back(c);

      // Erwartete Gesamtlänge bestimmen, sobald wir L kennen: 68 L L 68 ...
      if (rx_buffer_.size() == 4) {
        if (rx_buffer_[0] == 0x68 && rx_buffer_[3] == 0x68) {
          uint8_t L1 = rx_buffer_[1];
          uint8_t L2 = rx_buffer_[2];
          if (L1 == L2 && L1 >= 3) {
            expected_len_ = (size_t) L1 + 6;  // EN13757: total = L + 6
            ESP_LOGD(TAG, "Frame header ok. L=%u => expected_len=%u", (unsigned)L1, (unsigned)expected_len_);
          } else {
            // Header unplausibel -> neu syncen
            ESP_LOGW(TAG, "Bad length bytes: %02X %02X. Resync.", L1, L2);
            in_frame_ = false;
            rx_buffer_.clear();
            expected_len_ = 0;
            continue;
          }
        } else {
          // Kein 68 L L 68 -> resync
          ESP_LOGW(TAG, "Bad header sequence. Resync.");
          in_frame_ = false;
          rx_buffer_.clear();
          expected_len_ = 0;
          continue;
        }
      }

      // STREAMING PARSE: sobald genug Bytes da sind, immer wieder versuchen Marker zu finden
      // (damit du NICHT auf komplettes Telegram warten musst)
      if (rx_buffer_.size() >= 12) {
        this->parse_and_publish_(rx_buffer_);
      }

      // Frame komplett?
      if (expected_len_ > 0 && rx_buffer_.size() >= expected_len_) {
        // typischerweise endet es mit 0x16; wenn nicht, trotzdem parse versuchen
        if (!rx_buffer_.empty() && rx_buffer_.back() != 0x16) {
          ESP_LOGW(TAG, "Frame reached expected_len but no 0x16 at end (last=%02X).", rx_buffer_.back());
        }
        ESP_LOGI(TAG, "Frame complete (%u bytes).", (unsigned)rx_buffer_.size());
        this->parse_and_publish_(rx_buffer_);
        rx_buffer_.clear();
        in_frame_ = false;
        expected_len_ = 0;
        state = UM_IDLE;
        return;
      }
    }

    // Silence timeout: wenn wir im Frame sind und nix mehr kommt, trotzdem auswerten
    if (in_frame_ && (now - last_rx_ms_ > 1200)) {
      ESP_LOGW(TAG, "RX silence timeout. buf=%u -> parse partial", (unsigned)rx_buffer_.size());
      this->parse_and_publish_(rx_buffer_);
      rx_buffer_.clear();
      in_frame_ = false;
      expected_len_ = 0;
      state = UM_IDLE;
      return;
    }
  }

  // ---------------- State machine: Wakeup/Request bleibt unverändert ----------------

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

    // RX leeren (damit keine 0x55-Reste stören)
    uint8_t d;
    while (this->available())
      this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;

    // SND_NKE (unverändert)
    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {
    // REQ_UD2 (FCB toggeln)
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (uint8_t)((ctrl + 0xFE) & 0xFF);
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    // Vor dem Request nochmal RX leeren (wichtig!)
    uint8_t d;
    while (this->available())
      this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    reset_parse_flags_();

    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    // RX starten
    last_rx_ms_ = now;
    state = UM_RX;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
