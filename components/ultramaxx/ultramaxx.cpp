#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// --- Frame-Assembler (ein Gerät / eine Instanz) ---
static bool in_frame = false;
static int expected_total = -1;

// Publish-Guards (damit nicht mehrfach pro Read gepublished wird)
static bool got_serial = false;
static bool got_energy = false;
static bool got_volume = false;
static bool got_pwr = false;       // optional, falls später ergänzt
static bool got_tflow = false;
static bool got_tret = false;
static bool got_tdiff = false;
static bool got_time = false;

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0.0f;
  float value = 0.0f;
  float mul = 1.0f;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10.0f;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10.0f;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size() || len == 0 || len > 4) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) {
    v |= ((uint32_t) data[start + i]) << (8 * i);
  }
  return v;
}

// CP32-ish decode (EN13757). Falls Werte unplausibel -> false.
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  if (start + 4 > data.size()) return false;
  uint32_t v = decode_u_le(data, start, 4);

  int minute = (v >> 0) & 0x3F;   // 0..59
  int hour   = (v >> 6) & 0x1F;   // 0..23
  int day    = (v >> 11) & 0x1F;  // 1..31
  int month  = (v >> 16) & 0x0F;  // 1..12
  int year   = (v >> 20) & 0x7F;  // 0..127  (interpretiert als 2000+year)

  // Plausibilitätscheck
  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) return false;

  int full_year = 2000 + year;
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d", full_year, month, day, hour, minute);
  out = buf;
  return true;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Reset per cycle
  rx_buffer_.clear();
  in_frame = false;
  expected_total = -1;

  got_serial = got_energy = got_volume = got_pwr = got_tflow = got_tret = got_tdiff = got_time = false;

  // UART Wakeup: 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // WICHTIG: RX leer machen (alte Bytes raus)
  uint8_t d;
  while (this->available()) this->read_byte(&d);

  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

static inline bool match2(const std::vector<uint8_t> &b, size_t i, uint8_t a, uint8_t c) {
  return (i + 1 < b.size() && b[i] == a && b[i + 1] == c);
}

// Marker-Parser: arbeitet auf wachsendem Buffer und published sobald Daten da sind
void parse_markers(UltraMaXXComponent *self, const std::vector<uint8_t> &f) {
  // Wir scannen alles, aber publishen nur einmal pro Marker
  for (size_t i = 0; i + 1 < f.size(); i++) {

    // 0C 78 + 4B BCD => Seriennummer
    if (!got_serial && match2(f, i, 0x0C, 0x78)) {
      if (i + 2 + 4 <= f.size()) {
        float sn = self->decode_bcd(f, i + 2, 4);
        if (self->serial_number_) self->serial_number_->publish_state(sn);
        got_serial = true;
        ESP_LOGI(TAG, "Parsed serial_number = %.0f", sn);
      }
    }

    // 04 06 + 4B u32le => Energie (MWh) *0.001
    if (!got_energy && match2(f, i, 0x04, 0x06)) {
      if (i + 2 + 4 <= f.size()) {
        uint32_t raw = self->decode_u_le(f, i + 2, 4);
        float mwh = raw * 0.001f;
        if (self->total_energy_) self->total_energy_->publish_state(mwh);
        got_energy = true;
        ESP_LOGI(TAG, "Parsed total_energy = %.3f MWh (raw=%u)", mwh, raw);
      }
    }

    // 0C 14 + 4B BCD => Volumen (m³) *0.01
    if (!got_volume && match2(f, i, 0x0C, 0x14)) {
      if (i + 2 + 4 <= f.size()) {
        float raw_bcd = self->decode_bcd(f, i + 2, 4);
        float m3 = raw_bcd * 0.01f;
        if (self->total_volume_) self->total_volume_->publish_state(m3);
        got_volume = true;
        ESP_LOGI(TAG, "Parsed total_volume = %.2f m3 (bcd=%.0f)", m3, raw_bcd);
      }
    }

    // 0A 5A + 2B BCD => Vorlauf °C *0.1
    if (!got_tflow && match2(f, i, 0x0A, 0x5A)) {
      if (i + 2 + 2 <= f.size()) {
        float raw_bcd = self->decode_bcd(f, i + 2, 2);
        float c = raw_bcd * 0.1f;
        if (self->temp_flow_) self->temp_flow_->publish_state(c);
        got_tflow = true;
        ESP_LOGI(TAG, "Parsed temp_flow = %.1f C (bcd=%.0f)", c, raw_bcd);
      }
    }

    // 0A 5E + 2B BCD => Rücklauf °C *0.1
    if (!got_tret && match2(f, i, 0x0A, 0x5E)) {
      if (i + 2 + 2 <= f.size()) {
        float raw_bcd = self->decode_bcd(f, i + 2, 2);
        float c = raw_bcd * 0.1f;
        if (self->temp_return_) self->temp_return_->publish_state(c);
        got_tret = true;
        ESP_LOGI(TAG, "Parsed temp_return = %.1f C (bcd=%.0f)", c, raw_bcd);
      }
    }

    // 0B 61 + 3B BCD => DeltaT K *0.01
    if (!got_tdiff && match2(f, i, 0x0B, 0x61)) {
      if (i + 2 + 3 <= f.size()) {
        float raw_bcd = self->decode_bcd(f, i + 2, 3);
        float k = raw_bcd * 0.01f;
        if (self->temp_diff_) self->temp_diff_->publish_state(k);
        got_tdiff = true;
        ESP_LOGI(TAG, "Parsed temp_diff = %.2f K (bcd=%.0f)", k, raw_bcd);
      }
    }

    // 04 6D + 4B => Zählerzeit (CP32-ish)
    if (!got_time && match2(f, i, 0x04, 0x6D)) {
      if (i + 2 + 4 <= f.size()) {
        std::string ts;
        bool ok = self->decode_cp32_datetime_(f, i + 2, ts);
        if (!ok) {
          // Fallback: raw hex
          char buf[32];
          snprintf(buf, sizeof(buf), "%02X%02X%02X%02X", f[i+2], f[i+3], f[i+4], f[i+5]);
          ts = buf;
        }
        if (self->meter_time_) self->meter_time_->publish_state(ts);
        got_time = true;
        ESP_LOGI(TAG, "Parsed meter_time = %s", ts.c_str());
      }
    }
  }
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // ---------------- WAKEUP ----------------
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

  // ---------------- WAIT + SWITCH ----------------
  if (state == UM_WAIT) {
    if (now - state_ts_ > 350) {
      ESP_LOGI(TAG, "Switch to 2400 8E1");
      this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
      this->parent_->load_settings();

      // Ganz wichtig: RX leeren (Wakeup-Reste, Echo etc.)
      uint8_t d;
      while (this->available()) this->read_byte(&d);

      // SND_NKE
      uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
      this->write_array(reset, sizeof(reset));
      this->flush();
      ESP_LOGI(TAG, "SND_NKE gesendet");

      state = UM_SEND;
      state_ts_ = now;
    }
    return;
  }

  // ---------------- SEND REQ_UD2 ----------------
  if (state == UM_SEND) {
    if (now - state_ts_ > 80) {
      // Buffer nochmal leer
      uint8_t d;
      while (this->available()) this->read_byte(&d);

      // Toggle FCB über 0x5B/0x7B
      uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
      uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
      uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

      this->write_array(req, sizeof(req));
      this->flush();
      fcb_toggle_ = !fcb_toggle_;

      ESP_LOGI(TAG, "REQ_UD2 gesendet");

      // RX vorbereiten
      rx_buffer_.clear();
      in_frame = false;
      expected_total = -1;

      last_rx_byte_ = now;
      state_ts_ = now;
      state = UM_RX;
    }
    return;
  }

  // ---------------- RX / STREAM PARSER ----------------
  if (state == UM_RX) {

    // Bytes holen
    while (this->available()) {
      uint8_t c;
      if (!this->read_byte(&c)) break;
      last_rx_byte_ = now;

      // Frame-Start finden (alles vorher ignorieren!)
      if (!in_frame) {
        if (c == 0x68) {
          in_frame = true;
          rx_buffer_.clear();
          rx_buffer_.push_back(c);
        }
        continue;
      }

      // Wenn wir im Frame sind, alles sammeln
      rx_buffer_.push_back(c);

      // Sobald 68 L L 68 da ist -> erwartete Gesamtlänge berechnen
      if (expected_total < 0 && rx_buffer_.size() >= 4) {
        if (rx_buffer_[0] == 0x68 && rx_buffer_[3] == 0x68 && rx_buffer_[1] == rx_buffer_[2]) {
          uint8_t L = rx_buffer_[1];
          expected_total = (int) L + 6;  // 68 L L 68 + (L bytes: C..DATA) + CS + 16
          // ESP_LOGD(TAG, "Expected total frame len = %d", expected_total);
        } else {
          // kein gültiger Header -> resync (suche nächstes 0x68 in Buffer)
          in_frame = false;
          expected_total = -1;
          rx_buffer_.clear();
        }
      }

      // Streaming-Parsing: sobald wir Header haben, Marker schon während des Empfangs auswerten
      if (expected_total > 0 && rx_buffer_.size() >= 8) {
        parse_markers(this, rx_buffer_);
      }

      // Frame komplett?
      if (expected_total > 0 && (int) rx_buffer_.size() >= expected_total) {
        // Optional: rudimentäre Endprüfung
        if (rx_buffer_.back() == 0x16) {
          ESP_LOGI(TAG, "Frame complete (%d bytes)", (int) rx_buffer_.size());
          // Noch einmal final parsen (falls letzter Marker erst am Ende vollständig wurde)
          parse_markers(this, rx_buffer_);
        } else {
          ESP_LOGW(TAG, "Frame length reached but no 0x16 end");
        }

        // Ende dieses Zyklus
        rx_buffer_.clear();
        in_frame = false;
        expected_total = -1;
        state = UM_IDLE;
        ESP_LOGI(TAG, "Update finished.");
        return;
      }
    }

    // Timeout (wenn gar nix mehr kommt)
    if (now - state_ts_ > 12000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      in_frame = false;
      expected_total = -1;
      state = UM_IDLE;
      return;
    }

    return;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
