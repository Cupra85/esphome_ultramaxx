#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v5.12";

// Zustände: Wakeup-Sequenz bleibt wie bei dir, nur RX/Parsing stabilisiert
enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------- Decoder --------------------
float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0.0f;
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

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= ((uint32_t) data[start + i]) << (8 * i);
  return v;
}

// CP32 (0x6D) — bei deinem Zähler kommt das als 4 Byte, und in deinen Frames ist es (min, hour, day, month/yearbits)
// Du wolltest "einfach" – wir geben ein robustes, lesbares Format zurück, ohne DOW/Sommerzeit-Bits zu verkomplizieren.
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  if (start + 4 > data.size()) return false;

  // Viele Geräte liefern: [min][hour][day][month|yearbits] oder leicht variiert.
  // Deine bisherigen Beispiele passen zu: dd.mm hh:mm (ohne Jahr).
  const uint8_t b0 = data[start + 0];
  const uint8_t b1 = data[start + 1];
  const uint8_t b2 = data[start + 2];
  const uint8_t b3 = data[start + 3];

  // Heuristik: Minute/Hour sind < 60/24; Tag <= 31; Monat <= 12 (in low nibble/low bits)
  // Wir setzen:
  //   mm = b0, hh = b1, day = b2, month = (b3 & 0x0F)
  // Falls das nicht plausibel ist, fallback auf dein altes Mapping.
  uint8_t minute = b0;
  uint8_t hour   = b1;
  uint8_t day    = b2;
  uint8_t month  = (b3 & 0x0F);

  bool plausible = (minute < 60) && (hour < 24) && (day >= 1 && day <= 31) && (month >= 1 && month <= 12);

  char buf[32];
  if (plausible) {
    snprintf(buf, sizeof(buf), "%02u.%02u %02u:%02u", day, month, hour, minute);
    out = buf;
    return true;
  }

  // Fallback (dein älteres Mapping)
  snprintf(buf, sizeof(buf), "%02u.%02u %02u:%02u", data[start + 1], data[start + 0], data[start + 3], data[start + 2]);
  out = buf;
  return true;
}

// -------------------- Setup/Update --------------------
void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX Parser v6 LOADED");
  ESP_LOGI(TAG, "UltraMaXX STARTED --- VERSION: %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // Wakeup (8N1) – NICHT verändern
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_  = 0;

  state = UM_WAKEUP;
}

// -------------------- Parser: Longframe stabil erkennen --------------------
// M-Bus Longframe: 68 L L 68 C A CI [DATA...(L-3 bytes?)] CS 16
// Für die Voll-Länge gilt: total = L + 6  (68 + L + L + 68 + ... + CS + 16)
static inline bool mbus_has_complete_frame(const std::vector<uint8_t> &b, size_t &total_len_out) {
  total_len_out = 0;
  if (b.size() < 6) return false;
  if (b[0] != 0x68) return false;

  const uint8_t L1 = b[1];
  const uint8_t L2 = b[2];
  if (L1 != L2) return false;

  const size_t total = (size_t) L1 + 6;  // Standardformel für 68-Frames
  total_len_out = total;

  if (total < 6) return false;
  if (b.size() < total) return false;
  if (b[3] != 0x68) return false;
  if (b[total - 1] != 0x16) return false;
  return true;
}

// Scan-Funktion: findet Marker und published, sobald die Bytes da sind (ohne auf „perfekten“ DIF/VIF-Parser zu bestehen)
void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &f) {
  // Wir scannen über den kompletten Frame nach den von dir gewünschten Signaturen:
  // 0C 78 -> Seriennummer (4 BCD)
  // 04 06 -> Gesamtenergie (u32 le) * 0.001 => MWh
  // 0C 14 -> Gesamtvolumen (4 BCD) * 0.01 => m³
  // 0A 5A -> Vorlauf (2 BCD) * 0.1 => °C
  // 0A 5E -> Rücklauf (2 BCD) * 0.1 => °C
  // 0B 61 -> DeltaT (3 BCD) * 0.01 => K
  // 04 6D -> Zählerzeit (CP32, 4 Byte)

  bool any = false;

  for (size_t i = 0; i + 1 < f.size(); i++) {
    // SERIAL: 0C 78 + 4 bytes
    if (i + 2 + 4 <= f.size() && f[i] == 0x0C && f[i + 1] == 0x78) {
      float sn = decode_bcd(f, i + 2, 4);
      ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
      if (serial_number_) serial_number_->publish_state(sn);
      any = true;
      continue;
    }

    // ENERGY: 04 06 + 4 bytes little-endian
    if (i + 2 + 4 <= f.size() && f[i] == 0x04 && f[i + 1] == 0x06) {
      uint32_t raw = decode_u_le(f, i + 2, 4);
      float e_mwh = raw * 0.001f;
      ESP_LOGI(TAG, "ENERGY parsed: %.3f MWh (raw=%u)", e_mwh, (unsigned) raw);
      if (total_energy_) total_energy_->publish_state(e_mwh);
      any = true;
      continue;
    }

    // VOLUME: 0C 14 + 4 bytes BCD
    if (i + 2 + 4 <= f.size() && f[i] == 0x0C && f[i + 1] == 0x14) {
      float v = decode_bcd(f, i + 2, 4) * 0.01f;
      ESP_LOGI(TAG, "VOLUME parsed: %.2f m³", v);
      if (total_volume_) total_volume_->publish_state(v);
      any = true;
      continue;
    }

    // FLOW TEMP: 0A 5A + 2 bytes BCD
    if (i + 2 + 2 <= f.size() && f[i] == 0x0A && f[i + 1] == 0x5A) {
      float t = decode_bcd(f, i + 2, 2) * 0.1f;
      ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f °C", t);
      if (temp_flow_) temp_flow_->publish_state(t);
      any = true;
      continue;
    }

    // RETURN TEMP: 0A 5E + 2 bytes BCD
    if (i + 2 + 2 <= f.size() && f[i] == 0x0A && f[i + 1] == 0x5E) {
      float t = decode_bcd(f, i + 2, 2) * 0.1f;
      ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f °C", t);
      if (temp_return_) temp_return_->publish_state(t);
      any = true;
      continue;
    }

    // DELTA T: 0B 61 + 3 bytes BCD
    if (i + 2 + 3 <= f.size() && f[i] == 0x0B && f[i + 1] == 0x61) {
      float dt = decode_bcd(f, i + 2, 3) * 0.01f;
      ESP_LOGI(TAG, "DELTA T parsed: %.2f K", dt);
      if (temp_diff_) temp_diff_->publish_state(dt);
      any = true;
      continue;
    }

    // METER TIME: 04 6D + 4 bytes CP32
    if (i + 2 + 4 <= f.size() && f[i] == 0x04 && f[i + 1] == 0x6D) {
      std::string ts;
      if (decode_cp32_datetime_(f, i + 2, ts)) {
        ESP_LOGI(TAG, "METER TIME parsed: %s", ts.c_str());
        if (meter_time_) meter_time_->publish_state(ts);
        any = true;
      }
      continue;
    }
  }

  if (!any) {
    ESP_LOGW(TAG, "Parser found no known markers in frame (%u bytes).", (unsigned) f.size());
  }
}

// -------------------- Main loop --------------------
void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // ---------- RX sammeln nur im UM_RX ----------
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (!this->read_byte(&c)) break;

      last_rx_byte_ = now;

      // Resync: alles verwerfen bis Start 0x68
      if (rx_buffer_.empty()) {
        if (c != 0x68) continue;
      } else {
        // falls wir “vorne” Müll haben, bis zum nächsten 0x68 schneiden
        if (rx_buffer_.size() == 1 && rx_buffer_[0] != 0x68) {
          rx_buffer_.clear();
          if (c != 0x68) continue;
        }
      }

      rx_buffer_.push_back(c);
    }

    // Wenn wir einen kompletten Frame haben: parse + fertig
    size_t total_len = 0;
    if (mbus_has_complete_frame(rx_buffer_, total_len)) {
      // Frame ggf. auf exakte Länge kürzen (falls danach noch Bytes kamen)
      if (rx_buffer_.size() > total_len) {
        rx_buffer_.resize(total_len);
      }

      ESP_LOGI(TAG, "Parsing Frame (%u bytes)...", (unsigned) rx_buffer_.size());
      this->parse_and_publish_(rx_buffer_);
      ESP_LOGI(TAG, "Update finished.");

      rx_buffer_.clear();
      state = UM_IDLE;
    }

    // Timeout: wenn nach Request nichts mehr kommt
    // (wichtig: Timeout auf „seit letztem RX-Byte“, nicht auf „seit Request“)
    if (!rx_buffer_.empty()) {
      if (now - last_rx_byte_ > 1200) {
        ESP_LOGW(TAG, "RX Timeout (no new bytes for %ums, buffer=%u bytes)", (unsigned) (now - last_rx_byte_),
                 (unsigned) rx_buffer_.size());
        rx_buffer_.clear();
        state = UM_IDLE;
      }
    } else {
      // noch gar nichts empfangen
      if (now - state_ts_ > 3000) {
        ESP_LOGW(TAG, "RX Timeout (no frame start)");
        state = UM_IDLE;
      }
    }
  }

  // ---------- STATE MACHINE (Wakeup unverändert) ----------
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

    // UART-RX leeren (Echo/Wakeup-Reste raus)
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();

    // SND_NKE
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
    uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    // WICHTIG: RX vor dem Request noch einmal leeren
    uint8_t d;
    while (this->available()) this->read_byte(&d);
    rx_buffer_.clear();

    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    state = UM_RX;
    state_ts_ = now;
    last_rx_byte_ = now;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
