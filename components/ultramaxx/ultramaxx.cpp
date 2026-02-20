#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v5.19";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------- Decoder --------------------

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0.0f;

  float value = 0.0f;
  float mul = 1.0f;

  // BCD: low nibble = 1er, high nibble = 10er, byteweise "little" wie bei deinen Frames
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
  for (size_t i = 0; i < len; i++) {
    v |= ((uint32_t) data[start + i]) << (8 * i);
  }
  return v;
}

// CP32 Date/Time: wir halten es simpel wie in deinen bisherigen Versuchen (DD.MM HH:MM)
// In deinen Frames sieht das nach 04 6D so aus: 2D 11 54 32 ...
// => byte0=Day, byte1=Month, byte2=Hour, byte3=Minute (so wie du es schon gemacht hattest)
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;

  char buf[32];
  // Format: DD.MM HH:MM
  // (Interpretation wie in deinen früheren Builds: [day]=data[start], [month]=data[start+1], [hour]=data[start+2], [min]=data[start+3])
  snprintf(buf, sizeof(buf), "%02u.%02u %02u:%02u",
           (unsigned) data[start],
           (unsigned) data[start + 1],
           (unsigned) data[start + 2],
           (unsigned) data[start + 3]);
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

// “Streaming”-Parser: scannt den aktuellen Buffer nach den bekannten Marker-Sequenzen.
// Es wird nur published, wenn genug Daten hinter dem Marker verfügbar sind.
void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  // Wir scannen ab Index 0; publish-guards verhindern mehrfaches publish pro Telegram.
  for (size_t i = 0; i + 2 < buf.size(); i++) {

    // 0C 78 -> Seriennummer (4 BCD-Bytes)
    if (!got_serial_ && i + 2 + 4 <= buf.size() && buf[i] == 0x0C && buf[i + 1] == 0x78) {
      float sn = this->decode_bcd_(buf, i + 2, 4);
      got_serial_ = true;
      ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
      if (serial_number_ != nullptr) serial_number_->publish_state(sn);
      continue;
    }

    // 04 06 -> Gesamtenergie (4 bytes u32 little-endian) * 0.001 => MWh (wie in deinen Logs/Beispielen)
    if (!got_energy_ && i + 2 + 4 <= buf.size() && buf[i] == 0x04 && buf[i + 1] == 0x06) {
      uint32_t raw = this->decode_u_le_(buf, i + 2, 4);
      float mwh = ((float) raw) * 0.001f;
      got_energy_ = true;
      ESP_LOGI(TAG, "ENERGY parsed: %.3f MWh (raw=%u)", mwh, (unsigned) raw);
      if (total_energy_ != nullptr) total_energy_->publish_state(mwh);
      continue;
    }

    // 0C 14 -> Gesamtvolumen (4 BCD-Bytes) * 0.01 => m³ (passt zu deinen Frames 34 39 16 00 -> 1639.34)
    if (!got_volume_ && i + 2 + 4 <= buf.size() && buf[i] == 0x0C && buf[i + 1] == 0x14) {
      float v = this->decode_bcd_(buf, i + 2, 4) * 0.01f;
      got_volume_ = true;
      ESP_LOGI(TAG, "VOLUME parsed: %.2f m3", v);
      if (total_volume_ != nullptr) total_volume_->publish_state(v);
      continue;
    }

    // 0A 5A -> Vorlauf (2 BCD-Bytes) * 0.1 => °C
    if (!got_tflow_ && i + 2 + 2 <= buf.size() && buf[i] == 0x0A && buf[i + 1] == 0x5A) {
      float t = this->decode_bcd_(buf, i + 2, 2) * 0.1f;
      got_tflow_ = true;
      ESP_LOGI(TAG, "TFLOW parsed: %.1f C", t);
      if (temp_flow_ != nullptr) temp_flow_->publish_state(t);
      continue;
    }

    // 0A 5E -> Rücklauf (2 BCD-Bytes) * 0.1 => °C
    if (!got_tret_ && i + 2 + 2 <= buf.size() && buf[i] == 0x0A && buf[i + 1] == 0x5E) {
      float t = this->decode_bcd_(buf, i + 2, 2) * 0.1f;
      got_tret_ = true;
      ESP_LOGI(TAG, "TRET parsed: %.1f C", t);
      if (temp_return_ != nullptr) temp_return_->publish_state(t);
      continue;
    }

    // 0B 61 -> Temperaturdifferenz (3 bytes u?? in deinen Frames wie 06 00 00)
    // Wir machen es wie du wolltest: BCD(3) * 0.01
    if (!got_tdiff_ && i + 2 + 3 <= buf.size() && buf[i] == 0x0B && buf[i + 1] == 0x61) {
      float dt = this->decode_bcd_(buf, i + 2, 3) * 0.01f;
      got_tdiff_ = true;
      ESP_LOGI(TAG, "TDIFF parsed: %.2f K", dt);
      if (temp_diff_ != nullptr) temp_diff_->publish_state(dt);
      continue;
    }

    // 04 6D -> Zählerzeit (4 bytes CP32)
    if (!got_time_ && i + 2 + 4 <= buf.size() && buf[i] == 0x04 && buf[i + 1] == 0x6D) {
      std::string ts;
      if (this->decode_cp32_datetime_(buf, i + 2, ts)) {
        got_time_ = true;
        ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
        if (meter_time_ != nullptr) meter_time_->publish_state(ts);
      }
      continue;
    }
  }
}

// -------------------- ESPHome component --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // Wakeup: 2400 8N1 (so wie von dir gefordert: Wakeup-Sequenz NICHT ändern)
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // RX / state init
  rx_buffer_.clear();
  in_frame_ = false;
  expected_len_ = 0;
  last_rx_ms_ = millis();

  reset_parse_flags_();

  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  uint32_t now = millis();

  // -------------------- RX handling (nur wenn wir in UM_RX sind) --------------------
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (!this->read_byte(&c)) break;

      last_rx_ms_ = now;

      // Frame start suchen: ignoriere alles bis 0x68 (damit 0x55-Wakeup nicht stört)
      if (!in_frame_) {
        if (c != 0x68) continue;

        // Start erkannt
        in_frame_ = true;
        rx_buffer_.clear();
        rx_buffer_.push_back(c);
        expected_len_ = 0;
        continue;
      }

      // Im Frame: Byte sammeln
      rx_buffer_.push_back(c);

      // Länge bestimmen sobald wir 68 L L 68 haben
      if (expected_len_ == 0 && rx_buffer_.size() >= 4) {
        // rx_buffer_[0] == 0x68
        uint8_t l1 = rx_buffer_[1];
        uint8_t l2 = rx_buffer_[2];
        uint8_t s2 = rx_buffer_[3];

        if (s2 == 0x68 && l1 == l2 && l1 >= 3) {
          expected_len_ = (size_t) l1 + 6;  // total bytes including header+cs+end
          ESP_LOGD(TAG, "Longframe header ok. L=%u -> expected_total=%u", (unsigned) l1, (unsigned) expected_len_);
        } else {
          // Kein gültiger Longframe-Header -> resync: suche nächstes 0x68
          in_frame_ = false;
          rx_buffer_.clear();
          expected_len_ = 0;
          continue;
        }
      }

      // Streaming-Parse auf dem aktuellen Puffer (wie von dir gewünscht: nicht auf vollständigen Frame warten)
      this->parse_and_publish_(rx_buffer_);

      // Ende / kompletter Frame?
      if (expected_len_ > 0) {
        if (rx_buffer_.size() >= expected_len_) {
          // optional: check stop byte
          if (!rx_buffer_.empty() && rx_buffer_.back() == 0x16) {
            ESP_LOGD(TAG, "Frame complete (%u bytes).", (unsigned) rx_buffer_.size());
          } else {
            ESP_LOGW(TAG, "Frame length reached but no 0x16 at end. size=%u last=%02X",
                     (unsigned) rx_buffer_.size(), rx_buffer_.empty() ? 0 : rx_buffer_.back());
          }

          // Frame abschließen
          in_frame_ = false;
          expected_len_ = 0;
          rx_buffer_.clear();
          state = UM_IDLE;
          return;
        }
      }
    }

    // RX Timeout innerhalb UM_RX:
    // - wenn Frame gestartet: etwas großzügig
    // - wenn kein Frame gestartet: kürzer
    if (in_frame_) {
      if (now - last_rx_ms_ > 1200) {
        ESP_LOGW(TAG, "RX Timeout (in_frame). buf=%u", (unsigned) rx_buffer_.size());
        in_frame_ = false;
        expected_len_ = 0;
        rx_buffer_.clear();
        state = UM_IDLE;
      }
    } else {
      if (now - state_ts_ > 6000) {
        ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
        rx_buffer_.clear();
        state = UM_IDLE;
      }
    }
  }

  // -------------------- STATE MACHINE --------------------

  // WAKEUP
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

  // WAIT -> switch to 2400 8E1 and send NKE
  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // UART RX leer machen
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    // Parser state reset (wichtig: damit kein 0x55 Müll bleibt)
    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    reset_parse_flags_();

    // NKE
    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
    return;
  }

  // SEND -> REQ_UD2 (FCB toggling)
  if (state == UM_SEND && now - state_ts_ > 150) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;     // UD2 with FCB toggling
    uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);

    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    // RX vorbereiten
    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    last_rx_ms_ = now;
    reset_parse_flags_();

    state = UM_RX;
    state_ts_ = now;
    return;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
