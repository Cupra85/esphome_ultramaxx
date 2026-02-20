#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v6.0";

// Zustände (Wakeup Sequenz NICHT ändern)
enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------- Decoder --------------------

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return NAN;

  // BCD little-endian: jedes Byte enthält 2 Dezimalziffern
  // low nibble = niedrigere Stelle, high nibble = nächsthöhere Stelle
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

// CP32 (M-Bus) Datum/Zeit (hier: sehr simpel wie bei dir bisher genutzt)
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;

  // Achtung: das ist eine pragmatische Darstellung passend zu deinen Bytes
  // Format: DD.MM HH:MM (aus 4 Bytes)
  char buf[32];
  // data[start+0] = day?
  // data[start+1] = month?
  // data[start+2] = hour?
  // data[start+3] = minute?
  // Deine bisherigen Logs/Bytes passen zu dieser Darstellung:
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
  got_tflow_ = false;
  got_tret_ = false;
  got_tdiff_ = false;
  got_time_ = false;
}

// -------------------- Streaming Parser --------------------
// Idee: wir laufen über den aktuellen Buffer und suchen nach den Marker-Bytes.
// Sobald Marker + genügend Datenbytes vorhanden sind -> dekodieren + publishen.
// Wir publishen je Update-Zyklus jedes Feld nur einmal (guards_*).

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {
  // Minimum: Marker (2) + Daten (mind. 2..4)
  if (buf.size() < 6) return;

// ---------- STREAM PARSER (CORRECT) ----------
for (size_t i = 0; i + 6 < rx_buffer_.size(); i++) {

  // SERIAL
  if (!got_serial_ && rx_buffer_[i]==0x0C && rx_buffer_[i+1]==0x78) {
    float sn = decode_bcd_(rx_buffer_, i+2, 4);
    ESP_LOGI(TAG,"SERIAL parsed: %.0f",sn);
    if(serial_number_) serial_number_->publish_state(sn);
    got_serial_ = true;
  }

  // ENERGY
  if (!got_energy_ && rx_buffer_[i]==0x04 && rx_buffer_[i+1]==0x06) {
    uint32_t raw = decode_u_le_(rx_buffer_, i+2, 4);
    float e = raw * 0.001f;
    ESP_LOGI(TAG,"ENERGY parsed: %.3f",e);
    if(total_energy_) total_energy_->publish_state(e);
    got_energy_ = true;
  }

  // VOLUME
  if (!got_volume_ && rx_buffer_[i]==0x0C && rx_buffer_[i+1]==0x14) {
    float v = decode_bcd_(rx_buffer_, i+2, 4)*0.01f;
    ESP_LOGI(TAG,"VOLUME parsed: %.2f",v);
    if(total_volume_) total_volume_->publish_state(v);
    got_volume_ = true;
  }

  // FLOW TEMP
  if (!got_tflow_ && rx_buffer_[i]==0x0A && rx_buffer_[i+1]==0x5A) {
    float t = decode_bcd_(rx_buffer_, i+2, 2)*0.1f;
    ESP_LOGI(TAG,"FLOW TEMP parsed: %.1f",t);
    if(temp_flow_) temp_flow_->publish_state(t);
    got_tflow_ = true;
  }

  // RETURN TEMP
  if (!got_tret_ && rx_buffer_[i]==0x0A && rx_buffer_[i+1]==0x5E) {
    float t = decode_bcd_(rx_buffer_, i+2, 2)*0.1f;
    ESP_LOGI(TAG,"RETURN TEMP parsed: %.1f",t);
    if(temp_return_) temp_return_->publish_state(t);
    got_tret_ = true;
  }

  // DELTA T
  if (!got_tdiff_ && rx_buffer_[i]==0x0B && rx_buffer_[i+1]==0x61) {
    float dt = decode_bcd_(rx_buffer_, i+2, 3)*0.01f;
    ESP_LOGI(TAG,"DELTA T parsed: %.2f",dt);
    if(temp_diff_) temp_diff_->publish_state(dt);
    got_tdiff_ = true;
  }

  // METER TIME
  if (!got_time_ && rx_buffer_[i]==0x04 && rx_buffer_[i+1]==0x6D) {
    std::string ts;
    if(decode_cp32_datetime_(rx_buffer_, i+2, ts)) {
      ESP_LOGI(TAG,"TIME parsed: %s",ts.c_str());
      if(meter_time_) meter_time_->publish_state(ts);
      got_time_ = true;
    }
  }
}

// -------------------- ESPHome hooks --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // Start mit 2400 8N1 (Wakeup)
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // neuer Zyklus
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

  // -------------------- RX handling --------------------
  // WICHTIG: UARTDebug parallel deaktivieren, sonst kommen hier keine Bytes an!
  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c)) break;

    last_rx_ms_ = now;

    // Optional: Byte-Debug (macht Log riesig)
    // ESP_LOGD(TAG, "RX %02X", c);

    // Frame-Sync: wir starten erst ab 0x68
    if (!in_frame_) {
      if (c == 0x68) {
        in_frame_ = true;
        rx_buffer_.clear();
        rx_buffer_.push_back(c);
        expected_len_ = 0;
      }
      continue;
    }

    rx_buffer_.push_back(c);

    // Länge bestimmen sobald 68 L L 68 vollständig
    // Format: 68 L L 68 ...
    if (rx_buffer_.size() == 4) {
      if (rx_buffer_[0] == 0x68 && rx_buffer_[3] == 0x68 && rx_buffer_[1] == rx_buffer_[2]) {
        // Gesamtframe: 4 (header) + L bytes + 2 (cs+16)
        expected_len_ = (size_t) rx_buffer_[1] + 6;
      } else {
        // kein gültiger longframe header -> resync
        in_frame_ = false;
        rx_buffer_.clear();
        expected_len_ = 0;
        continue;
      }
    }

    // Streaming parse: sobald Daten im Buffer sind, laufend versuchen
    this->parse_and_publish_(rx_buffer_);

    // Frame-Ende: entweder erwartete Länge erreicht oder 0x16 gesehen
    if (expected_len_ > 0 && rx_buffer_.size() >= expected_len_) {
      // fertig, für nächsten Frame resync
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      // NICHT state=IDLE hier, State machine macht Timeout/Ende
    } else if (c == 0x16 && rx_buffer_.size() > 8) {
      // fallback: Ende erkannt
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
    }
  }

  // RX Stille-Timeout (falls Frame unvollständig)
  if (in_frame_ && (now - last_rx_ms_ > 1200)) {
    in_frame_ = false;
    rx_buffer_.clear();
    expected_len_ = 0;
  }

  // -------------------- State machine (Wakeup Sequenz unverändert) --------------------

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

    // UART RX FIFO leeren, damit kein 0x55-Müll oder ACK-Reste stören
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    // Parser resync für die Antwort
    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    last_rx_ms_ = now;

    // SND_NKE (Reset/Link)
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

    fcb_toggle_ = !fcb_toggle_;

    // Empfangsphase
    state = UM_RX;
    state_ts_ = now;

    // Parser bereit machen
    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    last_rx_ms_ = now;
    reset_parse_flags_();
  }

  if (state == UM_RX) {
    // wenn wir nach 5s keinen Start (0x68) gesehen haben -> Timeout
    if (!in_frame_ && (now - state_ts_ > 5000)) {
      ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
      rx_buffer_.clear();
      expected_len_ = 0;
      state = UM_IDLE;
    }

    // wenn wir Daten hatten aber Parser noch “in_frame” hängt zu lange
    if (in_frame_ && (now - last_rx_ms_ > 2000)) {
      ESP_LOGW(TAG, "RX Timeout (in frame). buf=%u", (unsigned) rx_buffer_.size());
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      state = UM_IDLE;
    }

    // Optional: wenn alle Werte schon da sind, kannst du früher abbrechen:
    // if (got_serial_ && got_energy_ && got_volume_ && got_tflow_ && got_tret_ && got_tdiff_ && got_time_) {
    //   ESP_LOGI(TAG, "All fields parsed -> finish");
    //   state = UM_IDLE;
    // }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
