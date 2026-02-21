#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const ULTRAMAXX_VERSION = "UltraMaXX Parser v6.7";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------- Decoder --------------------

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return NAN;
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
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= ((uint32_t)data[start + i]) << (8 * i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data,
                                               size_t start,
                                               std::string &out) const {
  if (start + 4 > data.size()) return false;

  // UltraMaXX liefert CP32 direkt als:
  // minute, hour, day, month (kein packed bitfield)

  const uint8_t minute = data[start + 0];
  const uint8_t hour   = data[start + 1];
  const uint8_t day    = data[start + 2];
  const uint8_t month  = data[start + 3];

  // Jahr liefert dieses Modell NICHT im CP32 Block.
  // Wir lassen es daher weg (wie viele M-Bus Reader auch).

  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12)
    return false;

  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02u.%02u %02u:%02u",
                day, month, hour, minute);

  out = buf;
  return true;
}

// -------------------- Flags --------------------

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = false;
  got_energy_ = false;
  got_volume_ = false;
  got_tflow_  = false;
  got_tret_   = false;
  got_tdiff_  = false;
  got_time_   = false;
  got_operating_ = false;
  got_error_ = false;
}

// -------------------- Parser --------------------

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {

  const size_t n = buf.size();
  if (n < 10) return;

  for (size_t i = 0; i + 2 < n; i++) {

    if (!got_serial_ && i + 6 <= n && buf[i]==0x0C && buf[i+1]==0x78) {
      float sn = decode_bcd_(buf,i+2,4);
      got_serial_ = true;
      ESP_LOGI(TAG,"SERIAL parsed: %.0f",sn);
      if(serial_number_) serial_number_->publish_state(sn);
    }

    if (!got_energy_ && i + 6 <= n && buf[i]==0x04 && buf[i+1]==0x06) {
      uint32_t raw = decode_u_le_(buf,i+2,4);
      float kwh = (float)raw;
      got_energy_=true;
      ESP_LOGI(TAG,"ENERGY parsed: %.0f kWh (raw=%u)",kwh,(unsigned)raw);
      if(total_energy_) total_energy_->publish_state(kwh);
    }

    if (!got_volume_ && i + 6 <= n && buf[i]==0x0C && buf[i+1]==0x14) {
      float v = decode_bcd_(buf,i+2,4)*0.01f;
      got_volume_=true;
      ESP_LOGI(TAG,"VOLUME parsed: %.2f m³",v);
      if(total_volume_) total_volume_->publish_state(v);
    }

    if (!got_tflow_ && i + 4 <= n && buf[i]==0x0A && buf[i+1]==0x5A) {
      float t = decode_bcd_(buf,i+2,2)*0.1f;
      got_tflow_=true;
      ESP_LOGI(TAG,"FLOW TEMP parsed: %.1f °C",t);
      if(temp_flow_) temp_flow_->publish_state(t);
    }

    if (!got_tret_ && i + 4 <= n && buf[i]==0x0A && buf[i+1]==0x5E) {
      float t = decode_bcd_(buf,i+2,2)*0.1f;
      got_tret_=true;
      ESP_LOGI(TAG,"RETURN TEMP parsed: %.1f °C",t);
      if(temp_return_) temp_return_->publish_state(t);
    }

    if (!got_tdiff_ && i + 5 <= n && buf[i]==0x0B && buf[i+1]==0x61) {
      float dt = decode_bcd_(buf,i+2,3)*0.01f;
      got_tdiff_=true;
      ESP_LOGI(TAG,"DELTA T parsed: %.2f K",dt);
      if(temp_diff_) temp_diff_->publish_state(dt);
    }

    if (!got_time_ && i + 6 <= n && buf[i]==0x04 && buf[i+1]==0x6D) {
      std::string ts;
      if(decode_cp32_datetime_(buf,i+2,ts)) {
        got_time_=true;
        ESP_LOGI(TAG,"TIME parsed: %s",ts.c_str());
        if(meter_time_) meter_time_->publish_state(ts);
      }
    }

    if (!got_operating_ && i + 4 <= n && buf[i]==0x02 && buf[i+1]==0x27) {
      uint32_t days = decode_u_le_(buf,i+2,2);
      got_operating_=true;
      ESP_LOGI(TAG,"OPERATING TIME parsed: %u days",(unsigned)days);
      if(operating_time_) operating_time_->publish_state(days);
    }

    if (!got_error_ && i + 4 <= n && buf[i]==0x09 && buf[i+1]==0xFD && buf[i+2]==0x0E) {
      uint32_t hours = buf[i+3];
      got_error_=true;
      ESP_LOGI(TAG,"ERROR TIME parsed: %u h",(unsigned)hours);
      if(error_time_) error_time_->publish_state(hours);
    }
  }
}

// -------------------- Component lifecycle --------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s", ULTRAMAXX_VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START (%s) ===", ULTRAMAXX_VERSION);

  // Wakeup-Phase: 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // RX state reset
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

  // -------------------- RX: Byte-stream aufnehmen & streaming parse --------------------
  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c)) break;

    last_rx_ms_ = now;

    // Start of long frame suchen (0x68). Alles davor ignorieren (z.B. 0x55 wakeup noise).
    if (!in_frame_) {
      if (c != 0x68) continue;
      in_frame_ = true;
      rx_buffer_.clear();
      expected_len_ = 0;
      rx_buffer_.push_back(c);
      continue;
    }

    rx_buffer_.push_back(c);

    // Expected length bestimmen sobald wir "68 L L 68" haben
    if (rx_buffer_.size() == 4) {
      // rx_buffer_[0] = 68
      const uint8_t L1 = rx_buffer_[1];
      const uint8_t L2 = rx_buffer_[2];
      const uint8_t S2 = rx_buffer_[3];
      if (S2 != 0x68 || L1 != L2 || L1 < 3) {
        // Kein gültiger Longframe-Header -> resync: suche neues 0x68
        in_frame_ = false;
        rx_buffer_.clear();
        expected_len_ = 0;
        continue;
      }
      expected_len_ = (size_t) (4 + L1 + 2);  // 68 L L 68 + (L bytes ab C..DATA) + CS + 16
    }

    // Streaming parse läuft, sobald genug Bytes im Buffer sind.
    // (Das ist genau dein Wunsch: nicht "bis vollständiger Longframe", sondern sobald Marker auftauchen.)
    this->parse_and_publish_(rx_buffer_);

    // Frame komplett?
    if (expected_len_ > 0 && rx_buffer_.size() >= expected_len_) {
      // Optional sanity: letztes Byte sollte 0x16 sein
      if (rx_buffer_.back() != 0x16) {
        ESP_LOGW(TAG, "Frame length reached (%u) but last byte != 0x16 (got %02X) -> resync",
                 (unsigned) rx_buffer_.size(), rx_buffer_.back());
      }
      // Telegram Ende: bereit für nächstes
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      // Flags bleiben gesetzt bis nächster update()-Zyklus (verhindert Spam)
    }
  }

  // -------------------- RX timeout (nur wenn wir wirklich auf Daten warten) --------------------
  if (state == UM_RX) {
    // Kein Header jemals gesehen
    if (!in_frame_ && (now - last_rx_ms_ > 2200)) {
      ESP_LOGW(TAG, "RX Timeout (no header). buf=%u", (unsigned) rx_buffer_.size());
      state = UM_IDLE;
    }
    // Header gesehen, aber Frame kommt nicht zu Ende
    if (in_frame_ && (now - last_rx_ms_ > 2200)) {
      ESP_LOGW(TAG, "RX Timeout (in frame). buf=%u exp=%u", (unsigned) rx_buffer_.size(), (unsigned) expected_len_);
      in_frame_ = false;
      rx_buffer_.clear();
      expected_len_ = 0;
      state = UM_IDLE;
    }
  }

  // -------------------- State machine --------------------

  // WAKEUP: 0x55… senden (8N1)
  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20];
      std::memset(buf, 0x55, sizeof(buf));
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

  // WAIT: Umschalten auf 8E1
  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // UART RX FIFO leeren (damit keine 0x55-Reste reinlaufen)
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;
    expected_len_ = 0;
    last_rx_ms_ = now;

    // SND_NKE
    const uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
    return;
  }

  // SEND: REQ_UD2
  if (state == UM_SEND && now - state_ts_ > 150) {
    const uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    const uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    const uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();
    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    // RX aktivieren
    state = UM_RX;
    last_rx_ms_ = now;
    return;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
