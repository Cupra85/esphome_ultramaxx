#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

// Wir nutzen nun globale Zustände nur noch für den Ablauf, nicht für das Byte-Sammeln
enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

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
  for (size_t i = 0; i < len; i++)
    v |= ((uint32_t)data[start + i]) << (8 * i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;
  char buf[32];
  // CP32 Format nach EN13757-3
  sprintf(buf, "%02u.%02u.20%02u %02u:%02u",
    data[start+2] & 0x1F, data[start+3] & 0x0F, (data[start+3] >> 4) | ((data[start+2] >> 5) << 4), 
    data[start+1] & 0x1F, data[start] & 0x3F);
  out = buf;
  return true;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX Ready");
}

void UltraMaXXComponent::update() {
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  uint32_t now = millis();

  // 1. DATA COLLECTION (Radikaler Ansatz: Alles lesen, was da ist)
  while (this->parent_->available() > 0) {
    uint8_t c;
    if (this->parent_->read_byte(&c)) {
      rx_buffer_.push_back(c);
      last_rx_byte_ = now;
      // Wenn wir ein Frame-Ende (0x16) sehen und genug Daten haben:
      if (c == 0x16 && rx_buffer_.size() > 70 && state == UM_RX) {
          this->parse_ultramaxx_frame_(rx_buffer_);
          rx_buffer_.clear();
          state = UM_IDLE;
      }
    }
  }

  // 2. WAKEUP (Unverändert)
  if (state == UM_WAKEUP) {
    if (now - last_send_ > 25) {
      uint8_t buf[20]; memset(buf, 0x55, 20);
      this->write_array(buf, 20);
      last_send_ = now;
    }
    if (now - wake_start_ > 2200) {
      state = UM_WAIT;
      state_ts_ = now;
    }
  }

  // 3. SWITCH TO 8E1 & SEND SND_NKE
  if (state == UM_WAIT && now - state_ts_ > 450) {
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();
    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, 5);
    state = UM_SEND;
    state_ts_ = now;
  }

  // 4. SEND REQ_UD2
  if (state == UM_SEND && now - state_ts_ > 250) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl + 0xFE) & 0xFF;
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    rx_buffer_.clear(); 
    this->write_array(req, 5);
    fcb_toggle_ = !fcb_toggle_;
    state = UM_RX;
    state_ts_ = now;
  }

  // 5. TIMEOUT SCHUTZ
  if (state == UM_RX && now - state_ts_ > 8000) {
    ESP_LOGW(TAG, "RX Timeout - Buffer hat %d Bytes", rx_buffer_.size());
    rx_buffer_.clear();
    state = UM_IDLE;
  }
}

// Separate Funktion für das Parsing (Sauberkeit)
void UltraMaXXComponent::parse_ultramaxx_frame_(const std::vector<uint8_t> &f) {
  ESP_LOGI(TAG, "Frame vollständig empfangen (%d Bytes). Starte Parsing...", f.size());
  
  // Suche den Header 68 L L 68
  for (size_t i = 0; i + 10 < f.size(); i++) {
    if (f[i] == 0x68 && f[i+3] == 0x68) {
      
      // Laut deinem Log:
      // Offset +20: Seriennummer (DIF 0x0C, VIF 0x78)
      // Offset +26: Energie (DIF 0x04, VIF 0x06 -> Binär!)
      // Offset +32: Volumen (DIF 0x0C, VIF 0x14 -> BCD)
      
      size_t p = i + 19;
      size_t end = f.size() - 2;

      while (p + 2 < end) {
        uint8_t dif = f[p];
        uint8_t vif = f[p+1];

        if (dif == 0x0C && vif == 0x78) { // SN
          if (serial_number_) serial_number_->publish_state(decode_bcd_(f, p+2, 4));
          p += 6;
        } else if (dif == 0x04 && vif == 0x06) { // Energie (Binär!)
          if (total_energy_) total_energy_->publish_state(decode_u_le_(f, p+2, 4) * 0.001f);
          p += 6;
        } else if (dif == 0x0C && vif == 0x14) { // Volumen (BCD)
          if (total_volume_) total_volume_->publish_state(decode_bcd_(f, p+2, 4) * 0.01f);
          p += 6;
        } else if (dif == 0x0A && vif == 0x5A) { // Vorlauf
          if (temp_flow_) temp_flow_->publish_state(decode_bcd_(f, p+2, 2) * 0.1f);
          p += 4;
        } else if (dif == 0x0A && vif == 0x5E) { // Rücklauf
          if (temp_return_) temp_return_->publish_state(decode_bcd_(f, p+2, 2) * 0.1f);
          p += 4;
        } else if (dif == 0x0B && vif == 0x61) { // Delta T
          if (temp_diff_) temp_diff_->publish_state(decode_bcd_(f, p+2, 3) * 0.01f);
          p += 5;
        } else if (dif == 0x04 && vif == 0x6D) { // Zeit
          std::string ts;
          if (meter_time_ && decode_cp32_datetime_(f, p+2, ts)) meter_time_->publish_state(ts);
          p += 6;
        } else {
          p++; 
        }
      }
      ESP_LOGI(TAG, "Parsing erfolgreich beendet.");
      return;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
