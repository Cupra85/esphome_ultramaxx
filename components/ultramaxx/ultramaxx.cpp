#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
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

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= ((uint32_t) data[start + i]) << (8 * i);
  return v;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_  = 0;

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // RX immer einsammeln (nur im RX-State in Buffer)
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // WAKEUP (unverändert)
  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20];
      memset(buf, 0x55, 20);
      this->write_array(buf, 20);
      last_send_ = now;
    }

    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
    }
  }

  // SWITCH UART + SND_NKE (unverändert)
  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while (this->available()) this->read_byte(&d);

    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  // REQ_UD2 / SND_UD toggle (unverändert)
  if (state == UM_SEND && now - state_ts_ > 150) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl + 0xFE) & 0xFF;

    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;
    rx_buffer_.clear();

    state = UM_RX;
    state_ts_ = now;
    last_rx_byte_ = now;
  }

  // ---------- PARSER ----------
  if (state == UM_RX) {

    // erst arbeiten wenn wir mind. etwas Sinnvolles haben
    if (rx_buffer_.size() >= 6) {

      // Suche Longframe: 68 L L 68 ... CS 16
      for (size_t i = 0; i + 5 < rx_buffer_.size(); i++) {

        if (rx_buffer_[i] != 0x68) continue;

        uint8_t L1 = rx_buffer_[i + 1];
        uint8_t L2 = rx_buffer_[i + 2];
        if (L1 != L2) continue;

        if (rx_buffer_[i + 3] != 0x68) continue;

        // ✅ FIX: Endbyte-Index ist i + L + 5 (nicht +6)
        size_t end_index = i + (size_t)L1 + 5;
        if (end_index >= rx_buffer_.size()) continue;

        if (rx_buffer_[end_index] != 0x16) continue;

        ESP_LOGI(TAG, "Valid Long Frame gefunden (L=%u, total=%u Bytes)", L1, (unsigned)(L1 + 6));

        // Datenbereich beginnt bei i+4 (C-Feld), aber wir scannen wie bisher nach DIF/VIF Mustern.
        // Du bekommst aktuell diese Marker sicher im Frame:
        // 0C 78 (Seriennummer), 04 06 (Energie LE u32), 0C 14 (Volumen BCD), 0A 5A / 0A 5E (Temperaturen), 0B 61 (DeltaT 3-byte BCD)

        // Wir scannen im gesamten Framebereich (ohne 68.. und ohne 16)
        const size_t frame_start = i;
        const size_t frame_end_excl = end_index; // 0x16 excluded

        for (size_t p = frame_start; p + 6 < frame_end_excl; p++) {

          // Seriennummer: 0C 78 + 4 BCD bytes
          if (rx_buffer_[p] == 0x0C && rx_buffer_[p + 1] == 0x78) {
            if (serial_number_) serial_number_->publish_state(decode_bcd(rx_buffer_, p + 2, 4));
          }

          // Energie: 04 06 + u32 little endian (dein Frame: CC 5E 00 00 => 24268 -> 24.268 MWh)
          else if (rx_buffer_[p] == 0x04 && rx_buffer_[p + 1] == 0x06) {
            if (total_energy_) total_energy_->publish_state(decode_u_le(rx_buffer_, p + 2, 4) * 0.001f);
          }

          // Volumen: 0C 14 + 4 BCD bytes, Skalierung 0.01 (dein Beispiel 34 39 16 00 -> 1639.34)
          else if (rx_buffer_[p] == 0x0C && rx_buffer_[p + 1] == 0x14) {
            if (total_volume_) total_volume_->publish_state(decode_bcd(rx_buffer_, p + 2, 4) * 0.01f);
          }

          // Vorlauf: 0A 5A + 2 BCD bytes, *0.1
          else if (rx_buffer_[p] == 0x0A && rx_buffer_[p + 1] == 0x5A) {
            if (temp_flow_) temp_flow_->publish_state(decode_bcd(rx_buffer_, p + 2, 2) * 0.1f);
          }

          // Rücklauf: 0A 5E + 2 BCD bytes, *0.1
          else if (rx_buffer_[p] == 0x0A && rx_buffer_[p + 1] == 0x5E) {
            if (temp_return_) temp_return_->publish_state(decode_bcd(rx_buffer_, p + 2, 2) * 0.1f);
          }

          // DeltaT: 0B 61 + 3 BCD bytes, *0.01
          else if (rx_buffer_[p] == 0x0B && rx_buffer_[p + 1] == 0x61) {
            if (temp_diff_) temp_diff_->publish_state(decode_bcd(rx_buffer_, p + 2, 3) * 0.01f);
          }
        }

        ESP_LOGI(TAG, "Update finished.");

        rx_buffer_.clear();
        state = UM_IDLE;
        return;
      }
    }

    if (now - state_ts_ > 10000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
