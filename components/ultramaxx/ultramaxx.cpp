#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_RESET,
  UM_SEND,
  UM_RX
};

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;
static uint32_t last_rx_byte = 0;

float UltraMaXXComponent::decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len) {
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return value;
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

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
    ESP_LOGI(TAG, "UltraMaXX Heat Meter Component started");
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  wake_start_ = millis();
  last_send_  = 0;
  rx_buffer_.clear();

  state = UM_WAKEUP;
    ESP_LOGI(TAG, "Starting Update Cycle...");
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

  // WAKEUP
  if (state == UM_WAKEUP) {

    if (now - last_send_ > 15) {
      uint8_t buf[20];
      for (int i = 0; i < 20; i++) buf[i] = 0x55;
      this->write_array(buf, 20);
      last_send_ = now;
    uint32_t now = millis();

    // 1. WAKEUP: 2,2 Sekunden 0x55 senden (2400 Baud)
    if (state == UM_WAKEUP) {
        if (now - last_send_ > 15) {
            uint8_t wakeup_byte = 0x55;
            this->write_array(&wakeup_byte, 1);
            last_send_ = now;
        }
        if (now - wake_start_ > 2200) {
            state = UM_WAIT;
            state_ts_ = now;
        }
    }

    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
    // 2. SWITCH: Wechsel auf 2400 8E1 (Even Parity)
    if (state == UM_WAIT && now - state_ts_ > 350) {
        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();
        
        // UART Buffer leeren
        uint8_t dummy;
        while (this->available()) this->read_byte(&dummy);

        // SND_NKE (Reset)
        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        this->flush();
        
        state = UM_SEND;
        state_ts_ = now;
    }
  }

  // SWITCH UART
  if (state == UM_WAIT && now - state_ts_ > 350) {

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t dummy;
    while (this->available()) this->read_byte(&dummy);

    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = millis();
  }

  // REQUEST FULL DATA
  if (state == UM_SEND && millis() - state_ts_ > 50) {

    uint8_t req[] = {0x10, 0x7B, 0xFE, 0x79, 0x16};
    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    state = UM_RX;
    state_ts_ = millis();
    last_rx_byte = millis();
  }

  // RX
  if (state == UM_RX) {

    while (this->available()) {
      uint8_t c;
      if (this->read_byte(&c)) {
        rx_buffer_.push_back(c);
        last_rx_byte = millis();
      }
    // 3. REQUEST: Daten anfordern (REQ_UD2)
    if (state == UM_SEND && now - state_ts_ > 100) {
        uint8_t req[] = {0x10, 0x7B, 0xFE, 0x79, 0x16};
        this->write_array(req, sizeof(req));
        this->flush();
        
        state = UM_RX;
        state_ts_ = now;
        last_rx_byte_ = now;
    }

    // ⭐ Frame fertig wenn 1s keine neuen Bytes
    if (rx_buffer_.size() > 10 && millis() - last_rx_byte > 1000) {

      auto &f = rx_buffer_;

      ESP_LOGI(TAG, "Frame size: %d", f.size());

      for (size_t i = 0; i + 6 < f.size(); i++) {

        // SERIAL 0C78
        if (serial_number_ && f[i] == 0x0C && f[i + 1] == 0x78) {
          serial_number_->publish_state(decode_bcd(f, i + 2, 4));
    // 4. RX: Daten einlesen und parsen
    if (state == UM_RX) {
        while (this->available()) {
            uint8_t c;
            if (this->read_byte(&c)) {
                rx_buffer_.push_back(c);
                last_rx_byte_ = millis();
            }
        }

        // ENERGY 0406
        if (total_energy_ && f[i] == 0x04 && f[i + 1] == 0x06) {
          uint32_t v = f[i + 2] | (f[i + 3] << 8) | (f[i + 4] << 16) | (f[i + 5] << 24);
          total_energy_->publish_state(v / 1000.0f);
        // Frame fertig? (Timeout 1s oder Puffer voll)
        if (rx_buffer_.size() > 20 && millis() - last_rx_byte_ > 1000) {
            auto &f = rx_buffer_;
            ESP_LOGD(TAG, "Parsing Frame, Size: %d", f.size());

            for (size_t i = 0; i + 4 < f.size(); i++) {
                
                // SERIAL (0C 78 ...) - BCD 8-stellig
                if (serial_number_ && f[i] == 0x0C && f[i+1] == 0x78) {
                    serial_number_->publish_state(decode_bcd(f, i+2, 4));
                }
                
                // ENERGY (04 06 ...) - Binär, Wh zu kWh (Teiler 1000 oder 100 prüfen)
                if (total_energy_ && f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t val = (uint32_t)f[i+2] | (uint32_t)f[i+3] << 8 | (uint32_t)f[i+4] << 16 | (uint32_t)f[i+5] << 24;
                    total_energy_->publish_state(val * 0.01f); // Basierend auf Log: CC 5E -> 24268 -> 242.68 kWh
                }

                // VOLUME (0C 14 ...) - BCD 8-stellig
                if (total_volume_ && f[i] == 0x0C && f[i+1] == 0x14) {
                    total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                }

                // POWER (3B 2D ...) - Binär 3 Byte
                if (current_power_ && f[i] == 0x3B && f[i+1] == 0x2D) {
                    uint32_t p = (uint32_t)f[i+2] | (uint32_t)f[i+3] << 8 | (uint32_t)f[i+4] << 16;
                    if (p != 0x999999) current_power_->publish_state((float)p);
                }

                // FLOW TEMP (0A 5A ...) - Binär 2 Byte (0.1°C)
                if (temp_flow_ && f[i] == 0x0A && f[i+1] == 0x5A) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3] << 8;
                    temp_flow_->publish_state(t * 0.1f);
                }

                // RETURN TEMP (0A 5E ...) - Binär 2 Byte (0.1°C)
                if (temp_return_ && f[i] == 0x0A && f[i+1] == 0x5E) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3] << 8;
                    temp_return_->publish_state(t * 0.1f);
                }

                // DELTA T (0B 61 ...) - Binär 3 Byte (0.01 K)
                if (temp_diff_ && f[i] == 0x0B && f[i+1] == 0x61) {
                    uint32_t td = (uint32_t)f[i+2] | (uint32_t)f[i+3] << 8 | (uint32_t)f[i+4] << 16;
                    temp_diff_->publish_state(td * 0.01f);
                }
            }

            rx_buffer_.clear();
            state = UM_IDLE;
            ESP_LOGI(TAG, "Update finished.");
        }

        // VOLUME 0C14
        if (total_volume_ && f[i] == 0x0C && f[i + 1] == 0x14) {
          uint32_t v = f[i + 2] | (f[i + 3] << 8) | (f[i + 4] << 16) | (f[i + 5] << 24);
          total_volume_->publish_state(v / 1000.0f);
        if (now - state_ts_ > 15000) {
            ESP_LOGW(TAG, "RX Timeout - No valid frame received");
            rx_buffer_.clear();
            state = UM_IDLE;
        }

        // POWER 3B2D
        if (current_power_ && f[i] == 0x3B && f[i + 1] == 0x2D) {
          uint32_t p = f[i + 2] | (f[i + 3] << 8) | (f[i + 4] << 16);
          current_power_->publish_state((float)p);
        }

        // FLOW TEMP 0A5A
        if (temp_flow_ && f[i] == 0x0A && f[i + 1] == 0x5A) {
          uint16_t t = f[i + 2] | (f[i + 3] << 8);
          temp_flow_->publish_state(t / 100.0f);
        }

        // RETURN TEMP 0A5E
        if (temp_return_ && f[i] == 0x0A && f[i + 1] == 0x5E) {
          uint16_t t = f[i + 2] | (f[i + 3] << 8);
          temp_return_->publish_state(t / 100.0f);
        }

        // DELTA T 0B61
        if (temp_diff_ && f[i] == 0x0B && f[i + 1] == 0x61) {
          uint32_t t = f[i + 2] | (f[i + 3] << 8) | (f[i + 4] << 16);
          temp_diff_->publish_state(t / 100.0f);
        }

        // DATE/TIME 046D
        if (meter_time_ && f[i] == 0x04 && f[i + 1] == 0x6D) {
          char buf[32];
          sprintf(buf, "%02X%02X%02X%02X", f[i + 2], f[i + 3], f[i + 4], f[i + 5]);
          meter_time_->publish_state(buf);
        }
      }

      rx_buffer_.clear();
      state = UM_IDLE;
    }

    if (millis() - state_ts_ > 12000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

}  // namespace ultramaxx
