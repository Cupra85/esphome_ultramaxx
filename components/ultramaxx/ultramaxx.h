#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;
static uint32_t last_rx_byte = 0;

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
    float value = 0;
    float mul = 1;
    for (size_t i = 0; i < len; i++) {
        if (start + i >= data.size()) break;
        uint8_t b = data[start + i];
        value += (b & 0x0F) * mul; mul *= 10;
        value += ((b >> 4) & 0x0F) * mul; mul *= 10;
    }
    return value;
}

void UltraMaXXComponent::setup() {
    ESP_LOGI(TAG, "UltraMaXX Component gestartet");
}

void UltraMaXXComponent::update() {
    ESP_LOGI(TAG, "=== READ START ===");
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

    // 1. WAKEUP
    if (state == UM_WAKEUP) {
        if (now - last_send_ > 20) {
            uint8_t buf[20];
            memset(buf, 0x55, 20);
            this->write_array(buf, 20);
            last_send_ = now;
        }
        if (now - wake_start_ > 2200) {
            state = UM_WAIT;
            state_ts_ = now;
        }
    }

    // 2. SWITCH UART & SEND SND_NKE
    if (state == UM_WAIT && now - state_ts_ > 400) {
        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();
        
        // Kleine Pause damit UART stabil ist
        delay(50); 
        
        uint8_t dummy;
        while (this->available()) this->read_byte(&dummy);

        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        this->flush();
        
        ESP_LOGI(TAG, "SND_NKE gesendet");
        state = UM_SEND;
        state_ts_ = now;
    }

    // 3. SEND REQ_UD2
    if (state == UM_SEND && now - state_ts_ > 200) {
        uint8_t req[] = {0x10, 0x7B, 0xFE, 0x79, 0x16};
        this->write_array(req, sizeof(req));
        this->flush();
        
        ESP_LOGI(TAG, "REQ_UD2 gesendet");
        rx_buffer_.clear();
        state = UM_RX;
        state_ts_ = now;
        last_rx_byte = now;
    }

    // 4. RX & PARSING
    if (state == UM_RX) {
        while (this->available()) {
            uint8_t c;
            if (this->read_byte(&c)) {
                rx_buffer_.push_back(c);
                last_rx_byte = millis();
            }
        }

        // Frame fertig wenn 1s keine neuen Bytes und Mindestgröße
        if (rx_buffer_.size() > 30 && now - last_rx_byte > 1000) {
            auto &f = rx_buffer_;
            ESP_LOGI(TAG, "Frame erhalten (%d Bytes). Starte Parsing...", f.size());

            // SIGNATUR-SUCHE laut Tasmota Script
            for (size_t i = 0; i + 8 < f.size(); i++) {
                // Serial 0C 78
                if (f[i] == 0x0C && f[i+1] == 0x78) {
                    if (serial_number_) serial_number_->publish_state(decode_bcd(f, i+2, 4));
                }
                // Energie 04 06 (MWh laut script, wir nehmen kWh)
                else if (f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t v = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16 | (uint32_t)f[i+5]<<24;
                    if (total_energy_) total_energy_->publish_state(v * 0.001f);
                }
                // Volumen 0C 14 (BCD)
                else if (f[i] == 0x0C && f[i+1] == 0x14) {
                    if (total_volume_) total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                }
                // Vorlauf 0A 5A (BCD)
                else if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    if (temp_flow_) temp_flow_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                // Rücklauf 0A 5E (BCD)
                else if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    if (temp_return_) temp_return_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
            }
            rx_buffer_.clear();
            state = UM_IDLE;
        }

        if (now - state_ts_ > 10000) {
            ESP_LOGW(TAG, "RX Timeout - Keine Daten empfangen");
            rx_buffer_.clear();
            state = UM_IDLE;
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
