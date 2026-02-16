#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;
static bool fcb_toggle = false;

// Korrekte Signatur mit const passend zur .h
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
    ESP_LOGI(TAG, "=== UPDATE START (FCB: %d) ===", fcb_toggle);
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

    // 1. WAKEUP SEQUENZ (2,2 Sek 0x55)
    if (state == UM_WAKEUP) {
        if (now - last_send_ > 15) {
            uint8_t b = 0x55;
            this->write_array(&b, 1);
            last_send_ = now;
        }
        if (now - wake_start_ > 2200) {
            state = UM_WAIT;
            state_ts_ = now;
        }
    }

    // 2. SWITCH ZU 8E1 & PUFFER REINIGEN
    if (state == UM_WAIT && now - state_ts_ > 350) {
        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();
        
        uint8_t d; 
        while(this->available()) this->read_byte(&d);
        rx_buffer_.clear();

        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        state = UM_SEND;
        state_ts_ = now;
    }

    // 3. DATEN ANFORDERN (REQ_UD2) mit FCB Toggle
    if (state == UM_SEND && now - state_ts_ > 100) {
        uint8_t ctrl = fcb_toggle ? 0x7B : 0x5B;
        uint8_t cs = (ctrl + 0xFE) & 0xFF;
        uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
        this->write_array(req, sizeof(req));
        this->flush();
        fcb_toggle = !fcb_toggle; // Toggelt für nächsten Poll

        state = UM_RX;
        state_ts_ = now;
        last_rx_byte_ = now;
    }

    // 4. RX & PARSING
    if (state == UM_RX) {
        while (this->available()) {
            uint8_t c;
            if (this->read_byte(&c)) {
                rx_buffer_.push_back(c);
                last_rx_byte_ = millis();
            }
        }

        // Frame verarbeiten wenn 1s Stille oder Puffer signifikant gefüllt
        if (rx_buffer_.size() > 40 && (now - last_rx_byte_ > 1000)) {
            auto &f = rx_buffer_;
            ESP_LOGI(TAG, "Parsing %d Bytes...", f.size());

            for (size_t i = 0; i + 6 < f.size(); i++) {
                // SERIAL (0C 78) - 4 Byte BCD
                if (f[i] == 0x0C && f[i+1] == 0x78) {
                    if (serial_number_) serial_number_->publish_state(decode_bcd(f, i+2, 4));
                }
                // ENERGIE (04 06) - 4 Byte Integer (uuUUuuUU) laut Script
                else if (f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t v = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16 | (uint32_t)f[i+5]<<24;
                    if (total_energy_) total_energy_->publish_state(v * 0.001f);
                }
                // VOLUMEN (0C 14) - 4 Byte BCD
                else if (f[i] == 0x0C && f[i+1] == 0x14) {
                    if (total_volume_) total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                }
                // VORLAUF (0A 5A) - 2 Byte BCD
                else if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    if (temp_flow_) temp_flow_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                // RÜCKLAUF (0A 5E) - 2 Byte BCD
                else if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    if (temp_return_) temp_return_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                // DELTA T (0B 61) - 3 Byte BCD
                else if (f[i] == 0x0B && f[i+1] == 0x61) {
                    if (temp_diff_) temp_diff_->publish_state(decode_bcd(f, i+2, 3) * 0.01f);
                }
                // LEISTUNG (0B 2D) - 3 Byte BCD
                else if (f[i] == 0x0B && f[i+1] == 0x2D) {
                    if (current_power_) current_power_->publish_state(decode_bcd(f, i+2, 3) * 0.1f);
                }
            }
            rx_buffer_.clear();
            state = UM_IDLE;
            ESP_LOGI(TAG, "Update Zyklus abgeschlossen.");
        }

        if (now - state_ts_ > 15000) {
            ESP_LOGW(TAG, "RX Timeout - Puffer gelöscht");
            rx_buffer_.clear();
            state = UM_IDLE;
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
