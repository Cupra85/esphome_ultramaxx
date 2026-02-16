#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;
static bool fcb_bit = false; // Das FCB-Bit für REQ_UD2

float UltraMaXXComponent::decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len) {
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

void UltraMaXXComponent::update() {
    ESP_LOGI(TAG, "=== START READ (FCB: %d) ===", fcb_bit);
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

    if (state == UM_WAKEUP) {
        if (now - last_send_ > 15) {
            uint8_t b = 0x55;
            this->write_array(&b, 1);
            last_send_ = now;
        }
        if (now - wake_start_ > 2200) {
            state = UM_WAIT; state_ts_ = now;
        }
    }

    if (state == UM_WAIT && now - state_ts_ > 350) {
        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();
        
        // Puffer leeren
        uint8_t dummy;
        while (this->available()) this->read_byte(&dummy);
        rx_buffer_.clear();

        // SND_NKE (Reset)
        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        state = UM_SEND; state_ts_ = now;
    }

    if (state == UM_SEND && now - state_ts_ > 100) {
        // REQ_UD2 mit FCB-Toggling: 0x5B (FCB=0) oder 0x7B (FCB=1)
        uint8_t control_byte = fcb_bit ? 0x7B : 0x5B;
        uint8_t checksum = (control_byte + 0xFE) & 0xFF;
        uint8_t req[] = {0x10, control_byte, 0xFE, checksum, 0x16};
        
        this->write_array(req, sizeof(req));
        fcb_bit = !fcb_bit; // Toggelt für das nächste Mal
        
        state = UM_RX; state_ts_ = now; last_rx_byte_ = now;
    }

    if (state == UM_RX) {
        while (this->available()) {
            uint8_t c;
            if (this->read_byte(&c)) {
                rx_buffer_.push_back(c);
                last_rx_byte_ = millis();
            }
        }

        if (rx_buffer_.size() > 20 && millis() - last_rx_byte_ > 1000) {
            auto &f = rx_buffer_;
            ESP_LOGI(TAG, "Parsing %d Bytes...", f.size());

            for (size_t i = 0; i + 5 < f.size(); i++) {
                // Serial (0C 78)
                if (f[i] == 0x0C && f[i+1] == 0x78) {
                    if (serial_number_) serial_number_->publish_state(decode_bcd(f, i+2, 4));
                }
                // Energy (04 06 oder 0C 06)
                else if ((f[i] == 0x04 || f[i] == 0x0C) && f[i+1] == 0x06) {
                    if (total_energy_) total_energy_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                }
                // Volume (04 13, 04 14 oder 0C 14)
                else if ((f[i] == 0x04 || f[i] == 0x0C) && (f[i+1] == 0x13 || f[i+1] == 0x14)) {
                    if (total_volume_) total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                }
                // Temps (Binär 0A 5A / 0A 5E)
                else if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3] << 8;
                    if (temp_flow_) temp_flow_->publish_state(t * 0.1f);
                }
                else if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3] << 8;
                    if (temp_return_) temp_return_->publish_state(t * 0.1f);
                }
            }
            rx_buffer_.clear();
            state = UM_IDLE;
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
