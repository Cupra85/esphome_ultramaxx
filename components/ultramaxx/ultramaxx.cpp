#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;
static bool fcb_toggle = false;

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
    ESP_LOGI(TAG, "=== START READ (FCB: %d) ===", fcb_toggle);
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
        if (now - wake_start_ > 2200) { state = UM_WAIT; state_ts_ = now; }
    }

    if (state == UM_WAIT && now - state_ts_ > 350) {
        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();
        uint8_t d; while(this->available()) this->read_byte(&d);
        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        state = UM_SEND; state_ts_ = now;
    }

    if (state == UM_SEND && now - state_ts_ > 100) {
        uint8_t ctrl = fcb_toggle ? 0x7B : 0x5B;
        uint8_t cs = (ctrl + 0xFE) & 0xFF;
        uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
        this->write_array(req, sizeof(req));
        fcb_toggle = !fcb_toggle;
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

        if (rx_buffer_.size() > 50 && (now - last_rx_byte_ > 1000)) {
            auto &f = rx_buffer_;
            ESP_LOGI(TAG, "Parsing Frame (%d Bytes)...", f.size());

            for (size_t i = 0; i + 10 < f.size(); i++) {
                // 1. Seriennummer: 0C 78 -> direkt 4 Byte BCD
                if (f[i] == 0x0C && f[i+1] == 0x78) {
                    if (serial_number_) serial_number_->publish_state(decode_bcd(f, i+2, 4));
                }
                // 2. Energie: 04 06 -> Tasmota sagt: 4 Byte Integer (uuUUuuUU)
                if (f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t v = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16 | (uint32_t)f[i+5]<<24;
                    if (total_energy_) total_energy_->publish_state(v * 0.001f); // MWh laut Script
                }
                // 3. Volumen: 0C 14 -> 4 Byte BCD
                if (f[i] == 0x0C && f[i+1] == 0x14) {
                    if (total_volume_) total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                }
                // 4. Leistung: 0B 2D -> 3 Byte BCD
                if (f[i] == 0x0B && f[i+1] == 0x2D) {
                    if (current_power_) current_power_->publish_state(decode_bcd(f, i+2, 3) * 0.1f);
                }
                // 5. Vorlauf: 0A 5A -> 2 Byte BCD
                if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    if (temp_flow_) temp_flow_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                // 6. Rücklauf: 0A 5E -> 2 Byte BCD
                if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    if (temp_return_) temp_return_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                // 7. Delta T: 0B 61 -> 3 Byte BCD
                if (f[i] == 0x0B && f[i+1] == 0x61) {
                    if (temp_diff_) temp_diff_->publish_state(decode_bcd(f, i+2, 3) * 0.01f);
                }
            }
            rx_buffer_.clear();
            state = UM_IDLE;
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
