#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

float UltraMaXXComponent::decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len) {
    float value = 0;
    float mul = 1;
    for (size_t i = 0; i < len; i++) {
        if (start + i >= data.size()) break;
        uint8_t b = data[start + i];
        value += (b & 0x0F) * mul;
        mul *= 10;
        value += ((b >> 4) & 0x0F) * mul;
        mul *= 10;
    }
    return value;
}

void UltraMaXXComponent::setup() { ESP_LOGI(TAG, "UltraMaXX gestartet"); }

void UltraMaXXComponent::update() {
    ESP_LOGI(TAG, "=== UPDATE START ===");
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
        uint8_t dummy;
        while (this->available()) this->read_byte(&dummy);
        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        state = UM_SEND; state_ts_ = now;
    }

    if (state == UM_SEND && now - state_ts_ > 100) {
        uint8_t req[] = {0x10, 0x7B, 0xFE, 0x79, 0x16};
        this->write_array(req, sizeof(req));
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

        if (rx_buffer_.size() > 10 && millis() - last_rx_byte_ > 1000) {
            auto &f = rx_buffer_;
            
            // HEX-DUMP FÜR FEHLERSUCHE
            std::string hex;
            char buf[5];
            for (uint8_t b : f) { sprintf(buf, "%02X ", b); hex += buf; }
            ESP_LOGI(TAG, "Frame (L=%d): %s", f.size(), hex.c_str());

            for (size_t i = 0; i + 4 < f.size(); i++) {
                // SERIAL (0C 78)
                if (f[i] == 0x0C && f[i+1] == 0x78) {
                    float val = decode_bcd(f, i+2, 4);
                    if (serial_number_) { serial_number_->publish_state(val); ESP_LOGI(TAG, "SN OK: %.0f", val); }
                    else { ESP_LOGW(TAG, "SN Sensor fehlt in YAML!"); }
                }
                // ENERGIE (04 06)
                if (f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t v = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16 | (uint32_t)f[i+5]<<24;
                    if (total_energy_) { total_energy_->publish_state(v * 0.01f); ESP_LOGI(TAG, "Energy OK: %.2f", v * 0.01f); }
                    else { ESP_LOGW(TAG, "Energy Sensor fehlt in YAML!"); }
                }
                // VOLUMEN (0C 14)
                if (f[i] == 0x0C && f[i+1] == 0x14) {
                    float val = decode_bcd(f, i+2, 4) * 0.01f;
                    if (total_volume_) { total_volume_->publish_state(val); ESP_LOGI(TAG, "Volume OK: %.2f", val); }
                }
                // VORLAUF (0A 5A)
                if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3]<<8;
                    if (temp_flow_) { temp_flow_->publish_state(t * 0.1f); ESP_LOGI(TAG, "Flow OK: %.1f", t * 0.1f); }
                }
                // RÜCKLAUF (0A 5E)
                if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3]<<8;
                    if (temp_return_) { temp_return_->publish_state(t * 0.1f); ESP_LOGI(TAG, "Return OK: %.1f", t * 0.1f); }
                }
            }
            rx_buffer_.clear();
            state = UM_IDLE;
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
