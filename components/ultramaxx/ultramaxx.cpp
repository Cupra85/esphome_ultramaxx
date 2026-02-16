#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

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
    ESP_LOGI(TAG, "UltraMaXX component started");
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

    // --- SCHRITT 1: DATEN-SICHERUNG (PRIORISIERT) ---
    // Wir lesen IMMER, wenn Daten da sind, damit der UART-Buffer nicht überläuft
    while (this->available()) {
        uint8_t c;
        if (this->read_byte(&c)) {
            if (state == UM_RX) {
                rx_buffer_.push_back(c);
                last_rx_byte_ = now;
            }
        }
    }

    // --- SCHRITT 2: ZUSTANDSMASCHINE ---

    // WAKEUP: Deine originale Sequenz (20-Byte Blöcke)
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

    // SWITCH: Wechsel auf 8E1 und SND_NKE
    if (state == UM_WAIT && now - state_ts_ > 350) {
        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();
        
        // Puffer leeren (Echo vom Wakeup)
        uint8_t dummy; while (this->available()) this->read_byte(&dummy);
        rx_buffer_.clear();

        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        this->flush();
        ESP_LOGI(TAG, "SND_NKE gesendet");
        state = UM_SEND;
        state_ts_ = now;
    }

    // REQUEST: Datenanforderung REQ_UD2
    if (state == UM_SEND && now - state_ts_ > 150) {
        uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
        uint8_t cs = (ctrl + 0xFE) & 0xFF;
        uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
        this->write_array(req, sizeof(req));
        this->flush();
        
        fcb_toggle_ = !fcb_toggle_;
        ESP_LOGI(TAG, "REQ_UD2 gesendet");
        state = UM_RX;
        state_ts_ = now;
        last_rx_byte_ = now;
    }

    // RX & PARSING
    if (state == UM_RX) {
        // Trigger: 800ms Stille ODER End-Byte 0x16 bei Mindestlänge
        if (!rx_buffer_.empty() && (now - last_rx_byte_ > 800 || (rx_buffer_.size() > 70 && rx_buffer_.back() == 0x16))) {
            auto &f = rx_buffer_;
            ESP_LOGI(TAG, "Parsing Frame (%d Bytes)...", f.size());

            for (size_t i = 0; i + 6 < f.size(); i++) {
                // Serial (0C 78) - BCD
                if (f[i] == 0x0C && f[i+1] == 0x78) {
                    if (serial_number_) serial_number_->publish_state(decode_bcd(f, i+2, 4));
                }
                // Energy (04 06) - 4 Byte Integer (MWh)
                else if (f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t v = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16 | (uint32_t)f[i+5]<<24;
                    if (total_energy_) total_energy_->publish_state(v * 0.001f);
                }
                // Volume (0C 14) - BCD
                else if (f[i] == 0x0C && f[i+1] == 0x14) {
                    if (total_volume_) total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                }
                // Vorlauf (0A 5A) - BCD (Log: 47 02 -> 24.7)
                else if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    if (temp_flow_) temp_flow_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                // Rücklauf (0A 5E) - BCD (Log: 36 02 -> 23.6)
                else if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    if (temp_return_) temp_return_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                // Delta T (0B 61) - BCD
                else if (f[i] == 0x0B && f[i+1] == 0x61) {
                    if (temp_diff_) temp_diff_->publish_state(decode_bcd(f, i+2, 3) * 0.01f);
                }
                // Meter Time (04 6D)
                else if (f[i] == 0x04 && f[i+1] == 0x6D && meter_time_) {
                    char time_buf[20];
                    sprintf(time_buf, "20%02X-%02X-%02X %02X:%02X", f[i+5]&0x3F, f[i+4]&0x0F, f[i+3]&0x1F, f[i+3]>>5, f[i+2]&0x3F);
                    meter_time_->publish_state(time_buf);
                }
            }
            rx_buffer_.clear();
            state = UM_IDLE;
            ESP_LOGI(TAG, "Update finished.");
        }

        if (now - state_ts_ > 10000) {
            ESP_LOGW(TAG, "RX Timeout - Frame incomplete (%d Bytes)", rx_buffer_.size());
            rx_buffer_.clear();
            state = UM_IDLE;
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
