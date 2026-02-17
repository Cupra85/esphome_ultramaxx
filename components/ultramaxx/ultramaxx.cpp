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
        uint8_t b = data[start + i];
        value += (b & 0x0F) * mul; mul *= 10;
        value += ((b >> 4) & 0x0F) * mul; mul *= 10;
    }
    return value;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &d, size_t s, size_t l) {
    uint32_t v = 0;
    for (size_t i = 0; i < l; i++) v |= ((uint32_t)d[s + i]) << (8 * i);
    return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &, size_t, std::string &) {
    return false; // optional
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

    // ===== RX IMMER SAMMELN =====
    while (this->available()) {
        uint8_t c;
        if (this->read_byte(&c)) {
            if (state == UM_RX) {
                rx_buffer_.push_back(c);
            }
        }
    }

    // ===== WAKEUP =====
    if (state == UM_WAKEUP) {
        if (now - last_send_ > 15) {
            uint8_t buf[20]; memset(buf, 0x55, 20);
            this->write_array(buf, 20);
            last_send_ = now;
        }
        if (now - wake_start_ > 2200) {
            ESP_LOGI(TAG, "Wakeup end");
            state = UM_WAIT;
            state_ts_ = now;
        }
    }

    // ===== SWITCH UART =====
    if (state == UM_WAIT && now - state_ts_ > 350) {

        ESP_LOGI(TAG, "Switch to 2400 8E1");

        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();

        uint8_t d;
        while (this->available()) this->read_byte(&d);

        uint8_t reset[] = {0x10,0x40,0xFE,0x3E,0x16};
        this->write_array(reset, sizeof(reset));
        this->flush();

        ESP_LOGI(TAG, "SND_NKE gesendet");

        state = UM_SEND;
        state_ts_ = now;
    }

    // ===== REQ_UD2 =====
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
    }

    // ===== FRAME EXTRACTOR (FIX!) =====
    if (state == UM_RX && rx_buffer_.size() > 6) {

        for (size_t i = 0; i + 6 < rx_buffer_.size(); i++) {

            if (rx_buffer_[i] == 0x68 && rx_buffer_[i+3] == 0x68) {

                uint8_t len = rx_buffer_[i+1];
                size_t frame_end = i + 4 + len + 2; // header + data + CS+16

                if (frame_end <= rx_buffer_.size() &&
                    rx_buffer_[frame_end-1] == 0x16) {

                    ESP_LOGI(TAG,"Parsing Frame (%d bytes)", len);

                    auto &f = rx_buffer_;

                    for (size_t p = i; p < frame_end; p++) {

                        if (p+6 >= frame_end) break;

                        if (f[p]==0x0C && f[p+1]==0x78 && serial_number_)
                            serial_number_->publish_state(decode_bcd(f,p+2,4));

                        else if (f[p]==0x04 && f[p+1]==0x06 && total_energy_)
                            total_energy_->publish_state(decode_u_le(f,p+2,4)*0.001f);

                        else if (f[p]==0x0C && f[p+1]==0x14 && total_volume_)
                            total_volume_->publish_state(decode_bcd(f,p+2,4)*0.01f);

                        else if (f[p]==0x0A && f[p+1]==0x5A && temp_flow_)
                            temp_flow_->publish_state(decode_bcd(f,p+2,2)*0.1f);

                        else if (f[p]==0x0A && f[p+1]==0x5E && temp_return_)
                            temp_return_->publish_state(decode_bcd(f,p+2,2)*0.1f);

                        else if (f[p]==0x0B && f[p+1]==0x61 && temp_diff_)
                            temp_diff_->publish_state(decode_bcd(f,p+2,3)*0.01f);
                    }

                    rx_buffer_.clear();
                    state = UM_IDLE;
                    ESP_LOGI(TAG,"Update finished.");
                    break;
                }
            }
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
