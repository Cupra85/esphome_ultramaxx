#include "ultramaxx.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ultramaxx {
@@ -26,7 +27,7 @@
}

void UltraMaXXComponent::update() {
    ESP_LOGI(TAG, "Starting Update Cycle...");
    ESP_LOGI(TAG, "=== READ START ===");
    this->parent_->set_baud_rate(2400);
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
    this->parent_->load_settings();
@@ -40,32 +41,33 @@
void UltraMaXXComponent::loop() {
    uint32_t now = millis();

    // 1. WAKEUP: 2,2 Sekunden 0x55 senden (2400 Baud)
    // 1. WAKEUP: 2,2s lang 0x55 senden
    if (state == UM_WAKEUP) {
        if (now - last_send_ > 15) {
            uint8_t wakeup_byte = 0x55;
            this->write_array(&wakeup_byte, 1);
            last_send_ = now;
        }
        if (now - wake_start_ > 2200) {
            ESP_LOGI(TAG, "Wakeup end");
            state = UM_WAIT;
            state_ts_ = now;
        }
    }

    // 2. SWITCH: Wechsel auf 2400 8E1 (Even Parity)
    // 2. SWITCH: Wechsel auf 2400 8E1 und Reset (SND_NKE)
    if (state == UM_WAIT && now - state_ts_ > 350) {
        ESP_LOGI(TAG, "Switch to 2400 8E1");
        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();

        // UART Buffer leeren
        uint8_t dummy;
        while (this->available()) this->read_byte(&dummy);

        // SND_NKE (Reset)
        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        this->flush();
        ESP_LOGI(TAG, "SND_NKE gesendet");

        state = UM_SEND;
        state_ts_ = now;
@@ -76,13 +78,14 @@
        uint8_t req[] = {0x10, 0x7B, 0xFE, 0x79, 0x16};
        this->write_array(req, sizeof(req));
        this->flush();
        ESP_LOGI(TAG, "REQ_UD2 gesendet");

        state = UM_RX;
        state_ts_ = now;
        last_rx_byte_ = now;
    }

    // 4. RX: Daten einlesen und parsen
    // 4. RX & PARSING
    if (state == UM_RX) {
        while (this->available()) {
            uint8_t c;
@@ -92,66 +95,85 @@
            }
        }

        // Frame fertig? (Timeout 1s oder Puffer voll)
        // Wenn 1s keine Daten mehr kommen, interpretieren wir den Frame als fertig
        if (rx_buffer_.size() > 20 && millis() - last_rx_byte_ > 1000) {
            auto &f = rx_buffer_;
            ESP_LOGD(TAG, "Parsing Frame, Size: %d", f.size());
            
            // Log den Frame als Hex-Kette für das Debugging
            char hex_buf[4];
            std::string log_msg = "Frame Hex: ";
            for (uint8_t b : f) {
                sprintf(hex_buf, "%02X ", b);
                log_msg += hex_buf;
            }
            ESP_LOGD(TAG, "%s", log_msg.c_str());

            for (size_t i = 0; i + 4 < f.size(); i++) {

                // SERIAL (0C 78 ...) - BCD 8-stellig
                // SERIAL (0C 78) - BCD
                if (serial_number_ && f[i] == 0x0C && f[i+1] == 0x78) {
                    serial_number_->publish_state(decode_bcd(f, i+2, 4));
                    float sn = decode_bcd(f, i+2, 4);
                    serial_number_->publish_state(sn);
                    ESP_LOGD(TAG, "Match: Serial %.0f", sn);
                }

                // ENERGY (04 06 ...) - Binär, Wh zu kWh (Teiler 1000 oder 100 prüfen)
                // ENERGY (04 06) - Binär (Wh oder kWh je nach Gerät)
                if (total_energy_ && f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t val = (uint32_t)f[i+2] | (uint32_t)f[i+3] << 8 | (uint32_t)f[i+4] << 16 | (uint32_t)f[i+5] << 24;
                    total_energy_->publish_state(val * 0.01f); // Basierend auf Log: CC 5E -> 24268 -> 242.68 kWh
                    total_energy_->publish_state(val * 0.01f);
                    ESP_LOGD(TAG, "Match: Energy %.2f", val * 0.01f);
                }

                // VOLUME (0C 14 ...) - BCD 8-stellig
                // VOLUME (0C 14) - BCD
                if (total_volume_ && f[i] == 0x0C && f[i+1] == 0x14) {
                    total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                    float vol = decode_bcd(f, i+2, 4) * 0.01f;
                    total_volume_->publish_state(vol);
                    ESP_LOGD(TAG, "Match: Volume %.2f", vol);
                }

                // POWER (3B 2D ...) - Binär 3 Byte
                // POWER (3B 2D) - Binär
                if (current_power_ && f[i] == 0x3B && f[i+1] == 0x2D) {
                    uint32_t p = (uint32_t)f[i+2] | (uint32_t)f[i+3] << 8 | (uint32_t)f[i+4] << 16;
                    if (p != 0x999999) current_power_->publish_state((float)p);
                    if (p != 0x999999) {
                        current_power_->publish_state((float)p);
                        ESP_LOGD(TAG, "Match: Power %.0f", (float)p);
                    }
                }

                // FLOW TEMP (0A 5A ...) - Binär 2 Byte (0.1°C)
                // FLOW TEMP (0A 5A) - Binär (0.1°C)
                if (temp_flow_ && f[i] == 0x0A && f[i+1] == 0x5A) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3] << 8;
                    temp_flow_->publish_state(t * 0.1f);
                    ESP_LOGD(TAG, "Match: Flow Temp %.1f", t * 0.1f);
                }

                // RETURN TEMP (0A 5E ...) - Binär 2 Byte (0.1°C)
                // RETURN TEMP (0A 5E) - Binär (0.1°C)
                if (temp_return_ && f[i] == 0x0A && f[i+1] == 0x5E) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3] << 8;
                    temp_return_->publish_state(t * 0.1f);
                    ESP_LOGD(TAG, "Match: Return Temp %.1f", t * 0.1f);
                }

                // DELTA T (0B 61 ...) - Binär 3 Byte (0.01 K)
                // DELTA T (0B 61) - Binär (0.01 K)
                if (temp_diff_ && f[i] == 0x0B && f[i+1] == 0x61) {
                    uint32_t td = (uint32_t)f[i+2] | (uint32_t)f[i+3] << 8 | (uint32_t)f[i+4] << 16;
                    temp_diff_->publish_state(td * 0.01f);
                    ESP_LOGD(TAG, "Match: Delta T %.2f", td * 0.01f);
                }
            }

            rx_buffer_.clear();
            state = UM_IDLE;
            ESP_LOGI(TAG, "Update finished.");
            ESP_LOGI(TAG, "Reading cycle finished.");
        }

        if (now - state_ts_ > 15000) {
        if (now - state_ts_ > 12000) {
            ESP_LOGW(TAG, "RX Timeout - No valid frame received");
            rx_buffer_.clear();
            state = UM_IDLE;
        }
    }
}

}  // namespace ultramaxx
}  // namespace esphome
