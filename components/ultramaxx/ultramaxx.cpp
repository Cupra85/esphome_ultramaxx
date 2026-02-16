#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// Die BCD-Funktion muss robust sein
float UltraMaXXComponent::decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len) {
    float value = 0;
    float mul = 1;
    for (size_t i = 0; i < len; i++) {
        if (start + i >= data.size()) break;
        uint8_t b = data[start + i];
        value += (b & 0x0F) * mul;
        mul *= 10;
@@ -23,10 +23,11 @@
}

void UltraMaXXComponent::setup() {
    ESP_LOGI(TAG, "UltraMaXX gestartet");
    ESP_LOGI(TAG, "UltraMaXX Component gestartet");
}

void UltraMaXXComponent::update() {
    ESP_LOGI(TAG, "=== UPDATE START ===");
    this->parent_->set_baud_rate(2400);
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
    this->parent_->load_settings();
@@ -79,52 +80,58 @@
            }
        }

        // PARSING LOGIK laut mikrocontroller.net
        // Frame fertig (nach 1 Sekunde Stille)
        if (rx_buffer_.size() > 20 && millis() - last_rx_byte_ > 1000) {
            auto &f = rx_buffer_;
            
            ESP_LOGI(TAG, "Frame mit %d Bytes wird geparst...", f.size());

            for (size_t i = 0; i + 4 < f.size(); i++) {
                // 1. Seriennummer (0C 78) - 8-stellig BCD
                // 1. Seriennummer: 0C 78 (4 Byte BCD)
                if (f[i] == 0x0C && f[i+1] == 0x78) {
                    if (serial_number_) serial_number_->publish_state(decode_bcd(f, i+2, 4));
                }
                
                // 2. Energie (04 06) -> Eigentlich oft 0C 06 beim Allmess für BCD!
                // Laut deinem Log ist es aber 04 06. Wir versuchen beides:
                if ((f[i] == 0x04 || f[i] == 0x0C) && f[i+1] == 0x06) {
                    float en = decode_bcd(f, i+2, 4); 
                    if (total_energy_) total_energy_->publish_state(en * 0.01f);
                // 2. Energie: 04 06 (4 Byte Binär oder BCD, wir nehmen Binär laut Log CC 5E)
                else if (f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t v = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16 | (uint32_t)f[i+5]<<24;
                    if (total_energy_) total_energy_->publish_state(v * 0.01f);
                    ESP_LOGD(TAG, "Energie gefunden");
                }

                // 3. Volumen (04 13 oder 0C 14)
                if ((f[i] == 0x04 || f[i] == 0x0C) && (f[i+1] == 0x13 || f[i+1] == 0x14)) {
                    float vol = decode_bcd(f, i+2, 4);
                    if (total_volume_) total_volume_->publish_state(vol * 0.01f);
                // 3. Volumen: 0C 14 (4 Byte BCD)
                else if (f[i] == 0x0C && f[i+1] == 0x14) {
                    if (total_volume_) total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                    ESP_LOGD(TAG, "Volumen gefunden");
                }

                // 4. Temperaturen (02 5A / 02 5E für 2-byte BCD oder 0A für Binär)
                // Dein Log zeigt 0A 5A -> Das IST binär!
                if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3] << 8;
                // 4. Vorlauf: 0A 5A (2 Byte Binär)
                else if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3]<<8;
                    if (temp_flow_) temp_flow_->publish_state(t * 0.1f);
                    ESP_LOGD(TAG, "Vorlauf gefunden");
                }
                if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3] << 8;
                // 5. Rücklauf: 0A 5E (2 Byte Binär)
                else if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    int16_t t = (int16_t)f[i+2] | (int16_t)f[i+3]<<8;
                    if (temp_return_) temp_return_->publish_state(t * 0.1f);
                    ESP_LOGD(TAG, "Rücklauf gefunden");
                }

                // 5. Leistung (02 2B oder 3B 2D)
                if (f[i] == 0x3B && f[i+1] == 0x2D) {
                    // Hier ist es oft 6-stellig BCD
                    float p = decode_bcd(f, i+2, 3);
                    if (current_power_) current_power_->publish_state(p);
                // 6. Delta T: 0B 61 (3 Byte Binär)
                else if (f[i] == 0x0B && f[i+1] == 0x61) {
                    uint32_t td = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16;
                    if (temp_diff_) temp_diff_->publish_state(td * 0.01f);
                    ESP_LOGD(TAG, "DeltaT gefunden");
                }
                // 7. Leistung: 3B 2D (3 Byte Binär)
                else if (f[i] == 0x3B && f[i+1] == 0x2D) {
                    uint32_t p = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16;
                    if (current_power_ && p != 0x999999) current_power_->publish_state((float)p);
                    ESP_LOGD(TAG, "Leistung gefunden");
                }
            }
            rx_buffer_.clear();
            state = UM_IDLE;
            ESP_LOGI(TAG, "Parsing abgeschlossen.");
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
