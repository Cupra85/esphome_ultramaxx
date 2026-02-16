void UltraMaXXComponent::loop() {
    uint32_t now = millis();

    // --- SCHRITT 1: DATEN SOFORT SICHERN ---
    // Wir lesen IMMER, wenn Daten da sind, damit der Puffer nicht überläuft
    while (this->available()) {
        uint8_t c;
        if (this->read_byte(&c)) {
            if (state == UM_RX) {
                rx_buffer_.push_back(c);
                last_rx_byte_ = now; // Zeitstempel für "Stille-Erkennung"
            }
        }
    }

    // --- SCHRITT 2: ZUSTANDSMASCHINE ---
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

    if (state == UM_WAIT && now - state_ts_ > 350) {
        this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        this->parent_->load_settings();
        
        // Letzte Reste vom Wakeup-Echo löschen
        uint8_t dummy; while (this->available()) this->read_byte(&dummy);
        rx_buffer_.clear();

        uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
        this->write_array(reset, sizeof(reset));
        this->flush();
        state = UM_SEND;
        state_ts_ = now;
    }

    if (state == UM_SEND && now - state_ts_ > 150) {
        uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
        uint8_t cs = (ctrl + 0xFE) & 0xFF;
        uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
        this->write_array(req, sizeof(req));
        this->flush();
        
        fcb_toggle_ = !fcb_toggle_;
        state = UM_RX;
        state_ts_ = now;
        last_rx_byte_ = now;
        rx_buffer_.clear();
    }

    if (state == UM_RX) {
        // Wenn wir Daten haben und 600ms Ruhe herrscht (Frame-Ende)
        if (!rx_buffer_.empty() && (now - last_rx_byte_ > 600)) {
            auto &f = rx_buffer_;
            ESP_LOGI(TAG, "Parsing Frame (%d Bytes)...", f.size());

            for (size_t i = 0; i + 6 < f.size(); i++) {
                // Serial
                if (f[i] == 0x0C && f[i+1] == 0x78) {
                    if (serial_number_) serial_number_->publish_state(decode_bcd(f, i+2, 4));
                }
                // Energie (Teiler 0.001 für MWh laut Log/Script)
                else if (f[i] == 0x04 && f[i+1] == 0x06) {
                    uint32_t v = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16 | (uint32_t)f[i+5]<<24;
                    if (total_energy_) total_energy_->publish_state(v * 0.001f);
                }
                // Volumen
                else if (f[i] == 0x0C && f[i+1] == 0x14) {
                    if (total_volume_) total_volume_->publish_state(decode_bcd(f, i+2, 4) * 0.01f);
                }
                // Temperaturen
                else if (f[i] == 0x0A && f[i+1] == 0x5A) {
                    if (temp_flow_) temp_flow_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                else if (f[i] == 0x0A && f[i+1] == 0x5E) {
                    if (temp_return_) temp_return_->publish_state(decode_bcd(f, i+2, 2) * 0.1f);
                }
                // Zeit (BCD Typ F)
                else if (f[i] == 0x04 && f[i+1] == 0x6D && meter_time_) {
                    char time_buf[20];
                    sprintf(time_buf, "20%02X-%02X-%02X %02X:%02X", f[i+5]&0x3F, f[i+4]&0x0F, f[i+3]&0x1F, f[i+3]>>5, f[i+2]&0x3F);
                    meter_time_->publish_state(time_buf);
                }
            }
            rx_buffer_.clear();
            state = UM_IDLE;
        }

        if (now - state_ts_ > 10000) {
            rx_buffer_.clear();
            state = UM_IDLE;
        }
    }
}
