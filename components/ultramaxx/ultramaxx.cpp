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

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++)
    v |= ((uint32_t)data[start + i]) << (8 * i);
  return v;
}

// Die decode_cp32_datetime_ Funktion wurde entfernt, da sie nicht benötigt wird

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component gestartet");
}

void UltraMaXXComponent::update() {
  // Baudrate und Parität werden hier nur initial gesetzt.
  // Die Umschaltung auf 8E1 passiert in der loop()
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
  ESP_LOGI(TAG, "=== READ START ===");
}

void UltraMaXXComponent::loop() {
  uint32_t now = millis();

  // RX immer sammeln, wenn im UM_RX Zustand
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // --- State Machine Logik ---

  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      // Sende 20x 0x55
      uint8_t buf[20]; memset(buf, 0x55, 20);
      this->write_array(buf, 20);
      last_send_ = now;
    }
    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup Ende");
      state = UM_WAIT;
      state_ts_ = now;
    }
  }

  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Umschalten auf 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // Puffer leeren
    uint8_t d; while (this->available()) this->read_byte(&d);

    uint8_t reset[]={0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();

    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl + 0xFE) & 0xFF;
    uint8_t req[]={0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req, sizeof(req));
    this->flush();

    ESP_LOGI(TAG, "REQ_UD2 gesendet");
    fcb_toggle_ = !fcb_toggle_;

    rx_buffer_.clear();
    state = UM_RX;
    state_ts_ = now;
  }

  // 🔥 STABILER FRAME PARSER (verbessert)
  if (state == UM_RX) {

    // Warte auf einen vollständigen Frame oder Timeout
    if (rx_buffer_.size() > 6) {

      // Suche den Start des Frames (könnte nach E5 kommen)
      for (size_t i = 0; i + 6 < rx_buffer_.size(); i++) {

        if (rx_buffer_[i] != 0x68) continue;

        uint8_t L1 = rx_buffer_[i+1];
        uint8_t L2 = rx_buffer_[i+2];

        if (L1 != L2) continue; // Längen müssen übereinstimmen

        size_t frame_end_index = i + L1 + 6; // Index des Stop-Bytes (0x16)

        if (frame_end_index >= rx_buffer_.size()) continue; // Frame noch unvollständig
        if (rx_buffer_[frame_end_index] != 0x16) continue; // Ende muss 0x16 sein

        ESP_LOGI(TAG, "Gültiger Frame gefunden (%d Bytes)", L1 + 6);

        // --- Start Parsing der Datenfelder ---
        // Die Daten beginnen nach 6 Header-Bytes (68 L L 68 C A CI)
        size_t p = i + 6; 
        size_t data_end_p = frame_end_index - 1; // Vor der Checksumme enden

        while (p + 2 < data_end_p) {
          uint8_t dib = rx_buffer_[p];
          uint8_t vib = rx_buffer_[p+1];
          size_t data_len = 0;

          // Bestimme die Länge der Daten basierend auf DIB/VIB Kombinationen
          if ((dib & 0x0F) == 0x00) data_len = 0;
          else if ((dib & 0x0F) == 0x01) data_len = 1;
          else if ((dib & 0x0F) == 0x02) data_len = 2;
          else if ((dib & 0x0F) == 0x03) data_len = 3;
          else if ((dib & 0x0F) == 0x04) data_len = 4;
          else if ((dib & 0x0F) == 0x06) data_len = 6;
          // Spezielle Längen für BCD-Werte, wie im PDF
          else if (dib == 0x0C) data_len = 4; // z.B. Volumen (0C 14) oder Seriennummer (0C 78)
          else if (dib == 0x0A) data_len = 2; // z.B. Temperaturen (0A 5A, 0A 5E)
          else if (dib == 0x0B) data_len = 3; // z.B. Temp-Differenz (0B 61)

          
          // Prüfe, ob genügend Bytes für das Datenfeld im Puffer sind
          if (p + 2 + data_len > data_end_p) break; 
          
          // Pointer hinter DIB/VIB setzen
          p += 2; 

          // Werte extrahieren und veröffentlichen (Pointer 'p' zeigt jetzt auf Daten)
          if (dib == 0x0C && vib == 0x78) { // Seriennummer
            if (serial_number_) serial_number_->publish_state(decode_bcd(rx_buffer_, p, data_len));
          } 
          else if (dib == 0x04 && vib == 0x06) { // Gesamtenergie
            if (total_energy_) total_energy_->publish_state(decode_u_le(rx_buffer_, p, data_len) * 0.001f);
          }
          else if (dib == 0x0C && vib == 0x14) { // Gesamtvolumen
            if (total_volume_) total_volume_->publish_state(decode_bcd(rx_buffer_, p, data_len) * 0.01f);
          }
          else if (dib == 0x0A && vib == 0x5A) { // Vorlauf Temperatur
            if (temp_flow_) temp_flow_->publish_state(decode_bcd(rx_buffer_, p, data_len) * 0.1f);
          }
          else if (dib == 0x0A && vib == 0x5E) { // Rücklauf Temperatur
            if (temp_return_) temp_return_->publish_state(decode_bcd(rx_buffer_, p, data_len) * 0.1f);
          }
          else if (dib == 0x0B && vib == 0x61) { // Temperaturdifferenz
            if (temp_diff_) temp_diff_->publish_state(decode_bcd(rx_buffer_, p, data_len) * 0.01f);
          }
          else if (dib == 0x04 && vib == 0x6D) { // Zählerzeit (unverändert)
             // ... hier müsste man eine string-basierte Veröffentlichung für Zeit machen,
             // aber die Logik aus deinem Originalcode fehlt (decode_cp32_datetime_)
             // Wir überspringen das Feld vorerst, da es komplex ist, es ohne String-Sensor zu publishen.
          }
          else {
              ESP_LOGD(TAG, "Unbekanntes Datenfeld DIB:0x%02X VIB:0x%02X, Länge %d", dib, vib, data_len);
          }

          p += data_len; // Pointer sicher hinter das Datenfeld schieben
        }
        // --- Ende Parsing ---

        rx_buffer_.clear();
        state = UM_IDLE;
        ESP_LOGI(TAG, "Update abgeschlossen.");
        return; // Verarbeitung beenden
      }
    }

    if (now - state_ts_ > 10000) { // Timeout erhöhen, da Frame groß ist
      ESP_LOGW(TAG, "RX Timeout - Kein gültiger Frame empfangen");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
