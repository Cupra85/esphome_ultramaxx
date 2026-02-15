void UltraMaXXComponent::loop() {
  static uint32_t last_run = 0;
  static uint32_t wakeup_start = 0;
  static int wakeup_count = 0;
  static bool waking = false;

  // alle 10 Sekunden neuen Zyklus starten
  if (!waking && millis() - last_run > 10000) {
    ESP_LOGI(TAG, "Wakeup start");

    uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE);

    wakeup_start = millis();
    wakeup_count = 0;
    waking = true;
    last_run = millis();
  }

  // nicht-blockierender Wakeup
  if (waking) {
    if (wakeup_count < 360) {
      uint8_t wake = 0x55;
      this->write_array(&wake, 1);
      wakeup_count++;
      delay(1);   // mini pause erlaubt scheduler
    } else {
      uart_set_parity(UART_NUM_1, UART_PARITY_EVEN);

      uint8_t snd_ud[] = {
          0x68, 0x03, 0x03, 0x68,
          0x53, 0xFE, 0xA6,
          0xF7,
          0x16};

      this->write_array(snd_ud, sizeof(snd_ud));
      ESP_LOGI(TAG, "SND_UD Init gesendet");

      waking = false;
    }
  }

  // RX lesen (läuft jetzt parallel!)
  while (available()) {
    uint8_t c;
    if (read_byte(&c)) {
      ESP_LOGD(TAG, "RX byte: 0x%02X", c);
    }
  }
}
