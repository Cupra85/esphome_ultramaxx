void UltraMaXXComponent::loop() {
  static uint32_t last_cycle = 0;
  static uint32_t state_ts = 0;
  static uint16_t wake_counter = 0;
  static uint8_t state = 0;

  uint32_t now = millis();

  // -------------------------------------------------
  // alle 10 Sekunden neuen Zyklus starten
  // -------------------------------------------------
  if (state == 0 && now - last_cycle > 10000) {
    ESP_LOGI(TAG, "Wakeup start");

    uart_set_parity(UART_NUM_1, UART_PARITY_DISABLE); // 8N1

    wake_counter = 0;
    state_ts = now;
    state = 1;
    last_cycle = now;
  }

  // -------------------------------------------------
  // STATE 1: 0x55 Wakeup senden (~3s)
  // -------------------------------------------------
  if (state == 1) {
    if (now - state_ts >= 8) {   // alle 8ms ein Byte
      uint8_t wake = 0x55;
      this->write_array(&wake, 1);

      state_ts = now;
      wake_counter++;

      if (wake_counter >= 360) {   // ~3 Sekunden
        state = 2;
        state_ts = now;
      }
    }
  }

  // -------------------------------------------------
  // STATE 2: Pause 234ms (laut Thread)
  // -------------------------------------------------
  if (state == 2) {
    if (now - state_ts > 234) {
      uart_set_parity(UART_NUM_1, UART_PARITY_EVEN); // 8E1
      state = 3;
    }
  }

  // -------------------------------------------------
  // STATE 3: SND_UD Init senden (CI=A6)
  // -------------------------------------------------
  if (state == 3) {
    uint8_t snd_ud[] = {
        0x68, 0x03, 0x03, 0x68,
        0x53, 0xFE, 0xA6,
        0xF7,
        0x16};

    this->write_array(snd_ud, sizeof(snd_ud));

    ESP_LOGI(TAG, "SND_UD Init gesendet");

    state = 4;
  }

  // -------------------------------------------------
  // STATE 4: RX lauschen (nicht blockierend!)
  // -------------------------------------------------
  while (available()) {
    uint8_t c;
    if (read_byte(&c)) {
      ESP_LOGD(TAG, "RX byte: 0x%02X", c);
    }
  }
}
