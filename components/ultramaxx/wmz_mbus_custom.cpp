#include "esphome.h"

class WMZMbusCustom : public PollingComponent {
 public:
  UARTComponent *uart_;
  uint32_t update_interval_ms_;

  // Sensor-Objekte
  Sensor *heat_energy_kwh;
  Sensor *volume_m3;
  Sensor *flow_m3h;
  Sensor *flow_temp_c;
  Sensor *return_temp_c;
  Sensor *power_kw;
  Sensor *temp_dif_c;
  Sensor *op_time_days;
  Sensor *month_end_values_kwh;

  void set_uart(UARTComponent *uart) { uart_ = uart; }
  void set_update_interval(uint32_t interval) { update_interval_ms_ = interval; }

  void setup() override {
    ESP_LOGD("wmz_mbus", "WMZ Custom Component setup");
  }

  void update() override {
    ESP_LOGD("wmz_mbus", "WMZ update triggered");
    wake_meter();
    read_meter();
  }

  void wake_meter() {
    ESP_LOGD("wmz_mbus", "Sending wake-up sequence");
    for(int i=0;i<528;i++) {
      uart_->write_byte(0x55);
      delay(1);
    }
    delay(400);
    uart_->write_byte(0xFE); // optional Address-Probe
    delay(2000);
  }

  void read_meter() {
    ESP_LOGD("wmz_mbus", "Reading meter bytes...");
    std::vector<uint8_t> telegram;
    while(uart_->available()) {
      uint8_t c = uart_->read();
      telegram.push_back(c);
    }

    if (telegram.size() < 10) return; // keine Daten

    // Beispiel: Dummy Parsing (hier musst du die echten M-Bus Offset/Werte einsetzen)
    // Für Demo: jedes Byte als Zahl verwenden
    float dummy_value = telegram[0] + telegram[1]/100.0;
    if (heat_energy_kwh) heat_energy_kwh->publish_state(dummy_value);
    if (volume_m3) volume_m3->publish_state(dummy_value/10.0);
    if (flow_m3h) flow_m3h->publish_state(dummy_value/100.0);
    if (flow_temp_c) flow_temp_c->publish_state(dummy_value/5.0);
    if (return_temp_c) return_temp_c->publish_state(dummy_value/5.0);
    if (power_kw) power_kw->publish_state(dummy_value/50.0);
    if (temp_dif_c) temp_dif_c->publish_state(dummy_value/10.0);
    if (op_time_days) op_time_days->publish_state(dummy_value/2.0);
    if (month_end_values_kwh) month_end_values_kwh->publish_state(dummy_value);
  }

  // Sensor-Zuweisung aus YAML
  void set_heat_energy(Sensor *s) { heat_energy_kwh = s; }
  void set_volume(Sensor *s) { volume_m3 = s; }
  void set_flow(Sensor *s) { flow_m3h = s; }
  void set_flow_temp(Sensor *s) { flow_temp_c = s; }
  void set_return_temp(Sensor *s) { return_temp_c = s; }
  void set_power(Sensor *s) { power_kw = s; }
  void set_temp_dif(Sensor *s) { temp_dif_c = s; }
  void set_op_time(Sensor *s) { op_time_days = s; }
  void set_month_end(Sensor *s) { month_end_values_kwh = s; }
};

  }

  if (frame.size() < 10 || frame[0] != 0x68) {
    ESP_LOGW("wmz_mbus", "Kein gültiges Telegramm empfangen (len=%d)", frame.size());
    return;
  }

  // Länge prüfen
  uint8_t len = frame[1];
  if (frame.size() < (len + 6)) {
    ESP_LOGW("wmz_mbus", "Unvollständiges Telegramm empfangen");
    return;
  }

  // Prüfsumme checken
  uint8_t cs = frame[len + 3];
  uint8_t calc = calc_checksum(frame, 4, len + 3);
  if (cs != calc) {
    ESP_LOGW("wmz_mbus", "Checksumme falsch (erwartet %02X, berechnet %02X)", cs, calc);
    return;
  }

  // ====== 3. Datenfelder auswerten ======
  // Ganz einfache Parser-Version: sucht nach bekannten VIF-Codes
  // (Allmess Integral liefert typ. Energie [kWh], Volumen [m3], Temperaturen [°C])

  size_t i = 9; // ab hier fangen die DIF/VIF-Blöcke an
  double heat_energy = NAN, volume = NAN, flow_temp = NAN, return_temp = NAN, power = NAN, flow = NAN;

  while (i < frame.size() - 2) {
    uint8_t dif = frame[i++];
    if (dif == 0x2F) continue; // filler

    uint8_t vif = frame[i++];
    int size = dif & 0x0F;

    if (i + size > frame.size()) break;

    uint32_t value = 0;
    for (int j = 0; j < size; j++) {
      value |= ((uint32_t)frame[i++]) << (8 * j);
    }

    // VIF auswerten
    if ((vif & 0x7F) == 0x6C) {        // Energie (Wh)
      heat_energy = value / 1000.0;    // -> kWh
    } else if ((vif & 0x7F) == 0x13) { // Volumen (m3)
      volume = value / 1000.0;
    } else if ((vif & 0x7F) == 0x5A) { // Vorlauftemperatur
      flow_temp = value / 100.0;
    } else if ((vif & 0x7F) == 0x5B) { // Rücklauftemperatur
      return_temp = value / 100.0;
    } else if ((vif & 0x7F) == 0x2B) { // Leistung (W)
      power = value / 1000.0;
    } else if ((vif & 0x7F) == 0x14) { // Durchfluss (l/h)
      flow = value / 1000.0;           // -> m3/h
    }
  }

  // ====== 4. Sensorwerte publizieren ======
  for (auto &pair : sensors_) {
    Sensor *sensor = pair.first;
    std::string type = pair.second;

    if (type == "heat_energy_kwh" && !isnan(heat_energy))
      sensor->publish_state(heat_energy);
    else if (type == "volume_m3" && !isnan(volume))
      sensor->publish_state(volume);
    else if (type == "flow_temp_c" && !isnan(flow_temp))
      sensor->publish_state(flow_temp);
    else if (type == "return_temp_c" && !isnan(return_temp))
      sensor->publish_state(return_temp);
    else if (type == "power_kw" && !isnan(power))
      sensor->publish_state(power);
    else if (type == "flow_m3/h" && !isnan(flow))
      sensor->publish_state(flow);
  }
}

void WMZMBUSCustom::add_sensor(Sensor *sensor, const std::string &type) {
  sensors_.push_back({sensor, type});
}

}  // namespace wmz_mbus_custom
}  // namespace esphome
