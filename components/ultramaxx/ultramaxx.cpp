#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// --------- Helpers ---------

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0.0f;
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
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= (uint32_t)data[start + i] << (8 * i);
  return v;
}

// CP32 DateTime (OMS/M-Bus) aus deinen Frames (04 6D + 4 Bytes)
// Heuristik/Format, die bei deinen Bytes z.B. 0x1F 0x16 0x51 0x32 -> 2026-02-17 22:31 ergibt.
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  if (start + 4 > data.size()) return false;
  uint8_t b0 = data[start + 0];
  uint8_t b1 = data[start + 1];
  uint8_t b2 = data[start + 2];
  uint8_t b3 = data[start + 3];

  int minute = b0 & 0x3F;
  int hour   = b1 & 0x1F;
  int day    = b2 & 0x1F;
  int month  = b3 & 0x0F;

  // Jahr = (high nibble b3)<<3 | (b2>>5)  => passt auf deine 0x51/0x32 zu 2026
  int year_offset = ((b3 >> 4) << 3) | (b2 >> 5);
  int year = 2000 + year_offset;

  if (minute < 0 || minute > 59) return false;
  if (hour < 0 || hour > 23) return false;
  if (day < 1 || day > 31) return false;
  if (month < 1 || month > 12) return false;
  if (year < 2000 || year > 2099) return false;

  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d", year, month, day, hour, minute);
  out = buf;
  return true;
}

// Extrahiert 1 validen Longframe aus rx_buffer_
// Rückgabe: true wenn Frame in out liegt und rx_buffer_ bis hinter Frame abgeschnitten wurde.
static bool extract_valid_longframe(std::vector<uint8_t> &buf, std::vector<uint8_t> &out) {
  // Suche Start 0x68
  size_t i = 0;
  while (i < buf.size() && buf[i] != 0x68) i++;
  if (i > 0) {
    // Müll davor wegwerfen (E5, 55, etc.)
    buf.erase(buf.begin(), buf.begin() + i);
  }
  if (buf.size() < 6) return false;  // min: 68 L L 68 ... CS 16

  if (buf[0] != 0x68) return false;
  uint8_t L1 = buf[1];
  uint8_t L2 = buf[2];
  if (L1 != L2) {
    // kein gültiger L/L -> 0x68 verwerfen und weiter
    buf.erase(buf.begin());
    return false;
  }
  if (buf[3] != 0x68) {
    // falsches zweites Startbyte -> resync
    buf.erase(buf.begin());
    return false;
  }

  size_t total_len = (size_t)L1 + 6;  // passt zu deinen Frames (0x4D + 6 = 83)
  if (buf.size() < total_len) return false;  // noch nicht komplett

  if (buf[total_len - 1] != 0x16) {
    // Ende stimmt nicht -> resync
    buf.erase(buf.begin());
    return false;
  }

  // Checksumme prüfen: Summe von C..letztes Datenbyte (also Index 4..total_len-3)
  uint8_t cs = 0;
  for (size_t k = 4; k <= total_len - 3; k++) cs += buf[k];
  uint8_t cs_rx = buf[total_len - 2];
  if (cs != cs_rx) {
    // checksum fail -> resync
    buf.erase(buf.begin());
    return false;
  }

  out.assign(buf.begin(), buf.begin() + total_len);
  buf.erase(buf.begin(), buf.begin() + total_len);
  return true;
}

// --------- Component ---------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup Phase: 2400 8N1
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

  // ---- RX bytes sammeln (nur im RX-State) ----
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (this->read_byte(&c)) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  } else {
    // In anderen States: UART ggf. leeren, damit kein Alt-Müll das Parsing stört
    while (this->available()) {
      uint8_t c;
      this->read_byte(&c);
    }
  }

  // ---- State Machine ----

  // 1) Wakeup: 2.2s 0x55 senden (unverändert)
  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20];
      memset(buf, 0x55, sizeof(buf));
      this->write_array(buf, sizeof(buf));
      last_send_ = now;
    }
    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG, "Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
    }
    return;
  }

  // 2) Pause + Switch zu 2400 8E1
  if (state == UM_WAIT) {
    if (now - state_ts_ > 350) {
      ESP_LOGI(TAG, "Switch to 2400 8E1");

      this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
      this->parent_->load_settings();

      // Wichtig: Nach dem Umschalten nochmal alles leeren (55-Reste / Echo)
      while (this->available()) {
        uint8_t c;
        this->read_byte(&c);
      }

      // SND_NKE
      uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
      this->write_array(reset, sizeof(reset));
      this->flush();
      ESP_LOGI(TAG, "SND_NKE gesendet");

      state = UM_SEND;
      state_ts_ = now;
    }
    return;
  }

  // 3) REQ_UD2 senden (mit FCB toggling optional, aber ok)
  if (state == UM_SEND) {
    if (now - state_ts_ > 150) {
      uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;  // REQ_UD2 toggling
      uint8_t cs = (uint8_t)((ctrl + 0xFE) & 0xFF);
      uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

      this->write_array(req, sizeof(req));
      this->flush();
      ESP_LOGI(TAG, "REQ_UD2 gesendet");

      fcb_toggle_ = !fcb_toggle_;

      rx_buffer_.clear();
      last_rx_byte_ = now;
      state_ts_ = now;
      state = UM_RX;
    }
    return;
  }

  // 4) RX: Longframe sammeln -> extrahieren -> parsen -> publish
  if (state == UM_RX) {
    // Versuch: sobald genug da ist, Frame extrahieren (kann auch mehrfach laufen)
    std::vector<uint8_t> frame;
    if (extract_valid_longframe(rx_buffer_, frame)) {
      ESP_LOGI(TAG, "Parsing Frame (%u bytes)", (unsigned) frame.size());

      // Frame Layout: 68 L L 68 C A CI DATA... CS 16
      // DATA beginnt bei Index 7, endet bei size-3
      const size_t data_start = 7;
      const size_t data_end   = frame.size() - 3;

      // Wir suchen nach den DIF/VIF-Paaren, wie sie in deinen Frames vorkommen:
      // 0C 78 [4B BCD]       -> Seriennummer
      // 04 06 [4B u32 LE]    -> Energie (MWh * 0.001)
      // 0C 14 [4B BCD]       -> Volumen (m³ * 0.01)
      // 0A 5A [2B BCD]       -> Vorlauf (°C * 0.1)
      // 0A 5E [2B BCD]       -> Rücklauf (°C * 0.1)
      // 0B 61 [3B BCD]       -> ΔT (K * 0.01)
      // 04 6D [4B CP32]      -> DateTime

      for (size_t i = data_start; i + 1 < data_end; i++) {
        uint8_t dif = frame[i];
        uint8_t vif = frame[i + 1];

        // Serial: 0C 78 + 4 bytes
        if (dif == 0x0C && vif == 0x78 && i + 1 + 4 < frame.size()) {
          if (serial_number_) {
            float sn = decode_bcd(frame, i + 2, 4);
            serial_number_->publish_state(sn);
          }
          continue;
        }

        // Energy: 04 06 + 4 bytes U32 little endian -> *0.001 MWh
        if (dif == 0x04 && vif == 0x06 && i + 1 + 4 < frame.size()) {
          if (total_energy_) {
            uint32_t raw = decode_u_le(frame, i + 2, 4);
            total_energy_->publish_state(raw * 0.001f);
          }
          continue;
        }

        // Volume: 0C 14 + 4 bytes BCD -> *0.01 m³
        if (dif == 0x0C && vif == 0x14 && i + 1 + 4 < frame.size()) {
          if (total_volume_) {
            float vol = decode_bcd(frame, i + 2, 4) * 0.01f;
            total_volume_->publish_state(vol);
          }
          continue;
        }

        // Flow temp: 0A 5A + 2 bytes BCD -> *0.1 °C
        if (dif == 0x0A && vif == 0x5A && i + 1 + 2 < frame.size()) {
          if (temp_flow_) {
            float t = decode_bcd(frame, i + 2, 2) * 0.1f;
            temp_flow_->publish_state(t);
          }
          continue;
        }

        // Return temp: 0A 5E + 2 bytes BCD -> *0.1 °C
        if (dif == 0x0A && vif == 0x5E && i + 1 + 2 < frame.size()) {
          if (temp_return_) {
            float t = decode_bcd(frame, i + 2, 2) * 0.1f;
            temp_return_->publish_state(t);
          }
          continue;
        }

        // Delta T: 0B 61 + 3 bytes BCD -> *0.01 K
        if (dif == 0x0B && vif == 0x61 && i + 1 + 3 < frame.size()) {
          if (temp_diff_) {
            float dt = decode_bcd(frame, i + 2, 3) * 0.01f;
            temp_diff_->publish_state(dt);
          }
          continue;
        }

        // Meter datetime: 04 6D + 4 bytes CP32
        if (dif == 0x04 && vif == 0x6D && i + 1 + 4 < frame.size()) {
          if (meter_time_) {
            std::string ts;
            if (decode_cp32_datetime_(frame, i + 2, ts)) meter_time_->publish_state(ts);
          }
          continue;
        }
      }

      ESP_LOGI(TAG, "Update finished.");
      state = UM_IDLE;
      return;
    }

    // Timeout nur, wenn wir keinen validen Frame bekommen haben
    if (now - state_ts_ > 10000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
      return;
    }

    return;
  }
}

}  // namespace ultramaxx
}  // namespace esphome
