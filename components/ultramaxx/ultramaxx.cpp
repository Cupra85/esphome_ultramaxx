#include "ultramaxx.h"
#include <cstring>

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_SEND_RESET,
  UM_SEND_REQ,
  UM_RX
};

static UMState state = UM_IDLE;

// -------------------------
// Helper: frame extraction
// -------------------------
static bool extract_valid_longframe_(const std::vector<uint8_t> &buf, std::vector<uint8_t> &out_frame) {
  // Valid long frame:
  // 68 L L 68 C A CI ... CS 16
  // Total length = 6 + L (bytes)  (because: 4 header + L bytes body + CS + 16)
  // L = number of bytes from C..(data..)
  if (buf.size() < 6) return false;

  // Search for 0x68 start
  for (size_t s = 0; s + 6 <= buf.size(); s++) {
    if (buf[s] != 0x68) continue;
    if (s + 3 >= buf.size()) return false;

    uint8_t l1 = buf[s + 1];
    uint8_t l2 = buf[s + 2];
    if (l1 != l2) continue;
    if (buf[s + 3] != 0x68) continue;

    size_t total = 6 + (size_t) l1;           // 68 L L 68 + (L bytes from C..) + CS + 16
    if (s + total > buf.size()) continue;     // not complete yet
    if (buf[s + total - 1] != 0x16) continue; // must end with 0x16

    // checksum check (optional but recommended)
    // checksum = sum(C..last data byte) & 0xFF
    // In longframe: checksum byte is at index: s + 4 + L
    // Data bytes C.. = length L starting at (s+4)
    uint8_t cs = 0;
    for (size_t i = 0; i < (size_t) l1; i++) cs = (uint8_t) (cs + buf[s + 4 + i]);
    uint8_t cs_frame = buf[s + 4 + (size_t) l1];
    if (cs != cs_frame) {
      // If checksum mismatch, skip this start byte and keep searching
      continue;
    }

    out_frame.assign(buf.begin() + s, buf.begin() + s + total);
    return true;
  }

  return false;
}

// -------------------------
// BCD / LE decoders
// -------------------------
float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0.0f;
  float value = 0.0f;
  float mul = 1.0f;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10.0f;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10.0f;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= ((uint32_t) data[start + i]) << (8 * i);
  return v;
}

// CP32: “date/time” coding used by many meters (4 bytes).
// In practice: this varies by manufacturer; for UltraMaXX it matches what you see at 04 6D xx xx xx xx.
// We'll decode common CP32 (minute/hour/day/month/year since 2000-ish).
bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) {
  if (start + 4 > data.size()) return false;

  uint32_t raw = decode_u_le(data, start, 4);

  // Common M-Bus CP32 layout (used in lots of WMZ):
  // bits  0..5  minute (0-59)
  // bits  6..10 hour   (0-23)
  // bits 11..15 day    (1-31)
  // bits 16..19 month  (1-12)
  // bits 20..27 year   (0-99) -> 2000+year
  // remaining bits sometimes flags
  uint8_t minute = (raw >> 0) & 0x3F;
  uint8_t hour   = (raw >> 6) & 0x1F;
  uint8_t day    = (raw >> 11) & 0x1F;
  uint8_t month  = (raw >> 16) & 0x0F;
  uint8_t year   = (raw >> 20) & 0xFF;

  if (minute > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) return false;

  char buf[32];
  // ISO-ish
  snprintf(buf, sizeof(buf), "20%02u-%02u-%02u %02u:%02u", year, month, day, hour, minute);
  out = buf;
  return true;
}

// -------------------------
// ESPHome lifecycle
// -------------------------
void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup: 2400 8N1
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  // HARD clear software buffer (our own)
  rx_buffer_.clear();

  // HARD clear UART RX FIFO (important: remove leftover 0x55 / noise)
  uint8_t d;
  while (this->available()) this->read_byte(&d);

  wake_start_ = millis();
  last_send_ = 0;
  state_ts_ = millis();
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
  const uint32_t now = millis();

  // Always consume incoming bytes into rx_buffer_ ONLY in RX state.
  // In other states we drain/ignore aggressively to prevent “early parse”.
  if (state == UM_RX) {
    while (this->available()) {
      uint8_t c;
      if (this->read_byte(&c)) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  } else {
    // Drain anything (e.g., leftover 0x55, echo, etc.)
    uint8_t d;
    while (this->available()) this->read_byte(&d);
  }

  // -------------------------
  // WAKEUP: send 0x55 for 2.2s
  // -------------------------
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

  // -------------------------
  // WAIT 350ms, switch to 8E1
  // -------------------------
  if (state == UM_WAIT) {
    if (now - state_ts_ < 350) return;

    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_baud_rate(2400);
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    // After reconfig: drain UART RX FIFO again
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    // prepare next
    state = UM_SEND_RESET;
    state_ts_ = now;
    return;
  }

  // -------------------------
  // Send SND_NKE (reset)
  // -------------------------
  if (state == UM_SEND_RESET) {
    // short frame: 10 40 FE CS 16, CS = (40+FE)&FF = 3E
    const uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    // After sending, drain any immediate echo/noise
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    state = UM_SEND_REQ;
    state_ts_ = now;
    return;
  }

  // -------------------------
  // Send REQ_UD2 (toggle 0x5B/0x7B)
  // -------------------------
  if (state == UM_SEND_REQ) {
    // Wait a bit like Tasmota (>=50ms). You used 150ms before; keep conservative.
    if (now - state_ts_ < 150) return;

    const uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    const uint8_t cs = (uint8_t) ((ctrl + 0xFE) & 0xFF);
    const uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();

    fcb_toggle_ = !fcb_toggle_;

    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    // IMPORTANT: Clear buffers AFTER sending request,
    // so the next bytes are only the longframe.
    rx_buffer_.clear();
    uint8_t d;
    while (this->available()) this->read_byte(&d);

    state = UM_RX;
    state_ts_ = now;
    last_rx_byte_ = now;
    return;
  }

  // -------------------------
  // RX: wait for a valid longframe and parse it
  // -------------------------
  if (state == UM_RX) {
    // Try to extract a valid frame as soon as we have something meaningful.
    // Do NOT parse “on silence only” because your ESP logs show the longframe arrives fast,
    // but “silence trigger” sometimes fires too early.
    std::vector<uint8_t> frame;
    if (extract_valid_longframe_(rx_buffer_, frame)) {
      ESP_LOGI(TAG, "Parsing Frame (%u bytes)", (unsigned) frame.size());

      // frame layout:
      // 0:68 1:L 2:L 3:68 4:C 5:A 6:CI 7.. data ... (CS) (16)
      const uint8_t L = frame[1];
      const size_t data_start = 7;
      const size_t cs_index = 4 + (size_t) L;
      if (cs_index + 1 >= frame.size()) {
        ESP_LOGW(TAG, "Frame too short (internal)");  // should not happen due to extractor
        rx_buffer_.clear();
        state = UM_IDLE;
        return;
      }

      // Data bytes: from data_start to cs_index-1 inclusive
      const size_t data_end = cs_index; // exclusive
      // Pattern scan inside data area for known DIF/VIF combos.
      for (size_t i = data_start; i + 1 < data_end; i++) {
        const uint8_t dif = frame[i];
        const uint8_t vif = frame[i + 1];

        // Serial number: 0C 78 + 4 BCD bytes
        if (serial_number_ && dif == 0x0C && vif == 0x78 && i + 2 + 4 <= data_end) {
          float sn = decode_bcd(frame, i + 2, 4);
          serial_number_->publish_state(sn);
        }

        // Energy: 04 06 + 4 bytes LE integer -> *0.001 = MWh (your frame: 0x00005ECC = 24268 => 24.268 MWh)
        if (total_energy_ && dif == 0x04 && vif == 0x06 && i + 2 + 4 <= data_end) {
          uint32_t v = decode_u_le(frame, i + 2, 4);
          total_energy_->publish_state((float) v * 0.001f);
        }

        // Volume: 0C 14 + 4 BCD bytes -> *0.01 = m³ (e.g. 34 39 16 00 => 1639.34)
        if (total_volume_ && dif == 0x0C && vif == 0x14 && i + 2 + 4 <= data_end) {
          float vol = decode_bcd(frame, i + 2, 4) * 0.01f;
          total_volume_->publish_state(vol);
        }

        // Current power: (often 04 2B + 4 bytes) -> W or kW depends on meter;
        // In your Mikrocontroller.net mapping it’s “042b u32s @1 Leistung W”.
        // If your sensor wants kW: /1000.
        if (current_power_ && dif == 0x04 && vif == 0x2B && i + 2 + 4 <= data_end) {
          uint32_t p_w = decode_u_le(frame, i + 2, 4);
          current_power_->publish_state((float) p_w / 1000.0f);
        }

        // Flow temp: 0A 5A + 2 BCD bytes -> *0.1 °C
        if (temp_flow_ && dif == 0x0A && vif == 0x5A && i + 2 + 2 <= data_end) {
          temp_flow_->publish_state(decode_bcd(frame, i + 2, 2) * 0.1f);
        }

        // Return temp: 0A 5E + 2 BCD bytes -> *0.1 °C
        if (temp_return_ && dif == 0x0A && vif == 0x5E && i + 2 + 2 <= data_end) {
          temp_return_->publish_state(decode_bcd(frame, i + 2, 2) * 0.1f);
        }

        // Delta T: 0B 61 + 3 BCD bytes -> *0.01 K
        if (temp_diff_ && dif == 0x0B && vif == 0x61 && i + 2 + 3 <= data_end) {
          temp_diff_->publish_state(decode_bcd(frame, i + 2, 3) * 0.01f);
        }

        // Meter time: 04 6D + 4 bytes CP32
        if (meter_time_ && dif == 0x04 && vif == 0x6D && i + 2 + 4 <= data_end) {
          std::string ts;
          if (decode_cp32_datetime_(frame, i + 2, ts)) {
            meter_time_->publish_state(ts);
          }
        }
      }

      // Done for this cycle
      rx_buffer_.clear();
      state = UM_IDLE;
      ESP_LOGI(TAG, "Update finished.");
      return;
    }

    // Timeout (give it enough time; your longframe arrives within <1s typically)
    if (now - state_ts_ > 6000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
      return;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
