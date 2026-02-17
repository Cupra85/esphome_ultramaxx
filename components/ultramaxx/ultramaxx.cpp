#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

// -------------------------
// Helpers
// -------------------------
static inline uint32_t u32_le(const std::vector<uint8_t> &d, size_t o) {
  if (o + 4 > d.size()) return 0;
  return (uint32_t)d[o] | ((uint32_t)d[o + 1] << 8) | ((uint32_t)d[o + 2] << 16) | ((uint32_t)d[o + 3] << 24);
}

static inline uint16_t u16_le(const std::vector<uint8_t> &d, size_t o) {
  if (o + 2 > d.size()) return 0;
  return (uint16_t)d[o] | ((uint16_t)d[o + 1] << 8);
}

static uint32_t bcd_to_uint(const std::vector<uint8_t> &data, size_t start, size_t len) {
  // Little-endian BCD: low byte contains lowest digits
  uint32_t value = 0;
  uint32_t mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return value;
}

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  return (float) bcd_to_uint(data, start, len);
}

// Extract a valid long frame 68 L L 68 ... CS 16 from rx_buffer_
// Returns true if extracted into "frame" (and removes leading garbage from rx_buffer_).
static bool extract_long_frame(std::vector<uint8_t> &buf, std::vector<uint8_t> &frame) {
  frame.clear();
  if (buf.size() < 6) return false;

  // find 0x68
  size_t s = 0;
  while (s < buf.size() && buf[s] != 0x68) s++;
  if (s >= buf.size()) {
    buf.clear();
    return false;
  }
  // need at least 4 bytes: 68 L L 68
  if (s + 4 > buf.size()) {
    if (s > 0) buf.erase(buf.begin(), buf.begin() + s);
    return false;
  }
  if (buf[s] != 0x68 || buf[s + 3] != 0x68) {
    // drop this 0x68 and continue later
    buf.erase(buf.begin(), buf.begin() + s + 1);
    return false;
  }

  uint8_t l1 = buf[s + 1];
  uint8_t l2 = buf[s + 2];
  if (l1 != l2) {
    buf.erase(buf.begin(), buf.begin() + s + 1);
    return false;
  }

  // Long frame total length = 6 + L (68 L L 68 + (L bytes from C..CS) + 16)
  // Actually: bytes from C..CS are (L + 1)?? In practice:
  // Layout: 68 L L 68 C A CI ... DATA ... CS 16
  // L counts bytes from C to CS inclusive => total = 6 + L
  // -> indices: s .. s+3 header, then L bytes, then 0x16.
  size_t total = 6 + (size_t)l1;  // 4 bytes header + L bytes (C..CS) + 0x16(1) + ??? (already in +6)
  // Check: For typical formula: total = L + 6 (works with many meters)
  if (s + total > buf.size()) {
    // not complete yet
    if (s > 0) buf.erase(buf.begin(), buf.begin() + s);
    return false;
  }

  // last byte must be 0x16
  if (buf[s + total - 1] != 0x16) {
    // try to resync: drop start byte
    buf.erase(buf.begin(), buf.begin() + s + 1);
    return false;
  }

  frame.assign(buf.begin() + s, buf.begin() + s + total);
  // remove consumed bytes up to end of frame
  buf.erase(buf.begin(), buf.begin() + s + total);
  return true;
}

// Parse Date/Time (VIF 0x6D, 4 bytes) per common EN13757-3 "Type F" usage:
// bytes: [time_lo time_hi date_lo date_hi], little-endian words.
// date word: day (0..4), month (5..8), year (9..15) offset 2000
// time word: minute (0..5), hour (6..10)
static std::string decode_datetime_6d(const std::vector<uint8_t> &f, size_t o) {
  if (o + 4 > f.size()) return {};
  uint16_t time = u16_le(f, o);
  uint16_t date = u16_le(f, o + 2);

  int minute = (int)(time & 0x3F);
  int hour   = (int)((time >> 6) & 0x1F);

  int day    = (int)(date & 0x1F);
  int month  = (int)((date >> 5) & 0x0F);
  int year   = 2000 + (int)((date >> 9) & 0x7F);

  char out[32];
  snprintf(out, sizeof(out), "%04d-%02d-%02d %02d:%02d", year, month, day, hour, minute);
  return std::string(out);
}

// Determine data length from DIF low nibble for our known UltraMaXX records.
// (robust enough for your actual frames; we also handle 0x02/0x03/0x04)
static int dif_data_len(uint8_t dif) {
  switch (dif & 0x0F) {
    case 0x02: return 1;
    case 0x03: return 2;
    case 0x04: return 4;
    case 0x0A: return 2; // seen: 0A 5A .. (temp)
    case 0x0B: return 3; // seen: 0B 61 .. (deltaT)
    case 0x0C: return 4; // seen: 0C 78, 0C 14 .. (BCD 8 digits)
    default:   return 0;
  }
}

// -------------------------
// Component
// -------------------------
void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG, "=== READ START ===");

  // Wakeup part stays as-is (per your requirement)
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

  // Collect RX bytes continuously when in UM_RX
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // --- Wakeup sequence (unchanged) ---
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
  }

  // --- Switch to 8E1 and send SND_NKE (unchanged) ---
  if (state == UM_WAIT && now - state_ts_ > 350) {
    ESP_LOGI(TAG, "Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while (this->available()) this->read_byte(&d);

    uint8_t reset[] = {0x10, 0x40, 0xFE, 0x3E, 0x16};
    this->write_array(reset, sizeof(reset));
    this->flush();
    ESP_LOGI(TAG, "SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  // --- Send REQ_UD2 (FCB toggled) ---
  if (state == UM_SEND && now - state_ts_ > 150) {
    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;     // REQ_UD2 with/without FCB
    uint8_t cs = (uint8_t)((ctrl + 0xFE) & 0xFF);
    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};

    this->write_array(req, sizeof(req));
    this->flush();
    ESP_LOGI(TAG, "REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;

    rx_buffer_.clear();
    last_rx_byte_ = now;
    state = UM_RX;
    state_ts_ = now;
  }

  // --- RX + Parsing ---
  if (state == UM_RX) {

    // Try extract frame when we have end marker or silence
    if (!rx_buffer_.empty() && (rx_buffer_.back() == 0x16 || (now - last_rx_byte_ > 300))) {

      std::vector<uint8_t> frame;
      // We might have multiple frames/garbage; extract one valid long frame
      std::vector<uint8_t> tmp = rx_buffer_;
      if (!extract_long_frame(tmp, frame)) {
        // keep collecting (or resync if buffer grows too big)
        if (rx_buffer_.size() > 512) rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 128);
      } else {
        // We consumed one; keep remainder
        rx_buffer_ = tmp;

        // Basic header checks
        if (frame.size() < 4 + 3) {
          ESP_LOGW(TAG, "Frame too short");
          state = UM_IDLE;
          return;
        }

        // long frame fields
        uint8_t C  = frame[4];
        uint8_t A  = frame[5];
        uint8_t CI = frame[6];

        // We expect variable data response CI=0x72 (like your logs)
        if (CI != 0x72) {
          ESP_LOGW(TAG, "Unexpected CI 0x%02X (C=0x%02X A=0x%02X)", CI, C, A);
          state = UM_IDLE;
          return;
        }

        // Fixed header (CI=0x72): 12 bytes after CI
        // ident(4) manuf(2) version(1) medium(1) access(1) status(1) signature(2)
        const size_t fixed_off = 7;
        const size_t fixed_len = 12;
        if (frame.size() < fixed_off + fixed_len + 2) {
          ESP_LOGW(TAG, "Frame missing fixed header");
          state = UM_IDLE;
          return;
        }

        // Ident from fixed header (also your serial)
        uint32_t ident = (uint32_t)frame[fixed_off] |
                         ((uint32_t)frame[fixed_off + 1] << 8) |
                         ((uint32_t)frame[fixed_off + 2] << 16) |
                         ((uint32_t)frame[fixed_off + 3] << 24);

        // records start
        size_t ptr = fixed_off + fixed_len;

        ESP_LOGI(TAG, "Parsing CI=0x72, ident=%u, frame=%dB", (unsigned)ident, (int)frame.size());

        // publish serial from fixed header as fallback
        if (serial_number_) serial_number_->publish_state((float)ident);

        // Iterate records (DIF + VIF (+ possible extensions))
        while (ptr + 2 < frame.size()) {
          // Stop before checksum/end (last 2 bytes: CS 16)
          if (ptr >= frame.size() - 2) break;

          uint8_t dif = frame[ptr++];

          // Skip DIFE(s) if extension bit set
          while (dif & 0x80) {
            if (ptr >= frame.size() - 2) break;
            uint8_t dife = frame[ptr++];
            dif = (uint8_t)(dif & 0x7F);  // keep base type; DIFE content not needed for our fixed keys
            if (!(dife & 0x80)) break;
          }

          if (ptr >= frame.size() - 2) break;

          uint8_t vif = frame[ptr++];

          // Skip VIFE(s) if extension bit set
          while (vif & 0x80) {
            if (ptr >= frame.size() - 2) break;
            uint8_t vife = frame[ptr++];
            vif = (uint8_t)(vif & 0x7F);  // base VIF for our keys
            if (!(vife & 0x80)) break;
          }

          int len = dif_data_len(dif);
          if (len <= 0) {
            // Unknown type -> best effort skip 1 byte to resync
            continue;
          }
          if (ptr + (size_t)len > frame.size() - 2) break;

          // Now we have (dif, vif, data[ptr..ptr+len-1])
          // ---- Map according to your actual telegram ----

          // Serial record: 0C 78 [BCD4]
          if (serial_number_ && dif == 0x0C && vif == 0x78 && len == 4) {
            float sn = decode_bcd(frame, ptr, 4);
            serial_number_->publish_state(sn);
          }

          // Energy: 04 06 [u32 LE], scale 0.001 -> MWh
          if (total_energy_ && dif == 0x04 && vif == 0x06 && len == 4) {
            uint32_t v = u32_le(frame, ptr);
            total_energy_->publish_state((float)v * 0.001f);
          }

          // Volume: 0C 14 [BCD4], scale 0.01 -> m³
          if (total_volume_ && dif == 0x0C && vif == 0x14 && len == 4) {
            float v = decode_bcd(frame, ptr, 4) * 0.01f;
            total_volume_->publish_state(v);
          }

          // Flow temp: 0A 5A [BCD2], scale 0.1 -> °C
          if (temp_flow_ && dif == 0x0A && vif == 0x5A && len == 2) {
            float t = decode_bcd(frame, ptr, 2) * 0.1f;
            temp_flow_->publish_state(t);
          }

          // Return temp: 0A 5E [BCD2], scale 0.1 -> °C
          if (temp_return_ && dif == 0x0A && vif == 0x5E && len == 2) {
            float t = decode_bcd(frame, ptr, 2) * 0.1f;
            temp_return_->publish_state(t);
          }

          // Delta T: 0B 61 [BCD3], scale 0.01 -> K
          if (temp_diff_ && dif == 0x0B && vif == 0x61 && len == 3) {
            float dt = decode_bcd(frame, ptr, 3) * 0.01f;
            temp_diff_->publish_state(dt);
          }

          // Meter time: 04 6D [4 bytes Type-F], publish ISO string
          if (meter_time_ && dif == 0x04 && vif == 0x6D && len == 4) {
            std::string ts = decode_datetime_6d(frame, ptr);
            if (!ts.empty()) meter_time_->publish_state(ts);
          }

          ptr += (size_t)len;
        }

        ESP_LOGI(TAG, "Update finished.");
        state = UM_IDLE;
        return;
      }
    }

    // Timeout window
    if (now - state_ts_ > 10000) {
      ESP_LOGW(TAG, "RX Timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

}  // namespace ultramaxx
}  // namespace esphome
