#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const VERSION = "UltraMaXX Parser v6.1";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;


/* ============================================================
 * Decoder
 * ============================================================*/

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data,
                                       size_t start, size_t len) const {
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul;
    mul *= 10;
    value += ((b >> 4) & 0x0F) * mul;
    mul *= 10;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data,
                                           size_t start, size_t len) const {
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++)
    v |= ((uint32_t) data[start + i]) << (8 * i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data,
                                                size_t start,
                                                std::string &out) const {
  if (start + 4 > data.size())
    return false;

  char buf[32];
  sprintf(buf, "%02u.%02u %02u:%02u",
          data[start + 1], data[start],
          data[start + 3], data[start + 2]);
  out = buf;
  return true;
}


/* ============================================================
 * Parser
 * ============================================================*/

void UltraMaXXComponent::reset_parse_flags_() {
  got_serial_ = false;
  got_energy_ = false;
  got_volume_ = false;
  got_tflow_  = false;
  got_tret_   = false;
  got_tdiff_  = false;
  got_time_   = false;
}

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &buf) {

  if (buf.size() < 20)
    return;

  for (size_t i = 0; i + 8 < buf.size(); i++) {

    // SERIAL 0C 78
    if (!got_serial_ && buf[i] == 0x0C && buf[i+1] == 0x78 && i+6 <= buf.size()) {
      float sn = decode_bcd_(buf, i+2, 4);
      ESP_LOGI(TAG, "SERIAL parsed: %.0f", sn);
      if (serial_number_) serial_number_->publish_state(sn);
      got_serial_ = true;
    }

    // ENERGY 04 06
    if (!got_energy_ && buf[i] == 0x04 && buf[i+1] == 0x06 && i+6 <= buf.size()) {
      uint32_t raw = decode_u_le_(buf, i+2, 4);
      float e = raw * 0.001f;
      ESP_LOGI(TAG, "ENERGY parsed: %.3f MWh (raw=%u)", e, raw);
      if (total_energy_) total_energy_->publish_state(e);
      got_energy_ = true;
    }

    // VOLUME 0C 14
    if (!got_volume_ && buf[i] == 0x0C && buf[i+1] == 0x14 && i+6 <= buf.size()) {
      float v = decode_bcd_(buf, i+2, 4) * 0.01f;
      ESP_LOGI(TAG, "VOLUME parsed: %.2f m3", v);
      if (total_volume_) total_volume_->publish_state(v);
      got_volume_ = true;
    }

    // FLOW TEMP 0A 5A
    if (!got_tflow_ && buf[i] == 0x0A && buf[i+1] == 0x5A && i+4 <= buf.size()) {
      float t = decode_bcd_(buf, i+2, 2) * 0.1f;
      ESP_LOGI(TAG, "FLOW TEMP parsed: %.1f °C", t);
      if (temp_flow_) temp_flow_->publish_state(t);
      got_tflow_ = true;
    }

    // RETURN TEMP 0A 5E
    if (!got_tret_ && buf[i] == 0x0A && buf[i+1] == 0x5E && i+4 <= buf.size()) {
      float t = decode_bcd_(buf, i+2, 2) * 0.1f;
      ESP_LOGI(TAG, "RETURN TEMP parsed: %.1f °C", t);
      if (temp_return_) temp_return_->publish_state(t);
      got_tret_ = true;
    }

    // DELTA T 0B 61
    if (!got_tdiff_ && buf[i] == 0x0B && buf[i+1] == 0x61 && i+5 <= buf.size()) {
      float dt = decode_bcd_(buf, i+2, 3) * 0.01f;
      ESP_LOGI(TAG, "DELTA T parsed: %.2f K", dt);
      if (temp_diff_) temp_diff_->publish_state(dt);
      got_tdiff_ = true;
    }

    // TIME 04 6D
    if (!got_time_ && buf[i] == 0x04 && buf[i+1] == 0x6D && i+6 <= buf.size()) {
      std::string ts;
      if (decode_cp32_datetime_(buf, i+2, ts)) {
        ESP_LOGI(TAG, "TIME parsed: %s", ts.c_str());
        if (meter_time_) meter_time_->publish_state(ts);
        got_time_ = true;
      }
    }
  }
}


/* ============================================================
 * Lifecycle
 * ============================================================*/

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX STARTED --- %s", VERSION);
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START (%s) ===", VERSION);

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  reset_parse_flags_();

  wake_start_ = millis();
  last_send_  = 0;
  state       = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  /* ========================= RX ========================= */

  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c))
      break;

    // Start nur bei Longframe
    if (!in_frame_) {
      if (c == 0x68) {
        in_frame_ = true;
        rx_buffer_.clear();
        rx_buffer_.push_back(c);
      }
      continue;
    }

    rx_buffer_.push_back(c);

    // Frame Ende
    if (c == 0x16) {
      ESP_LOGI(TAG, "Frame received (%d bytes)", rx_buffer_.size());
      parse_and_publish_(rx_buffer_);
      rx_buffer_.clear();
      in_frame_ = false;
    }
  }


  /* ========================= STATE ========================= */

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

    ESP_LOGI(TAG, "Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while (this->available())
      this->read_byte(&d);

    rx_buffer_.clear();
    in_frame_ = false;

    uint8_t reset[] = {0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  if (state == UM_SEND && now - state_ts_ > 150) {

    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs   = (ctrl + 0xFE) & 0xFF;

    uint8_t req[] = {0x10, ctrl, 0xFE, cs, 0x16};
    this->write_array(req,sizeof(req));
    this->flush();

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    fcb_toggle_ = !fcb_toggle_;
    state = UM_RX;
  }
}

} // namespace ultramaxx
} // namespace esphome
