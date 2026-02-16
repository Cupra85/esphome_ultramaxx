#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_RESET,
  UM_SEND,
  UM_RX
};

static UMState state = UM_IDLE;
static uint32_t wake_start = 0;
static uint32_t last_send  = 0;
static uint32_t state_ts   = 0;

// ----------------------------------------------------
// BCD decode helper (little endian)
// ----------------------------------------------------
static float decode_bcd_le(const uint8_t *p, int bytes, float div = 1.0f) {
  uint32_t value = 0;
  uint32_t mul = 1;

  for (int i = 0; i < bytes; i++) {
    uint8_t b = p[i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }

  return value / div;
}

// ----------------------------------------------------
// Type-F DateTime decode (VIF 0x6D)
// ----------------------------------------------------
static bool decode_type_f_datetime(const uint8_t *p4, char *out, size_t len) {

  const uint8_t b0 = p4[0];
  const uint8_t b1 = p4[1];
  const uint8_t b2 = p4[2];
  const uint8_t b3 = p4[3];

  if (b0 & 0x80) return false;

  uint8_t minute = (b0 & 0x3F);
  uint8_t hour   = (b1 & 0x1F);
  uint8_t hy     = (b1 >> 5) & 0x03;
  uint8_t su     = (b1 >> 7) & 0x01;

  uint8_t day    = (b2 & 0x1F);
  uint8_t y_low  = (b2 >> 5) & 0x07;

  uint8_t month  = (b3 & 0x0F);
  uint8_t y_high = (b3 >> 4) & 0x0F;

  uint8_t year_in_century = (y_high << 3) | y_low;

  int century = 1900;
  if (hy == 1) century = 2000;
  else if (hy == 2) century = 2100;
  else if (hy == 3) century = 2200;

  int year = century + year_in_century;

  if (su)
    snprintf(out, len, "%04d-%02u-%02u %02u:%02u DST", year, month, day, hour, minute);
  else
    snprintf(out, len, "%04d-%02u-%02u %02u:%02u", year, month, day, hour, minute);

  return true;
}

// ----------------------------------------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  wake_start = millis();
  last_send  = 0;
  rx_buffer_.clear();

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // ---------------- WAKEUP ----------------
  if (state == UM_WAKEUP) {

    if (now - last_send >= 15) {
      uint8_t buf[20];
      for (int i=0;i<20;i++) buf[i]=0x55;
      this->write_array(buf,20);
      last_send = now;
    }

    if (now - wake_start >= 2200) {
      ESP_LOGI(TAG,"Wakeup end");
      state = UM_WAIT;
      state_ts = now;
    }
  }

  // ---------------- SWITCH ----------------
  if (state == UM_WAIT && now - state_ts >= 350) {

    ESP_LOGI(TAG,"Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t dummy;
    while (this->available()) this->read_byte(&dummy);

    state = UM_RESET;
  }

  // ---------------- SND_NKE ----------------
  if (state == UM_RESET) {

    uint8_t reset[] = {0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state = UM_SEND;
    state_ts = millis();
  }

  // ---------------- SND_UD ----------------
  if (state == UM_SEND && millis()-state_ts > 50) {

    uint8_t snd_ud[] = {0x10,0x5B,0xFE,0x59,0x16};
    this->write_array(snd_ud,sizeof(snd_ud));
    this->flush();

    ESP_LOGI(TAG,"SND_UD Init gesendet");

    state = UM_RX;
    state_ts = millis();
  }

  // ---------------- RX PARSER ----------------
  if (state == UM_RX) {

    while (this->available()) {
      uint8_t c;
      if (this->read_byte(&c)) {
        rx_buffer_.push_back(c);
      }
    }

    if (!rx_buffer_.empty() && rx_buffer_.back()==0x16) {

      auto &f = rx_buffer_;

      for (size_t i=0;i+8<f.size();i++) {

        if (serial_number_ && f[i]==0x0C && f[i+1]==0x78)
          serial_number_->publish_state(decode_bcd_le(&f[i+2],4));

        if (total_energy_ && f[i]==0x04 && f[i+1]==0x06)
          total_energy_->publish_state(decode_bcd_le(&f[i+2],4,1000.0f));

        if (total_volume_ && f[i]==0x0C && f[i+1]==0x14)
          total_volume_->publish_state(decode_bcd_le(&f[i+2],4,1000.0f));

        if (current_power_ && f[i]==0x0B && f[i+1]==0x2D)
          current_power_->publish_state(decode_bcd_le(&f[i+2],3,10.0f));

        if (temp_flow_ && f[i]==0x0A && f[i+1]==0x5A)
          temp_flow_->publish_state(decode_bcd_le(&f[i+2],2,10.0f));

        if (temp_return_ && f[i]==0x0A && f[i+1]==0x5E)
          temp_return_->publish_state(decode_bcd_le(&f[i+2],2,10.0f));

        if (temp_diff_ && f[i]==0x0B && f[i+1]==0x61)
          temp_diff_->publish_state(decode_bcd_le(&f[i+2],3,100.0f));

        if (meter_time_ && f[i]==0x04 && f[i+1]==0x6D) {
          char ts[32];
          if (decode_type_f_datetime(&f[i+2],ts,sizeof(ts)))
            meter_time_->publish_state(ts);
        }
      }

      rx_buffer_.clear();
      state = UM_IDLE;
    }

    if (millis()-state_ts>6000) {
      ESP_LOGI(TAG,"RX timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
