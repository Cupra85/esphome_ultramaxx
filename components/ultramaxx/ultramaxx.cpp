#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  if (start + len > data.size()) return 0;
  float value = 0;
  float mul = 1;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[start + i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return value;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG,"UltraMaXX component started");
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG,"=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_ = millis();
  last_send_  = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // RX immer sammeln
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // WAKEUP
  if (state == UM_WAKEUP) {
    if (now - last_send_ > 15) {
      uint8_t buf[20]; memset(buf,0x55,20);
      this->write_array(buf,20);
      last_send_ = now;
    }
    if (now - wake_start_ > 2200) {
      ESP_LOGI(TAG,"Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
    }
  }

  // SWITCH UART
  if (state == UM_WAIT && now - state_ts_ > 350) {

    ESP_LOGI(TAG,"Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while(this->available()) this->read_byte(&d);

    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state = UM_SEND;
    state_ts_ = now;
  }

  // REQ_UD2
  if (state == UM_SEND && now - state_ts_ > 150) {

    uint8_t req[]={0x10,0x7B,0xFE,0x79,0x16};
    this->write_array(req,sizeof(req));
    this->flush();

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    rx_buffer_.clear();
    last_rx_byte_ = now;
    state = UM_RX;
  }

  // PARSER
  if (state == UM_RX) {

    // Frame vollständig?
    if (!rx_buffer_.empty() && rx_buffer_.back()==0x16) {

      ESP_LOGI(TAG,"Parsing Frame (%d Bytes)...",(int)rx_buffer_.size());

      auto &f = rx_buffer_;

      // ⭐⭐ USERDATA START ⭐⭐
      size_t i = 7;

      while (i + 6 < f.size()) {

        uint8_t DIF = f[i];
        uint8_t VIF = f[i+1];

        // Seriennummer 0C78
        if (DIF==0x0C && VIF==0x78) {
          if(serial_number_) serial_number_->publish_state(decode_bcd(f,i+2,4));
          i+=6;
          continue;
        }

        // Energie 0406
        if (DIF==0x04 && VIF==0x06) {
          uint32_t v = (uint32_t)f[i+2] |
                       (uint32_t)f[i+3]<<8 |
                       (uint32_t)f[i+4]<<16 |
                       (uint32_t)f[i+5]<<24;
          if(total_energy_) total_energy_->publish_state(v*0.001f);
          i+=6;
          continue;
        }

        // Volumen 0C14
        if (DIF==0x0C && VIF==0x14) {
          if(total_volume_) total_volume_->publish_state(decode_bcd(f,i+2,4)*0.01f);
          i+=6;
          continue;
        }

        // Vorlauf 0A5A
        if (DIF==0x0A && VIF==0x5A) {
          if(temp_flow_) temp_flow_->publish_state(decode_bcd(f,i+2,2)*0.1f);
          i+=4;
          continue;
        }

        // Rücklauf 0A5E
        if (DIF==0x0A && VIF==0x5E) {
          if(temp_return_) temp_return_->publish_state(decode_bcd(f,i+2,2)*0.1f);
          i+=4;
          continue;
        }

        // DeltaT 0B61
        if (DIF==0x0B && VIF==0x61) {
          if(temp_diff_) temp_diff_->publish_state(decode_bcd(f,i+2,3)*0.01f);
          i+=5;
          continue;
        }

        i++;
      }

      rx_buffer_.clear();
      state = UM_IDLE;
      ESP_LOGI(TAG,"Update finished.");
    }

    if(now-last_rx_byte_>4000){
      ESP_LOGW(TAG,"RX Timeout");
      rx_buffer_.clear();
      state=UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
