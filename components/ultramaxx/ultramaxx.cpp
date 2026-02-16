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

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
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

  // RX immer einsammeln
  while (this->available()) {
    uint8_t c;
    if (this->read_byte(&c)) {
      if (state == UM_RX) {
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // Wakeup (unverändert)
  if(state==UM_WAKEUP){
    if(now-last_send_>15){
      uint8_t buf[20]; memset(buf,0x55,20);
      this->write_array(buf,20);
      last_send_=now;
    }
    if(now-wake_start_>2200){
      ESP_LOGI(TAG,"Wakeup end");
      state=UM_WAIT;
      state_ts_=now;
    }
  }

  if(state==UM_WAIT && now-state_ts_>350){

    ESP_LOGI(TAG,"Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while(this->available()) this->read_byte(&d);

    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state=UM_SEND;
    state_ts_=now;
  }

  if(state==UM_SEND && now-state_ts_>150){

    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs   = (ctrl + 0xFE) & 0xFF;
    uint8_t req[]={0x10,ctrl,0xFE,cs,0x16};

    this->write_array(req,sizeof(req));
    this->flush();

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    fcb_toggle_=!fcb_toggle_;

    rx_buffer_.clear();

    state=UM_RX;
    state_ts_=now;
    last_rx_byte_=now;
  }

  if(state==UM_RX){

    // Frame fertig wenn 0x16 erkannt ODER 500ms Ruhe
    if(!rx_buffer_.empty() &&
      (rx_buffer_.back()==0x16 || now-last_rx_byte_>500)){

      ESP_LOGI(TAG,"Parsing Frame (%d Bytes)...",(int)rx_buffer_.size());

      auto &f = rx_buffer_;

      // DATA START laut EN13757-3
      if(f.size()<10) return;
      size_t i = 7;   // ⭐ WICHTIG: Daten beginnen ab Byte 7

      while(i+6 < f.size()){

        uint8_t DIF = f[i];
        uint8_t VIF = f[i+1];

        // SERIAL
        if(VIF==0x78 && serial_number_){
          serial_number_->publish_state(decode_bcd(f,i+2,4));
        }

        // ENERGY (04 06)
        if(VIF==0x06 && total_energy_){
          uint32_t v =
            (uint32_t)f[i+2] |
            (uint32_t)f[i+3]<<8 |
            (uint32_t)f[i+4]<<16 |
            (uint32_t)f[i+5]<<24;
          total_energy_->publish_state(v*0.001f);
        }

        // VOLUME
        if(VIF==0x14 && total_volume_){
          total_volume_->publish_state(decode_bcd(f,i+2,4)*0.01f);
        }

        // FLOW TEMP
        if(VIF==0x5A && temp_flow_){
          temp_flow_->publish_state(decode_bcd(f,i+2,2)*0.1f);
        }

        // RETURN TEMP
        if(VIF==0x5E && temp_return_){
          temp_return_->publish_state(decode_bcd(f,i+2,2)*0.1f);
        }

        // DELTA T
        if(VIF==0x61 && temp_diff_){
          temp_diff_->publish_state(decode_bcd(f,i+2,3)*0.01f);
        }

        // nächstes Feld (laut Thread immer 6 Bytes weiter)
        i += 6;
      }

      ESP_LOGI(TAG,"Update finished.");
      rx_buffer_.clear();
      state=UM_IDLE;
    }

    if(now-state_ts_>12000){
      ESP_LOGW(TAG,"RX Timeout");
      rx_buffer_.clear();
      state=UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
