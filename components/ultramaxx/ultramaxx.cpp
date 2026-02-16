#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
  float value = 0;
  float mul = 1;
  for(size_t i=0;i<len;i++){
    uint8_t b=data[start+i];
    value += (b & 0x0F) * mul; mul*=10;
    value += ((b>>4)&0x0F) * mul; mul*=10;
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

  // RX immer einsammeln
  while(this->available()){
    uint8_t c;
    if(this->read_byte(&c)){
      if(state==UM_RX){
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // WAKEUP
  if(state==UM_WAKEUP){
    if(now-last_send_>15){
      uint8_t buf[20];
      memset(buf,0x55,20);
      this->write_array(buf,20);
      last_send_=now;
    }
    if(now-wake_start_>2200){
      ESP_LOGI(TAG,"Wakeup end");
      state=UM_WAIT;
      state_ts_=now;
    }
  }

  // SWITCH
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

  // REQ_UD2
  if(state==UM_SEND && now-state_ts_>150){

    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl + 0xFE) & 0xFF;
    uint8_t req[]={0x10,ctrl,0xFE,cs,0x16};

    this->write_array(req,sizeof(req));
    this->flush();

    fcb_toggle_ = !fcb_toggle_;

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    rx_buffer_.clear();
    state=UM_RX;
    state_ts_=now;
    last_rx_byte_=now;
  }

  // ======================
  // PARSER (FIXED!)
  // ======================

  if(state==UM_RX){

    if(!rx_buffer_.empty() && (now-last_rx_byte_>400 || rx_buffer_.back()==0x16)){

      ESP_LOGI(TAG,"Parsing Frame (%d Bytes)",rx_buffer_.size());

      auto &f = rx_buffer_;

      for(size_t i=0;i+6<f.size();i++){

        uint8_t DIF = f[i];
        uint8_t VIF = f[i+1];

        uint8_t len_code = DIF & 0x0F;

        int len = 0;
        if(len_code==0x04) len=4;
        else if(len_code==0x02) len=2;
        else if(len_code==0x03) len=3;
        else continue;

        // SERIAL 0C78
        if(serial_number_ && (DIF&0x0F)==0x0C && VIF==0x78){
          serial_number_->publish_state(decode_bcd(f,i+2,4));
        }

        // ENERGY 0406 UINT32 LE
        if(total_energy_ && (DIF&0x0F)==0x04 && VIF==0x06){
          uint32_t v = (uint32_t)f[i+2] | (uint32_t)f[i+3]<<8 | (uint32_t)f[i+4]<<16 | (uint32_t)f[i+5]<<24;
          total_energy_->publish_state(v * 0.001f);
        }

        // VOLUME 0C14 BCD
        if(total_volume_ && (DIF&0x0F)==0x0C && VIF==0x14){
          total_volume_->publish_state(decode_bcd(f,i+2,4)*0.01f);
        }

        // FLOW TEMP 0A5A
        if(temp_flow_ && (DIF&0x0F)==0x0A && VIF==0x5A){
          temp_flow_->publish_state(decode_bcd(f,i+2,2)*0.1f);
        }

        // RETURN TEMP 0A5E
        if(temp_return_ && (DIF&0x0F)==0x0A && VIF==0x5E){
          temp_return_->publish_state(decode_bcd(f,i+2,2)*0.1f);
        }

        // DELTA T 0B61
        if(temp_diff_ && (DIF&0x0F)==0x0B && VIF==0x61){
          temp_diff_->publish_state(decode_bcd(f,i+2,3)*0.01f);
        }
      }

      ESP_LOGI(TAG,"Parsing finished.");

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
