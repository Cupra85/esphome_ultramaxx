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

// ------------------------------------------------
// BCD decode
// ------------------------------------------------
float UltraMaXXComponent::decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len) {
  float value = 0;
  float mul = 1;

  for(size_t i=0;i<len;i++){
    uint8_t b=data[start+i];
    value += (b & 0x0F) * mul; mul*=10;
    value += ((b>>4)&0x0F) * mul; mul*=10;
  }
  return value;
}

// ------------------------------------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG,"UltraMaXX component started");
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG,"=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  wake_start_ = millis();
  last_send_  = 0;
  rx_buffer_.clear();

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now=millis();

  // WAKEUP
  if(state==UM_WAKEUP){

    if(now-last_send_>15){
      uint8_t buf[20];
      for(int i=0;i<20;i++) buf[i]=0x55;
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

    uint8_t dummy;
    while(this->available()) this->read_byte(&dummy);

    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state=UM_SEND;
    state_ts_=millis();
  }

  // REQUEST
  if(state==UM_SEND && millis()-state_ts_>50){

    uint8_t req[]={0x10,0x5B,0xFE,0x59,0x16};
    this->write_array(req,sizeof(req));
    this->flush();

    ESP_LOGI(TAG,"SND_UD Init gesendet");

    state=UM_RX;
    state_ts_=millis();
  }

  // RX
  if(state==UM_RX){

    while(this->available()){
      uint8_t c;
      if(this->read_byte(&c)){
        rx_buffer_.push_back(c);
      }
    }

    if(rx_buffer_.size()>10 && rx_buffer_.back()==0x16){

      ESP_LOGI(TAG,"Frame komplett -> record parsing");

      auto &f = rx_buffer_;

      // CI Field ist nach 68 L L 68 A C CI -> index 6
      size_t ptr = 7;

      while(ptr+2 < f.size()){

        uint8_t DIF = f[ptr++];
        uint8_t VIF = f[ptr++];

        int len = 0;

        switch(DIF & 0x0F){
          case 0x02: len=1; break;
          case 0x03: len=2; break;
          case 0x04: len=4; break;
          case 0x05: len=4; break;
          case 0x06: len=6; break;
          case 0x07: len=8; break;
          case 0x0C: len=4; break;
          case 0x0B: len=3; break;
          case 0x0A: len=2; break;
          default: len=0; break;
        }

        if(ptr+len >= f.size()) break;

        // -------- SERIAL --------
        if(serial_number_ && DIF==0x0C && VIF==0x78){
          float sn=decode_bcd(f,ptr,4);
          serial_number_->publish_state(sn);
        }

        // -------- ENERGY --------
        if(total_energy_ && DIF==0x04 && VIF==0x06){
          total_energy_->publish_state(decode_bcd(f,ptr,4)/1000.0);
        }

        // -------- VOLUME --------
        if(total_volume_ && DIF==0x0C && VIF==0x14){
          total_volume_->publish_state(decode_bcd(f,ptr,4)/1000.0);
        }

        // -------- POWER --------
        if(current_power_ && DIF==0x3B && VIF==0x2D){
          current_power_->publish_state(decode_bcd(f,ptr,3)/10.0);
        }

        // -------- FLOW TEMP --------
        if(temp_flow_ && DIF==0x0A && VIF==0x5A){
          temp_flow_->publish_state(decode_bcd(f,ptr,2)/10.0);
        }

        // -------- RETURN TEMP --------
        if(temp_return_ && DIF==0x0A && VIF==0x5E){
          temp_return_->publish_state(decode_bcd(f,ptr,2)/10.0);
        }

        // -------- DELTA T --------
        if(temp_diff_ && DIF==0x0B && VIF==0x61){
          temp_diff_->publish_state(decode_bcd(f,ptr,3)/100.0);
        }

        ptr += len;
      }

      rx_buffer_.clear();
      state=UM_IDLE;
    }

    if(millis()-state_ts_>6000){
      rx_buffer_.clear();
      state=UM_IDLE;
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
