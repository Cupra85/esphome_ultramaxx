#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &d, size_t s, size_t l) {
  float v = 0, m = 1;
  for (size_t i = 0; i < l; i++) {
    uint8_t b = d[s+i];
    v += (b & 0x0F) * m; m *= 10;
    v += ((b >> 4) & 0x0F) * m; m *= 10;
  }
  return v;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &d,size_t s,size_t l){
  uint32_t v=0;
  for(size_t i=0;i<l;i++) v |= (uint32_t)d[s+i] << (8*i);
  return v;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG,"UltraMaXX component started");
}

void UltraMaXXComponent::update() {

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

  // -------- RX SAMMLER --------
  while(this->available()){
    uint8_t c;
    if(this->read_byte(&c)){
      if(state==UM_RX){
        rx_buffer_.push_back(c);
        last_rx_byte_ = now;
      }
    }
  }

  // -------- WAKEUP --------
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

  // -------- SWITCH UART --------
  if(state==UM_WAIT && now-state_ts_>350){

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uint8_t d;
    while(this->available()) this->read_byte(&d); // BUFFER LEEREN !!!

    rx_buffer_.clear();

    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state=UM_SEND;
    state_ts_=now;
  }

  // -------- REQ_UD2 --------
  if(state==UM_SEND && now-state_ts_>150){

    uint8_t ctrl = fcb_toggle_ ? 0x7B : 0x5B;
    uint8_t cs = (ctrl+0xFE)&0xFF;
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

  // -------- FRAME DETECTOR --------
  if(state==UM_RX){

    if(rx_buffer_.size()<6) return;

    // Suche Longframe
    int start=-1;
    for(size_t i=0;i<rx_buffer_.size()-3;i++){
      if(rx_buffer_[i]==0x68 && rx_buffer_[i+3]==0x68){
        start=i;
        break;
      }
    }
    if(start<0) return;

    uint8_t len = rx_buffer_[start+1];

    if(start+len+6 > rx_buffer_.size()) return; // noch nicht komplett

    if(rx_buffer_[start+len+5] != 0x16) return;

    ESP_LOGI(TAG,"Parsing Frame (%d Bytes)",len+6);

    auto &f = rx_buffer_;

    for(size_t i=start;i<f.size()-6;i++){

      if(f[i]==0x0C && f[i+1]==0x78){
        if(serial_number_) serial_number_->publish_state(decode_bcd(f,i+2,4));
      }
      else if(f[i]==0x04 && f[i+1]==0x06){
        if(total_energy_) total_energy_->publish_state(decode_u_le(f,i+2,4)*0.001f);
      }
      else if(f[i]==0x0C && f[i+1]==0x14){
        if(total_volume_) total_volume_->publish_state(decode_bcd(f,i+2,4)*0.01f);
      }
      else if(f[i]==0x0A && f[i+1]==0x5A){
        if(temp_flow_) temp_flow_->publish_state(decode_bcd(f,i+2,2)*0.1f);
      }
      else if(f[i]==0x0A && f[i+1]==0x5E){
        if(temp_return_) temp_return_->publish_state(decode_bcd(f,i+2,2)*0.1f);
      }
      else if(f[i]==0x0B && f[i+1]==0x61){
        if(temp_diff_) temp_diff_->publish_state(decode_bcd(f,i+2,3)*0.01f);
      }
    }

    rx_buffer_.clear();
    state=UM_IDLE;

    ESP_LOGI(TAG,"Update finished.");
  }

  if(state==UM_RX && now-state_ts_>10000){
    ESP_LOGW(TAG,"RX Timeout");
    rx_buffer_.clear();
    state=UM_IDLE;
  }
}

} // namespace ultramaxx
} // namespace esphome
