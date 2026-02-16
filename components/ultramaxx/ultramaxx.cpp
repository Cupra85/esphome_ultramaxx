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

static uint32_t le_u32(std::vector<uint8_t> &d, size_t i){
  return (uint32_t)d[i] |
         ((uint32_t)d[i+1]<<8) |
         ((uint32_t)d[i+2]<<16) |
         ((uint32_t)d[i+3]<<24);
}

static uint16_t le_u16(std::vector<uint8_t> &d, size_t i){
  return (uint16_t)d[i] | ((uint16_t)d[i+1]<<8);
}

float UltraMaXXComponent::decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len) {
  float value = 0;
  float mul = 1;
  float value=0;
  float mul=1;
  for(size_t i=0;i<len;i++){
    uint8_t b=data[start+i];
    value += (b & 0x0F) * mul; mul*=10;
    value += ((b>>4)&0x0F) * mul; mul*=10;
    value+=(b&0x0F)*mul; mul*=10;
    value+=((b>>4)&0x0F)*mul; mul*=10;
  }
  return value;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG,"UltraMaXX component started");
void UltraMaXXComponent::setup(){
  ESP_LOGI(TAG,"UltraMaXX started");
}

void UltraMaXXComponent::update() {
void UltraMaXXComponent::update(){

  ESP_LOGI(TAG,"=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  wake_start_ = millis();
  last_send_  = 0;
  wake_start_=millis();
  last_send_=0;
  rx_buffer_.clear();

  state = UM_WAKEUP;
  state=UM_WAKEUP;
}

void UltraMaXXComponent::loop() {
void UltraMaXXComponent::loop(){

  uint32_t now=millis();

  // WAKEUP
  if(state==UM_WAKEUP){

    if(now-last_send_>15){
@@ -67,7 +77,6 @@
    }
  }

  // SWITCH UART
  if(state==UM_WAIT && now-state_ts_>350){

    ESP_LOGI(TAG,"Switch to 2400 8E1");
@@ -88,7 +97,6 @@
    state_ts_=millis();
  }

  // REQUEST
  if(state==UM_SEND && millis()-state_ts_>50){

    uint8_t req[]={0x10,0x5B,0xFE,0x59,0x16};
@@ -101,7 +109,6 @@
    state_ts_=millis();
  }

  // RX PARSER
  if(state==UM_RX){

    while(this->available()){
@@ -111,49 +118,44 @@
      }
    }

    // komplettes Frame erkannt
    if(rx_buffer_.size()>10 && rx_buffer_.back()==0x16){

      auto &f = rx_buffer_;
      auto &f=rx_buffer_;

      for(size_t i=0;i+6<f.size();i++){

        // SERIAL (0C 78)
        if(serial_number_ && f[i]==0x0C && f[i+1]==0x78){
          serial_number_->publish_state(decode_bcd(f,i+2,4));
        }

        // ENERGY (04 06)
        if(total_energy_ && f[i]==0x04 && f[i+1]==0x06){
          total_energy_->publish_state(decode_bcd(f,i+2,4)/1000.0);
          total_energy_->publish_state(le_u32(f,i+2)/1000.0f);
        }

        // VOLUME (0C 14)
        if(total_volume_ && f[i]==0x0C && f[i+1]==0x14){
          total_volume_->publish_state(decode_bcd(f,i+2,4)/1000.0);
          total_volume_->publish_state(le_u32(f,i+2)/1000.0f);
        }

        // POWER (3B 2D)
        if(current_power_ && f[i]==0x3B && f[i+1]==0x2D){
          current_power_->publish_state(decode_bcd(f,i+2,3));
          uint32_t p = f[i+2] | (f[i+3]<<8) | (f[i+4]<<16);
          current_power_->publish_state((float)p);
        }

        // FLOW TEMP (0A 5A)
        if(temp_flow_ && f[i]==0x0A && f[i+1]==0x5A){
          temp_flow_->publish_state(decode_bcd(f,i+2,2));
          temp_flow_->publish_state(le_u16(f,i+2)/100.0f);
        }

        // RETURN TEMP (0A 5E)
        if(temp_return_ && f[i]==0x0A && f[i+1]==0x5E){
          temp_return_->publish_state(decode_bcd(f,i+2,2));
          temp_return_->publish_state(le_u16(f,i+2)/100.0f);
        }

        // DELTA T (0B 61)
        if(temp_diff_ && f[i]==0x0B && f[i+1]==0x61){
          temp_diff_->publish_state(decode_bcd(f,i+2,3)/100.0);
          temp_diff_->publish_state(le_u32(f,i+2)/100.0f);
        }
      }

      ESP_LOGI(TAG,"Frame parsed OK");

      rx_buffer_.clear();
      state=UM_IDLE;
    }
