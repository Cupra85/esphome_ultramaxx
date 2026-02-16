#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_RESET,
  UM_WAIT_ACK,
  UM_SEND,
  UM_RX
};

static UMState state = UM_IDLE;

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG,"UltraMaXX component started");
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG,"=== READ START ===");

  parent_->set_baud_rate(2400);
  parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  parent_->load_settings();

  wake_start_ = millis();
  last_send_ = 0;
  rx_buffer_.clear();

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // WAKEUP
  if(state==UM_WAKEUP){

    if(now-last_send_>15){
      uint8_t buf[20];
      memset(buf,0x55,20);
      write_array(buf,20);
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

    parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    parent_->load_settings();

    uint8_t dummy;
    while(available()) read_byte(&dummy);

    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    write_array(reset,sizeof(reset));
    flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state = UM_WAIT_ACK;
    state_ts_ = millis();
  }

  // ⭐ WICHTIG: AUF ACK WARTEN
  if(state==UM_WAIT_ACK){

    while(available()){
      uint8_t c;
      if(read_byte(&c)){
        if(c==0xE5){
          ESP_LOGI(TAG,"ACK E5 erhalten");
          state=UM_SEND;
          state_ts_=millis();
        }
      }
    }

    // Fallback falls ACK nicht kommt
    if(millis()-state_ts_>800){
      ESP_LOGW(TAG,"ACK Timeout → trotzdem weiter");
      state=UM_SEND;
      state_ts_=millis();
    }
  }

  // REQ_UD2
  if(state==UM_SEND && millis()-state_ts_>50){

    uint8_t req[]={0x10,0x7B,0xFE,0x79,0x16};
    write_array(req,sizeof(req));
    flush();

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    state=UM_RX;
    state_ts_=millis();
  }

  // RX
  if(state==UM_RX){

    while(available()){
      uint8_t c;
      if(read_byte(&c)){
        rx_buffer_.push_back(c);
      }
    }

    if(rx_buffer_.size()>10 && rx_buffer_.back()==0x16){
      ESP_LOGI(TAG,"Frame erhalten (%d bytes)",rx_buffer_.size());
      rx_buffer_.clear();
      state=UM_IDLE;
    }

    if(millis()-state_ts_>6000){
      ESP_LOGW(TAG,"RX Timeout");
      rx_buffer_.clear();
      state=UM_IDLE;
    }
  }
}

} }
