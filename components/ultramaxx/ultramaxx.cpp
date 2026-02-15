#include "ultramaxx.h"
#include "driver/uart.h"

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

static std::vector<uint8_t> frame;
static bool frame_complete = false;

static uint64_t bcd_le_to_u64_(const uint8_t *p, size_t n) {
  uint64_t v = 0;
  uint64_t mul = 1;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = p[i];
    v += (b & 0x0F) * mul; mul *= 10;
    v += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return v;
}

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

  frame.clear();
  frame_complete = false;

  wake_start = millis();
  last_send  = 0;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  if (state == UM_WAKEUP) {
    if (now - last_send >= 15) {
      uint8_t buf[20];
      for(int i=0;i<20;i++) buf[i]=0x55;
      this->write_array(buf,20);
      last_send=now;
    }
    if(now-wake_start>=2200){
      ESP_LOGI(TAG,"Wakeup end");
      state=UM_WAIT;
      state_ts=now;
    }
  }

  if(state==UM_WAIT && now-state_ts>=350){
    ESP_LOGI(TAG,"Switch to 2400 8E1");
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

    uint8_t d;
    while(this->available()) this->read_byte(&d);

    state=UM_RESET;
  }

  if(state==UM_RESET){
    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,5);
    this->flush();
    ESP_LOGI(TAG,"SND_NKE gesendet");

    state=UM_SEND;
    state_ts=millis();
  }

  if(state==UM_SEND && millis()-state_ts>50){
    uint8_t snd[]={0x10,0x5B,0xFE,0x59,0x16};
    this->write_array(snd,5);
    this->flush();
    ESP_LOGI(TAG,"SND_UD Init gesendet");

    state=UM_RX;
    state_ts=millis();
  }

  if(state==UM_RX){

    while(this->available()){
      uint8_t c;
      if(!this->read_byte(&c)) break;

      if(frame.empty()){
        if(c==0x68) frame.push_back(c);
        continue;
      }

      frame.push_back(c);

      if(frame.size()==4){
        if(frame[3]!=0x68){frame.clear();}
      }

      if(frame.size()>5 && frame.back()==0x16){
        frame_complete=true;
        break;
      }
    }

    if(frame_complete){

      // Seriennummer 0C 78
      if(serial_number_){
        for(size_t i=0;i+6<frame.size();i++){
          if(frame[i]==0x0C && frame[i+1]==0x78){
            uint64_t sn=bcd_le_to_u64_(&frame[i+2],4);
            serial_number_->publish_state((float)sn);
            break;
          }
        }
      }

      // DateTime 04 6D
      if(meter_time_){
        for(size_t i=0;i+8<frame.size();i++){
          if(frame[i]==0x04 && frame[i+1]==0x6D){
            char buf[32];
            sprintf(buf,"%02X%02X%02X%02X",
              frame[i+2],frame[i+3],frame[i+4],frame[i+5]);
            meter_time_->publish_state(buf);
            break;
          }
        }
      }

      frame.clear();
      frame_complete=false;
      state=UM_IDLE;
    }

    if(millis()-state_ts>6000){
      frame.clear();
      state=UM_IDLE;
    }
  }
}

}
}
