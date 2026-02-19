#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState {
  UM_IDLE,
  UM_WAKEUP,
  UM_WAIT,
  UM_SEND,
  UM_RX
};

static UMState state = UM_IDLE;

// ------------------------------------------------
// Helpers
// ------------------------------------------------
float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &d, size_t s, size_t len) {
  float value = 0;
  float mul = 1;
  for(size_t i=0;i<len;i++){
    uint8_t b=d[s+i];
    value += (b & 0x0F) * mul; mul*=10;
    value += ((b>>4)&0x0F) * mul; mul*=10;
  }
  return value;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &d, size_t s, size_t len) {
  uint32_t v=0;
  for(size_t i=0;i<len;i++) v |= (uint32_t)d[s+i]<<(8*i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &d, size_t s, std::string &out){
  if(s+4>d.size()) return false;
  char buf[32];
  snprintf(buf,sizeof(buf),"%02u.%02u %02u:%02u",
    d[s+2]&0x1F,
    d[s+3]&0x0F,
    d[s]&0x1F,
    d[s+1]&0x3F
  );
  out=buf;
  return true;
}

// ------------------------------------------------
// Setup / Update
// ------------------------------------------------
void UltraMaXXComponent::setup(){
  ESP_LOGI(TAG,"UltraMaXX started");
}

void UltraMaXXComponent::update(){
  ESP_LOGI(TAG,"=== READ START ===");

  parent_->set_baud_rate(2400);
  parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  parent_->load_settings();

  rx_buffer_.clear();
  wake_start_=millis();
  last_send_=0;
  state=UM_WAKEUP;
}

// ------------------------------------------------
// LOOP
// ------------------------------------------------
void UltraMaXXComponent::loop(){

  uint32_t now=millis();

  // ================================
  // STREAM PARSER (IMMER AKTIV)
  // ================================
  while(this->available()){

    uint8_t c;
    if(!this->read_byte(&c)) return;

    // Wakeup Müll ignorieren
    if(state!=UM_RX) continue;

    rx_buffer_.push_back(c);

    auto &f = rx_buffer_;
    size_t i = f.size();

    // Wir prüfen nur die letzten Bytes
    if(i < 8) continue;

    size_t p = i - 8;

    // -------- SERIAL 0C78 ----------
    if(serial_number_ && f[p]==0x0C && f[p+1]==0x78){
      serial_number_->publish_state(decode_bcd(f,p+2,4));
    }

    // -------- ENERGY 0406 ----------
    if(total_energy_ && f[p]==0x04 && f[p+1]==0x06){
      uint32_t v=decode_u_le(f,p+2,4);
      total_energy_->publish_state(v*0.001f);
    }

    // -------- VOLUME 0C14 ----------
    if(total_volume_ && f[p]==0x0C && f[p+1]==0x14){
      total_volume_->publish_state(decode_bcd(f,p+2,4)*0.01f);
    }

    // -------- FLOW TEMP 0A5A -------
    if(temp_flow_ && f[p]==0x0A && f[p+1]==0x5A){
      temp_flow_->publish_state(decode_bcd(f,p+2,2)*0.1f);
    }

    // -------- RETURN TEMP 0A5E -----
    if(temp_return_ && f[p]==0x0A && f[p+1]==0x5E){
      temp_return_->publish_state(decode_bcd(f,p+2,2)*0.1f);
    }

    // -------- DELTA T 0B61 ---------
    if(temp_diff_ && f[p]==0x0B && f[p+1]==0x61){
      temp_diff_->publish_state(decode_bcd(f,p+2,3)*0.01f);
    }

    // -------- TIME 046D ------------
    if(meter_time_ && f[p]==0x04 && f[p+1]==0x6D){
      std::string ts;
      if(decode_cp32_datetime_(f,p+2,ts))
        meter_time_->publish_state(ts);
    }

    // Buffer klein halten (stabil!)
    if(f.size()>120) f.erase(f.begin(), f.begin()+60);
  }

  // ================================
  // STATE MACHINE (DEIN WAKEUP)
  // ================================
  if(state==UM_WAKEUP){
    if(now-last_send_>15){
      uint8_t buf[20]; memset(buf,0x55,20);
      write_array(buf,20);
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

    parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    parent_->load_settings();

    uint8_t d;
    while(this->available()) this->read_byte(&d);

    rx_buffer_.clear();

    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    write_array(reset,sizeof(reset));
    flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state=UM_SEND;
    state_ts_=now;
  }

  if(state==UM_SEND && now-state_ts_>120){
    uint8_t ctrl=fcb_toggle_?0x7B:0x5B;
    uint8_t cs=(ctrl+0xFE)&0xFF;
    uint8_t req[]={0x10,ctrl,0xFE,cs,0x16};
    write_array(req,sizeof(req));
    flush();

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    fcb_toggle_=!fcb_toggle_;
    state=UM_RX;
  }
}

} // namespace ultramaxx
} // namespace esphome
