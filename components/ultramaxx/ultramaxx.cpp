#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";
static const char *const VERSION = "UltraMaXX Parser v5.18";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

/* ---------------- DECODER ---------------- */

float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &d,size_t s,size_t l) const {
  float v=0,m=1;
  for(size_t i=0;i<l;i++){
    uint8_t b=d[s+i];
    v+=(b&0x0F)*m; m*=10;
    v+=((b>>4)&0x0F)*m; m*=10;
  }
  return v;
}

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &d,size_t s,size_t l) const {
  uint32_t v=0;
  for(size_t i=0;i<l;i++) v|=((uint32_t)d[s+i])<<(8*i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &d,size_t s,std::string &out) const {
  if(s+4>d.size()) return false;
  char buf[32];
  sprintf(buf,"%02u.%02u %02u:%02u",d[s+1],d[s],d[s+3],d[s+2]);
  out=buf;
  return true;
}

/* ---------------- SETUP ---------------- */

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG,"UltraMaXX Parser gestartet (%s)",VERSION);
}

void UltraMaXXComponent::update() {
  ESP_LOGI(TAG,"=== READ START (%s) ===",VERSION);

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  in_frame_=false;
  expected_len_=0;

  wake_start_=millis();
  last_send_=0;
  state=UM_WAKEUP;
}

/* ---------------- PARSER ---------------- */

void UltraMaXXComponent::parse_and_publish_(const std::vector<uint8_t> &b){

  ESP_LOGI(TAG,"Frame komplett (%u Bytes)",b.size());

  for(size_t i=0;i+6<b.size();i++){

    if(!got_serial_ && b[i]==0x0C && b[i+1]==0x78){
      float sn=decode_bcd_(b,i+2,4);
      ESP_LOGI(TAG,"SERIAL %.0f",sn);
      if(serial_number_) serial_number_->publish_state(sn);
      got_serial_=true;
    }

    if(!got_energy_ && b[i]==0x04 && b[i+1]==0x06){
      float e=decode_u_le_(b,i+2,4)*0.001f;
      ESP_LOGI(TAG,"ENERGY %.3f",e);
      if(total_energy_) total_energy_->publish_state(e);
      got_energy_=true;
    }

    if(!got_volume_ && b[i]==0x0C && b[i+1]==0x14){
      float v=decode_bcd_(b,i+2,4)*0.01f;
      ESP_LOGI(TAG,"VOLUME %.2f",v);
      if(total_volume_) total_volume_->publish_state(v);
      got_volume_=true;
    }

    if(!got_tflow_ && b[i]==0x0A && b[i+1]==0x5A){
      float t=decode_bcd_(b,i+2,2)*0.1f;
      ESP_LOGI(TAG,"FLOW %.1f",t);
      if(temp_flow_) temp_flow_->publish_state(t);
      got_tflow_=true;
    }

    if(!got_tret_ && b[i]==0x0A && b[i+1]==0x5E){
      float t=decode_bcd_(b,i+2,2)*0.1f;
      ESP_LOGI(TAG,"RETURN %.1f",t);
      if(temp_return_) temp_return_->publish_state(t);
      got_tret_=true;
    }

    if(!got_tdiff_ && b[i]==0x0B && b[i+1]==0x61){
      float dt=decode_bcd_(b,i+2,3)*0.01f;
      ESP_LOGI(TAG,"DELTA %.2f",dt);
      if(temp_diff_) temp_diff_->publish_state(dt);
      got_tdiff_=true;
    }

    if(!got_time_ && b[i]==0x04 && b[i+1]==0x6D){
      std::string ts;
      if(decode_cp32_datetime_(b,i+2,ts)){
        ESP_LOGI(TAG,"TIME %s",ts.c_str());
        if(meter_time_) meter_time_->publish_state(ts);
      }
      got_time_=true;
    }
  }
}

/* ---------------- LOOP ---------------- */

void UltraMaXXComponent::loop(){

  uint32_t now=millis();

  while(this->available()){
    uint8_t c;
    this->read_byte(&c);

    /* --- FRAME START --- */
    if(!in_frame_){
      if(c==0x68){
        rx_buffer_.clear();
        rx_buffer_.push_back(c);
        in_frame_=true;
      }
      continue;
    }

    rx_buffer_.push_back(c);

    /* Länge bestimmen */
    if(rx_buffer_.size()==4){
      expected_len_=rx_buffer_[1]+6;
      ESP_LOGD(TAG,"Longframe erkannt, erwartete Länge=%u",expected_len_);
    }

    /* Frame komplett */
    if(expected_len_>0 && rx_buffer_.size()>=expected_len_){
      parse_and_publish_(rx_buffer_);
      rx_buffer_.clear();
      in_frame_=false;
      expected_len_=0;
    }
  }

  /* ---- STATE MACHINE unverändert ---- */

  if(state==UM_WAKEUP){
    if(now-last_send_>15){
      uint8_t b[20]; memset(b,0x55,20);
      this->write_array(b,20);
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

    rx_buffer_.clear();
    in_frame_=false;
    expected_len_=0;

    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state=UM_SEND;
    state_ts_=now;
  }

  if(state==UM_SEND && now-state_ts_>150){
    uint8_t ctrl=fcb_toggle_?0x7B:0x5B;
    uint8_t cs=(ctrl+0xFE)&0xFF;

    uint8_t req[]={0x10,ctrl,0xFE,cs,0x16};
    this->write_array(req,sizeof(req));
    this->flush();

    ESP_LOGI(TAG,"REQ_UD2 gesendet");

    fcb_toggle_=!fcb_toggle_;
    state=UM_RX;
  }
}

} // namespace
} // namespace
