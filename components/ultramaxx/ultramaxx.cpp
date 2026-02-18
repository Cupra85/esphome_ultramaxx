#include "ultramaxx.h"
#include <cstring>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

// ---------- Helpers ----------
float UltraMaXXComponent::decode_bcd_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
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

uint32_t UltraMaXXComponent::decode_u_le_(const std::vector<uint8_t> &data, size_t start, size_t len) const {
  if (start + len > data.size()) return 0;
  uint32_t v = 0;
  for (size_t i = 0; i < len; i++) v |= (uint32_t)data[start + i] << (8 * i);
  return v;
}

bool UltraMaXXComponent::decode_cp32_datetime_(const std::vector<uint8_t> &data, size_t start, std::string &out) const {
  if (start + 4 > data.size()) return false;
  int minute = data[start] & 0x3F;
  int hour   = data[start+1] & 0x1F;
  int day    = data[start+2] & 0x1F;
  int month  = ((data[start+2] & 0xE0) >> 5) | ((data[start+3] & 0x01) << 3);
  int year   = (data[start+3] >> 1) + 2000;
  char tmp[32];
  snprintf(tmp,sizeof(tmp),"%04d-%02d-%02d %02d:%02d",year,month,day,hour,minute);
  out = tmp;
  return true;
}

bool UltraMaXXComponent::extract_long_frame_(const std::vector<uint8_t> &buf, std::vector<uint8_t> &out) const {
  for (size_t start=0; start+6<=buf.size(); start++) {
    if (buf[start]!=0x68) continue;
    uint8_t l1=buf[start+1], l2=buf[start+2];
    if (l1!=l2 || buf[start+3]!=0x68) continue;
    size_t total=l1+6;
    if (start+total>buf.size()) return false;
    if (buf[start+total-1]!=0x16) continue;
    out.assign(buf.begin()+start,buf.begin()+start+total);
    return true;
  }
  return false;
}

// ---------- Component ----------
void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG,"UltraMaXX started");
}

void UltraMaXXComponent::update() {
  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  rx_buffer_.clear();
  wake_start_=millis();
  last_send_=0;
  state_=UM_WAKEUP;
  state_ts_=millis();
}

void UltraMaXXComponent::loop() {
  uint32_t now=millis();

  while(this->available()){
    uint8_t c;
    if(this->read_byte(&c)){
      if(state_==UM_RX){
        rx_buffer_.push_back(c);
        last_rx_byte_=now;
      }
    }
  }

  if(state_==UM_WAKEUP){
    if(now-last_send_>15){
      uint8_t buf[20]; memset(buf,0x55,20);
      this->write_array(buf,20);
      last_send_=now;
    }
    if(now-wake_start_>2200){
      state_=UM_WAIT;
      state_ts_=now;
    }
    return;
  }

  if(state_==UM_WAIT && now-state_ts_>350){
    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();
    uint8_t d; while(this->available()) this->read_byte(&d);

    const uint8_t snd[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(snd,sizeof(snd));
    this->flush();

    state_=UM_SEND;
    state_ts_=now;
    return;
  }

  if(state_==UM_SEND && now-state_ts_>120){
    uint8_t ctrl=fcb_toggle_?0x7B:0x5B;
    uint8_t cs=(ctrl+0xFE)&0xFF;
    const uint8_t req[]={0x10,ctrl,0xFE,cs,0x16};
    this->write_array(req,sizeof(req));
    this->flush();
    fcb_toggle_=!fcb_toggle_;
    rx_buffer_.clear();
    state_=UM_RX;
    state_ts_=now;
    last_rx_byte_=now;
    return;
  }

  if(state_==UM_RX){
    if(rx_buffer_.size()>10 && (now-last_rx_byte_)>200){
      std::vector<uint8_t> frame;
      if(extract_long_frame_(rx_buffer_,frame)){
        size_t rec=19;
        size_t end=frame.size()-2;

        while(rec+2<=end){
          uint8_t dif=frame[rec++];
          uint8_t vif=frame[rec++];

          int len=0;
          switch(dif&0x0F){
            case 0x0A: len=2; break;
            case 0x0B: len=3; break;
            case 0x0C: len=4; break;
            case 0x04: len=4; break;
            default: break;
          }
          if(rec+len>end) break;

          if(serial_number_ && (dif&0x0F)==0x0C && vif==0x78)
            serial_number_->publish_state(decode_bcd_(frame,rec,4));

          if(total_energy_ && (dif&0x0F)==0x04 && vif==0x06)
            total_energy_->publish_state(decode_u_le_(frame,rec,4)*0.001f);

          if(total_volume_ && (dif&0x0F)==0x0C && vif==0x14)
            total_volume_->publish_state(decode_bcd_(frame,rec,4)*0.01f);

          if(current_power_ && (dif&0x0F)==0x0B && vif==0x2D)
            current_power_->publish_state(decode_bcd_(frame,rec,3)*0.1f);

          if(temp_flow_ && (dif&0x0F)==0x0A && vif==0x5A)
            temp_flow_->publish_state(decode_bcd_(frame,rec,2)*0.1f);

          if(temp_return_ && (dif&0x0F)==0x0A && vif==0x5E)
            temp_return_->publish_state(decode_bcd_(frame,rec,2)*0.1f);

          if(temp_diff_ && (dif&0x0F)==0x0B && vif==0x61)
            temp_diff_->publish_state(decode_bcd_(frame,rec,3)*0.01f);

          rec+=len;
        }
      }
      rx_buffer_.clear();
      state_=UM_IDLE;
      return;
    }

    if(now-state_ts_>4000){
      state_=UM_IDLE;
      rx_buffer_.clear();
    }
  }
}

} // namespace ultramaxx
} // namespace esphome
