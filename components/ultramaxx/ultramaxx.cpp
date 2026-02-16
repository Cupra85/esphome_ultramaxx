#include "ultramaxx.h"
#include "driver/uart.h"

namespace esphome {
namespace ultramaxx {
@@ -20,20 +19,64 @@ static uint32_t wake_start = 0;
static uint32_t last_send  = 0;
static uint32_t state_ts   = 0;

static std::vector<uint8_t> frame;
static bool frame_complete = false;
// ----------------------------------------------------
// BCD decode helper (little endian)
// ----------------------------------------------------
static float decode_bcd_le(const uint8_t *p, int bytes, float div = 1.0f) {
  uint32_t value = 0;
  uint32_t mul = 1;

static uint64_t bcd_le_to_u64_(const uint8_t *p, size_t n) {
  uint64_t v = 0;
  uint64_t mul = 1;
  for (size_t i = 0; i < n; i++) {
  for (int i = 0; i < bytes; i++) {
    uint8_t b = p[i];
    v += (b & 0x0F) * mul; mul *= 10;
    v += ((b >> 4) & 0x0F) * mul; mul *= 10;
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b >> 4) & 0x0F) * mul; mul *= 10;
  }
  return v;

  return value / div;
}

// ----------------------------------------------------
// Type-F DateTime decode (VIF 0x6D)
// ----------------------------------------------------
static bool decode_type_f_datetime(const uint8_t *p4, char *out, size_t len) {

  const uint8_t b0 = p4[0];
  const uint8_t b1 = p4[1];
  const uint8_t b2 = p4[2];
  const uint8_t b3 = p4[3];

  if (b0 & 0x80) return false;

  uint8_t minute = (b0 & 0x3F);
  uint8_t hour   = (b1 & 0x1F);
  uint8_t hy     = (b1 >> 5) & 0x03;
  uint8_t su     = (b1 >> 7) & 0x01;

  uint8_t day    = (b2 & 0x1F);
  uint8_t y_low  = (b2 >> 5) & 0x07;

  uint8_t month  = (b3 & 0x0F);
  uint8_t y_high = (b3 >> 4) & 0x0F;

  uint8_t year_in_century = (y_high << 3) | y_low;

  int century = 1900;
  if (hy == 1) century = 2000;
  else if (hy == 2) century = 2100;
  else if (hy == 3) century = 2200;

  int year = century + year_in_century;

  if (su)
    snprintf(out, len, "%04d-%02u-%02u %02u:%02u DST", year, month, day, hour, minute);
  else
    snprintf(out, len, "%04d-%02u-%02u %02u:%02u", year, month, day, hour, minute);

  return true;
}

// ----------------------------------------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
}
@@ -46,126 +89,129 @@ void UltraMaXXComponent::update() {
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

  frame.clear();
  frame_complete = false;

  wake_start = millis();
  last_send  = 0;
  rx_buffer_.clear();

  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // ---------------- WAKEUP ----------------
  if (state == UM_WAKEUP) {

    if (now - last_send >= 15) {
      uint8_t buf[20];
      for(int i=0;i<20;i++) buf[i]=0x55;
      for (int i=0;i<20;i++) buf[i]=0x55;
      this->write_array(buf,20);
      last_send=now;
      last_send = now;
    }
    if(now-wake_start>=2200){

    if (now - wake_start >= 2200) {
      ESP_LOGI(TAG,"Wakeup end");
      state=UM_WAIT;
      state_ts=now;
      state = UM_WAIT;
      state_ts = now;
    }
  }

  if(state==UM_WAIT && now-state_ts>=350){
  // ---------------- SWITCH ----------------
  if (state == UM_WAIT && now - state_ts >= 350) {

    ESP_LOGI(TAG,"Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();
    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

    uint8_t d;
    while(this->available()) this->read_byte(&d);
    uint8_t dummy;
    while (this->available()) this->read_byte(&dummy);

    state=UM_RESET;
    state = UM_RESET;
  }

  if(state==UM_RESET){
    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,5);
  // ---------------- SND_NKE ----------------
  if (state == UM_RESET) {

    uint8_t reset[] = {0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
    this->flush();

    ESP_LOGI(TAG,"SND_NKE gesendet");

    state=UM_SEND;
    state_ts=millis();
    state = UM_SEND;
    state_ts = millis();
  }

  if(state==UM_SEND && millis()-state_ts>50){
    uint8_t snd[]={0x10,0x5B,0xFE,0x59,0x16};
    this->write_array(snd,5);
  // ---------------- SND_UD ----------------
  if (state == UM_SEND && millis()-state_ts > 50) {

    uint8_t snd_ud[] = {0x10,0x5B,0xFE,0x59,0x16};
    this->write_array(snd_ud,sizeof(snd_ud));
    this->flush();

    ESP_LOGI(TAG,"SND_UD Init gesendet");

    state=UM_RX;
    state_ts=millis();
    state = UM_RX;
    state_ts = millis();
  }

  if(state==UM_RX){
  // ---------------- RX PARSER ----------------
  if (state == UM_RX) {

    while(this->available()){
    while (this->available()) {
      uint8_t c;
      if(!this->read_byte(&c)) break;

      if(frame.empty()){
        if(c==0x68) frame.push_back(c);
        continue;
      if (this->read_byte(&c)) {
        rx_buffer_.push_back(c);
      }
    }

      frame.push_back(c);
    if (!rx_buffer_.empty() && rx_buffer_.back()==0x16) {

      if(frame.size()==4){
        if(frame[3]!=0x68){frame.clear();}
      }
      auto &f = rx_buffer_;

      if(frame.size()>5 && frame.back()==0x16){
        frame_complete=true;
        break;
      }
    }
      for (size_t i=0;i+8<f.size();i++) {

    if(frame_complete){
        if (serial_number_ && f[i]==0x0C && f[i+1]==0x78)
          serial_number_->publish_state(decode_bcd_le(&f[i+2],4));

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
        if (total_energy_ && f[i]==0x04 && f[i+1]==0x06)
          total_energy_->publish_state(decode_bcd_le(&f[i+2],4,1000.0f));

        if (total_volume_ && f[i]==0x0C && f[i+1]==0x14)
          total_volume_->publish_state(decode_bcd_le(&f[i+2],4,1000.0f));

        if (current_power_ && f[i]==0x0B && f[i+1]==0x2D)
          current_power_->publish_state(decode_bcd_le(&f[i+2],3,10.0f));

        if (temp_flow_ && f[i]==0x0A && f[i+1]==0x5A)
          temp_flow_->publish_state(decode_bcd_le(&f[i+2],2,10.0f));

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
        if (temp_return_ && f[i]==0x0A && f[i+1]==0x5E)
          temp_return_->publish_state(decode_bcd_le(&f[i+2],2,10.0f));

        if (temp_diff_ && f[i]==0x0B && f[i+1]==0x61)
          temp_diff_->publish_state(decode_bcd_le(&f[i+2],3,100.0f));

        if (meter_time_ && f[i]==0x04 && f[i+1]==0x6D) {
          char ts[32];
          if (decode_type_f_datetime(&f[i+2],ts,sizeof(ts)))
            meter_time_->publish_state(ts);
        }
      }

      frame.clear();
      frame_complete=false;
      state=UM_IDLE;
      rx_buffer_.clear();
      state = UM_IDLE;
    }

    if(millis()-state_ts>6000){
      frame.clear();
      state=UM_IDLE;
    if (millis()-state_ts>6000) {
      ESP_LOGI(TAG,"RX timeout");
      rx_buffer_.clear();
      state = UM_IDLE;
    }
  }
}

}
}
} // namespace ultramaxx
} // namespace esphome
