#include "ultramaxx.h"
#include "driver/uart.h"

namespace esphome {
namespace ultramaxx {
@@ -17,69 +16,73 @@ enum UMState {

static UMState state = UM_IDLE;

// ------------------------------------------------
// BCD little endian decode
// ------------------------------------------------
float UltraMaXXComponent::decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len) {
  float value = 0;
  float mul = 1;

  for(int i=0;i<len;i++){
    uint8_t b = data[start+i];
    value += (b & 0x0F) * mul; mul *= 10;
    value += ((b>>4)&0x0F) * mul; mul *= 10;
  }
  return value;
}

// ------------------------------------------------

void UltraMaXXComponent::setup() {
  ESP_LOGI(TAG, "UltraMaXX component started");
  ESP_LOGI(TAG,"UltraMaXX component started");
}

void UltraMaXXComponent::update() {

  ESP_LOGI(TAG, "=== READ START ===");
  ESP_LOGI(TAG,"=== READ START ===");

  this->parent_->set_baud_rate(2400);
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_NONE);
  this->parent_->load_settings();

  uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

  wake_start_ = millis();
  last_send_ = 0;
  state = UM_WAKEUP;
}
  last_send_  = 0;
  rx_buffer_.clear();

float UltraMaXXComponent::decode_bcd(std::vector<uint8_t> &data, size_t start, size_t len) {
  float value = 0;
  float factor = 1;
  for (int i=len-1;i>=0;i--) {
    uint8_t b = data[start+i];
    value += (b & 0x0F) * factor;
    factor *= 10;
    value += ((b>>4)&0x0F) * factor;
    factor *= 10;
  }
  return value;
  state = UM_WAKEUP;
}

void UltraMaXXComponent::loop() {

  uint32_t now = millis();

  // WAKEUP
  if (state == UM_WAKEUP) {
    if (now - last_send_ >= 15) {
  // ---------------- WAKEUP ----------------
  if(state==UM_WAKEUP){

    if(now-last_send_>15){
      uint8_t buf[20];
      for(int i=0;i<20;i++) buf[i]=0x55;
      this->write_array(buf,20);
      last_send_=now;
    }

    if (now - wake_start_ >= 2200) {
    if(now-wake_start_>2200){
      ESP_LOGI(TAG,"Wakeup end");
      state = UM_WAIT;
      state_ts_ = now;
      state=UM_WAIT;
      state_ts_=now;
    }
  }

  // WAIT
  if (state==UM_WAIT && now-state_ts_>=350) {
  // ---------------- SWITCH ----------------
  if(state==UM_WAIT && now-state_ts_>350){

    ESP_LOGI(TAG,"Switch to 2400 8E1");

    this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
    this->parent_->load_settings();

    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

    rx_buffer_.clear();
    uint8_t dummy;
    while(this->available()) this->read_byte(&dummy);

    uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
    this->write_array(reset,sizeof(reset));
@@ -91,11 +94,11 @@ void UltraMaXXComponent::loop() {
    state_ts_=millis();
  }

  // SEND REQUEST
  if (state==UM_SEND && millis()-state_ts_>50) {
  // ---------------- REQUEST ----------------
  if(state==UM_SEND && millis()-state_ts_>50){

    uint8_t snd_ud[]={0x10,0x5B,0xFE,0x59,0x16};
    this->write_array(snd_ud,sizeof(snd_ud));
    uint8_t req[]={0x10,0x5B,0xFE,0x59,0x16};
    this->write_array(req,sizeof(req));
    this->flush();

    ESP_LOGI(TAG,"SND_UD Init gesendet");
@@ -104,8 +107,8 @@ void UltraMaXXComponent::loop() {
    state_ts_=millis();
  }

  // RECEIVE
  if (state==UM_RX) {
  // ---------------- RECEIVE ----------------
  if(state==UM_RX){

    while(this->available()){
      uint8_t c;
@@ -114,65 +117,62 @@ void UltraMaXXComponent::loop() {
      }
    }

    // vollständiges Frame erkannt?
    if(rx_buffer_.size()>10 && rx_buffer_.back()==0x16){

      ESP_LOGI(TAG,"Frame komplett, parse Daten");
      ESP_LOGI(TAG,"Frame komplett -> Parsing");

      for(size_t i=0;i<rx_buffer_.size()-5;i++){
      auto &f = rx_buffer_;

      for(size_t i=0;i+6<f.size();i++){

        // Seriennummer
        if(rx_buffer_[i]==0x72){
          float sn = decode_bcd(rx_buffer_,i+1,4);
          if(serial_number_) serial_number_->publish_state(sn);
        if(serial_number_ && f[i]==0x0C && f[i+1]==0x78){
          float sn = decode_bcd(f,i+2,4);
          serial_number_->publish_state(sn);
        }

        // Energie
        if(rx_buffer_[i]==0x04 && rx_buffer_[i+1]==0x06){
          float energy = decode_bcd(rx_buffer_,i+2,4)/1000.0;
          if(total_energy_) total_energy_->publish_state(energy);
        // Gesamtenergie
        if(total_energy_ && f[i]==0x04 && f[i+1]==0x06){
          float energy = decode_bcd(f,i+2,4)/1000.0;
          total_energy_->publish_state(energy);
        }

        // Volumen
        if(rx_buffer_[i]==0x0C && rx_buffer_[i+1]==0x14){
          float vol = decode_bcd(rx_buffer_,i+2,4)/100.0;
          if(total_volume_) total_volume_->publish_state(vol);
        // Gesamtvolumen
        if(total_volume_ && f[i]==0x0C && f[i+1]==0x14){
          float vol = decode_bcd(f,i+2,4)/1000.0;
          total_volume_->publish_state(vol);
        }

        // Leistung
        if(rx_buffer_[i]==0x3B && rx_buffer_[i+1]==0x2D){
          float p = decode_bcd(rx_buffer_,i+2,3)/10.0;
          if(current_power_) current_power_->publish_state(p);
        // Leistung  (WICHTIG: 3B 2D!)
        if(current_power_ && f[i]==0x3B && f[i+1]==0x2D){
          float p = decode_bcd(f,i+2,3)/10.0;
          current_power_->publish_state(p);
        }

        // Vorlauf
        if(rx_buffer_[i]==0x0A && rx_buffer_[i+1]==0x5A){
          float tf = decode_bcd(rx_buffer_,i+2,2)/10.0;
          if(temp_flow_) temp_flow_->publish_state(tf);
        if(temp_flow_ && f[i]==0x0A && f[i+1]==0x5A){
          float tf = decode_bcd(f,i+2,2)/10.0;
          temp_flow_->publish_state(tf);
        }

        // Rücklauf
        if(rx_buffer_[i]==0x0A && rx_buffer_[i+1]==0x5E){
          float tr = decode_bcd(rx_buffer_,i+2,2)/10.0;
          if(temp_return_) temp_return_->publish_state(tr);
        if(temp_return_ && f[i]==0x0A && f[i+1]==0x5E){
          float tr = decode_bcd(f,i+2,2)/10.0;
          temp_return_->publish_state(tr);
        }

        // DeltaT
        if(rx_buffer_[i]==0x0B && rx_buffer_[i+1]==0x61){
          float td = decode_bcd(rx_buffer_,i+2,3)/100.0;
          if(temp_diff_) temp_diff_->publish_state(td);
        if(temp_diff_ && f[i]==0x0B && f[i+1]==0x61){
          float td = decode_bcd(f,i+2,3)/100.0;
          temp_diff_->publish_state(td);
        }

        // Datum/Zeit
        if(rx_buffer_[i]==0x04 && rx_buffer_[i+1]==0x6D){
        // Zählerzeit
        if(meter_time_ && f[i]==0x04 && f[i+1]==0x6D){
          char buf[32];
          sprintf(buf,"20%02X-%02X-%02X %02X:%02X",
            rx_buffer_[i+2],
            rx_buffer_[i+3],
            rx_buffer_[i+4],
            rx_buffer_[i+5],
            rx_buffer_[i+6]);
          if(meter_time_) meter_time_->publish_state(buf);
            f[i+2],f[i+3],f[i+4],f[i+5],f[i+6]);
          meter_time_->publish_state(buf);
        }
      }

@@ -182,6 +182,7 @@ void UltraMaXXComponent::loop() {

    if(millis()-state_ts_>6000){
      state=UM_IDLE;
      rx_buffer_.clear();
    }
  }
}
