#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

static const char *const TAG = "ultramaxx";

enum UMState { UM_IDLE, UM_WAKEUP, UM_WAIT, UM_SEND, UM_RX };
static UMState state = UM_IDLE;

float UltraMaXXComponent::decode_bcd(const std::vector<uint8_t> &data, size_t start, size_t len) {
    if(start+len>data.size()) return 0;
    float value=0,mul=1;
    for(size_t i=0;i<len;i++){
        uint8_t b=data[start+i];
        value+=(b&0x0F)*mul; mul*=10;
        value+=((b>>4)&0x0F)*mul; mul*=10;
    }
    return value;
}

uint32_t UltraMaXXComponent::decode_u_le(const std::vector<uint8_t> &data,size_t start,size_t len){
    if(start+len>data.size()) return 0;
    uint32_t v=0;
    for(size_t i=0;i<len;i++) v|=((uint32_t)data[start+i])<<(8*i);
    return v;
}

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

void UltraMaXXComponent::loop(){

    uint32_t now=millis();

    // RX immer sammeln
    while(available()){
        uint8_t c;
        if(read_byte(&c)){
            rx_buffer_.push_back(c);
        }
    }

    // WAKEUP
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

    // SWITCH
    if(state==UM_WAIT && now-state_ts_>350){
        ESP_LOGI(TAG,"Switch to 2400 8E1");

        parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
        parent_->load_settings();

        uint8_t reset[]={0x10,0x40,0xFE,0x3E,0x16};
        write_array(reset,sizeof(reset));
        flush();

        ESP_LOGI(TAG,"SND_NKE gesendet");

        state=UM_SEND;
        state_ts_=now;
    }

    // REQUEST
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

    // PARSER (SOBALD FRAME KOMPLETT)
    if(state==UM_RX){

        // Frame gefunden?
        if(rx_buffer_.size()>10 && rx_buffer_.front()==0x68 && rx_buffer_.back()==0x16){

            ESP_LOGI(TAG,"Parsing Frame (%d Bytes)",rx_buffer_.size());

            auto &f=rx_buffer_;

            for(size_t i=0;i+6<f.size();i++){

                if(f[i]==0x0C && f[i+1]==0x78){
                    if(serial_number_)
                        serial_number_->publish_state(decode_bcd(f,i+2,4));
                }

                else if(f[i]==0x04 && f[i+1]==0x06){
                    if(total_energy_)
                        total_energy_->publish_state(decode_u_le(f,i+2,4)*0.001f);
                }

                else if(f[i]==0x0C && f[i+1]==0x14){
                    if(total_volume_)
                        total_volume_->publish_state(decode_bcd(f,i+2,4)*0.01f);
                }

                else if(f[i]==0x0A && f[i+1]==0x5A){
                    if(temp_flow_)
                        temp_flow_->publish_state(decode_bcd(f,i+2,2)*0.1f);
                }

                else if(f[i]==0x0A && f[i+1]==0x5E){
                    if(temp_return_)
                        temp_return_->publish_state(decode_bcd(f,i+2,2)*0.1f);
                }

                else if(f[i]==0x0B && f[i+1]==0x61){
                    if(temp_diff_)
                        temp_diff_->publish_state(decode_bcd(f,i+2,3)*0.01f);
                }
            }

            rx_buffer_.clear();
            state=UM_IDLE;
            ESP_LOGI(TAG,"Update finished.");
        }

        if(now-state_ts_>9000){
            ESP_LOGW(TAG,"RX Timeout");
            rx_buffer_.clear();
            state=UM_IDLE;
        }
    }
}

} // namespace ultramaxx
} // namespace esphome
