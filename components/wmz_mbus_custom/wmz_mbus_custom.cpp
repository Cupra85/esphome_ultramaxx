#include "wmz_mbus_custom.h"
#include "esphome/core/log.h"
#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace wmz_mbus_custom {

static const char *const TAG = "wmz";

WMZComponent::WMZComponent(int tx_pin, int rx_pin, uint32_t update_interval_ms)
    : PollingComponent(update_interval_ms), tx_pin_(tx_pin), rx_pin_(rx_pin) {}

void WMZComponent::setup() {
  ESP_LOGI(TAG, "WMZComponent setup: TX=%d RX=%d", tx_pin_, rx_pin_);
}

void WMZComponent::uart_configure(int baud, uart_parity_t parity) {
  uart_driver_delete(uart_num_);
  uart_config_t cfg = {};
  cfg.baud_rate = baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = parity;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  uart_param_config(uart_num_, &cfg);
  uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num_, 4096, 4096, 0, NULL, 0);
}

void WMZComponent::uart_write(const uint8_t *data, size_t len, uint32_t wait_ms) {
  if (len == 0) return;
  uart_write_bytes(uart_num_, (const char *)data, len);
  uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(wait_ms));
}

int WMZComponent::uart_read(uint8_t *data, size_t maxlen, uint32_t timeout_ms) {
  return uart_read_bytes(uart_num_, data, maxlen, pdMS_TO_TICKS(timeout_ms));
}

// Wake-Up: 0x55 für 2,2 s @ 2400 8N1, dann 100 ms Ruhe
void WMZComponent::wake_up() {
  this->uart_configure(2400, UART_PARITY_DISABLE);
  uint8_t block[64];
  memset(block, 0x55, sizeof(block));

  uint64_t start = esp_timer_get_time();  // µs seit Boot
  while ((esp_timer_get_time() - start) / 1000 < 2200) {
    this->uart_write(block, sizeof(block), 0);
  }

  vTaskDelay(pdMS_TO_TICKS(100));  // 100 ms Ruhe
}

bool WMZComponent::send_init_expect_e5() {
  this->uart_configure(2400, UART_PARITY_EVEN);
  const uint8_t snd_nke[] = {0x10, 0x40, 0x00, 0x40, 0x16};
  this->uart_write(snd_nke, sizeof(snd_nke), 50);

  uint8_t b;
  int n = this->uart_read(&b, 1, 150);
  if (n == 1 && b == 0xE5) {
    ESP_LOGI(TAG, "ACK E5 erhalten");
    return true;
  }
  ESP_LOGW(TAG, "Kein E5-ACK auf SND_NKE");
  return false;
}

void WMZComponent::send_req_ud2() {
  const uint8_t req_ud2[] = {0x10, 0x5B, 0xFE, 0x59, 0x16};
  this->uart_write(req_ud2, sizeof(req_ud2), 50);
}

std::vector<uint8_t> WMZComponent::read_frame(uint32_t window_ms) {
  std::vector<uint8_t> out;
  out.reserve(512);
  uint64_t t0 = esp_timer_get_time();  // µs

  uint8_t tmp[256];
  while ((esp_timer_get_time() - t0) / 1000 < window_ms) {
    int n = this->uart_read(tmp, sizeof(tmp), 50);
    if (n > 0) out.insert(out.end(), tmp, tmp + n);
  }

  if (!out.empty())
    ESP_LOGD(TAG, "RX %u Bytes", (unsigned) out.size());

  return out;
}

void WMZComponent::parse_and_publish(const std::vector<uint8_t> &buf) {
  if (buf.empty()) return;

  auto find_vif = [&](uint8_t vif, uint32_t &val) -> bool {
    for (size_t i = 0; i + 5 < buf.size(); i++) {
      if (buf[i] == 0x04 && buf[i + 1] == vif) {
        val = u32le(&buf[i + 2]);
        return true;
      }
    }
    return false;
  };

  uint32_t v;
  if (find_vif(0x06, v)) heat_energy_kwh_->publish_state(v / 1000.0f);
  if (find_vif(0x13, v)) volume_m3_->publish_state(v / 1000.0f);
  if (find_vif(0x5B, v)) flow_temp_c_->publish_state(v / 100.0f);
  if (find_vif(0x5F, v)) return_temp_c_->publish_state(v / 100.0f);
  if (find_vif(0x2B, v)) power_w_->publish_state(v);
  if (find_vif(0x3B, v)) flow_lh_->publish_state(v);
}

void WMZComponent::update() {
  ESP_LOGI(TAG, "WMZ Poll...");
  wake_up();
  send_init_expect_e5();
  send_req_ud2();
  auto data = read_frame(1200);

  if (data.empty()) {
    ESP_LOGW(TAG, "Keine Antwort vom Zähler");
    if (debug_frame_ != nullptr)
      debug_frame_->publish_state("No data");
    return;
  }

  // Hexdump erzeugen
  std::string hex;
  for (uint8_t b : data) {
    char buf[4];
    sprintf(buf, "%02X ", b);
    hex += buf;
  }

  if (debug_frame_ != nullptr)
    debug_frame_->publish_state(hex);

  parse_and_publish(data);
}

}  // namespace wmz_mbus_custom
}  // namespace esphome
