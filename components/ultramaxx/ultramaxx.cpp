#include "esphome.h"
#include "ultramaxx.h"

namespace esphome {
namespace ultramaxx {

void UltramaxxComponent::setup() {
  ESP_LOGD("ultramaxx", "Ultramaxx Component setup");
}

void UltramaxxComponent::update() {
  ESP_LOGD("ultramaxx", "Ultramaxx update triggered");
  // Hier könntest du wie gehabt wake_meter() + read_meter() einbauen
}

}  // namespace ultramaxx
}  // namespace esphome
