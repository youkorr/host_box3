#pragma once

#include "esphome/core/component.h"
#include "esphome/components/media_player/media_player.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

// Utilisation des headers ESPHome au lieu des headers ESP-IDF
#include "esp32-hal-usb.h"

namespace esphome {
namespace host_box3 {

class HostBox3Component : public Component {
 public:
  HostBox3Component();
  ~HostBox3Component();
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
  
  void init_usb();
  bool init_usb_audio();
  
 private:
  bool usb_initialized_ = false;
  
  static void usb_event_callback(void* arg, esp_event_base_t event_base, 
                               int32_t event_id, void* event_data);
};

}  // namespace host_box3
}  // namespace esphome
