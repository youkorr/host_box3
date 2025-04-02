#pragma once

#include "esphome/core/component.h"
#include "esphome/components/media_player/media_player.h"
#include "esp_audio.h"
#include "esp_usb_host.h"

namespace esphome {
namespace host_box3 {

class HostBox3Component : public Component {
 public:
  HostBox3Component();
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
  
  void init_usb_audio();
  bool route_audio_to_usb();
  
 private:
  esp_audio_handle_t audio_dev;
  usb_host_client_handle_t client_hdl;
  usb_phy_handle_t phy_hdl;
  bool usb_audio_initialized = false;
  TaskHandle_t usb_task_handle;

  static void usb_event_task(void *arg);
  static void audio_event_handler(esp_audio_handle_t audio, esp_audio_event_t event, void *user_ctx);
};

}  // namespace host_box3
}  // namespace esphome

