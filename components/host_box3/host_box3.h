#pragma once

#include "esphome/core/component.h"
#include "esphome/components/media_player/media_player.h"
#include "usb/usb_host.h"


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
  usb_host_client_handle_t client_hdl;
  usb_phy_handle_t phy_hdl;
  bool usb_audio_initialized = false;
  TaskHandle_t usb_task_handle;

  static void usb_event_task(void *arg);
};

}  // namespace host_box3
}  // namespace esphome


