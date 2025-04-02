#pragma once

#include "esphome/core/component.h"
#include "usb/usb_host.h"
#include "usb/usb_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

 private:
  usb_host_client_handle_t client_hdl = nullptr;
  usb_phy_handle_t phy_hdl = nullptr;
  bool usb_audio_initialized = false;
  TaskHandle_t usb_task_handle = nullptr;

  void init_usb_audio();
  bool route_audio_to_usb();

  static void usb_event_task(void *arg);
};

}  // namespace host_box3
}  // namespace esphome



