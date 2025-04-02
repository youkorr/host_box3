#pragma once

#include "esphome/core/component.h"
#include "esphome/components/media_player/media_player.h"

// Use ESP-IDF's USB host headers
#include "driver/usb_host.h"
#include "esp_err.h"
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
  
  void init_usb_host();
  bool init_usb_audio();
  
 private:
  usb_host_client_handle_t client_hdl_;
  bool usb_initialized_ = false;
  TaskHandle_t usb_task_handle_ = nullptr;
  
  static void usb_event_task(void *arg);
};

}  // namespace host_box3
}  // namespace esphome
