#pragma once

#include "esphome/core/component.h"
#include "esphome/components/media_player/media_player.h"

// Inclusions nécessaires pour le USB Audio Class Host
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

#include "../host_box3/uac_host.h"
#include "usb/usb_phy.h"

namespace esphome {
namespace host_box3 {

class HostBox3Component : public Component {
 public:
  HostBox3Component();
  
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }
  
  // Configuration du périphérique audio USB
  void init_usb_audio();
  bool route_audio_to_usb();
  
 private:
  // Variables pour gérer l'audio USB
  usb_host_client_handle_t client_hdl;
  uac_host_handle_t uac_handle;
  usb_phy_handle_t phy_hdl;  // Gestionnaire de l'interface physique USB
  bool usb_audio_initialized = false;
  
  // Tâche pour gérer les événements USB
  static void usb_event_task(void *arg);
  TaskHandle_t usb_task_handle;
};

}  // namespace host_box3
}  // namespace esphome
