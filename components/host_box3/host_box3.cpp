#include "host_box3.h"
#include "esphome/core/log.h"

static const char *TAG = "host_box3";

namespace esphome {
namespace host_box3 {

HostBox3Component::HostBox3Component() 
  : client_hdl(nullptr), phy_hdl(nullptr), usb_audio_initialized(false) {}

HostBox3Component::~HostBox3Component() {
  if (client_hdl) usb_host_client_deregister(client_hdl);
  if (phy_hdl) usb_del_phy(phy_hdl);
  usb_host_uninstall();
  if (usb_task_handle) vTaskDelete(usb_task_handle);
}

void HostBox3Component::setup() {
  ESP_LOGCONFIG(TAG, "Initializing ESP32-S3-BOX3 USB Audio Host...");
  init_usb_audio();
}

void HostBox3Component::loop() {
  if (!usb_audio_initialized) {
    // Tentative d'initialisation
    if (route_audio_to_usb()) {
      ESP_LOGI(TAG, "USB Audio successfully initialized");
      usb_audio_initialized = true;
    }
  }
}

void HostBox3Component::init_usb_audio() {
  const usb_phy_config_t phy_config = {
    .controller = USB_PHY_CTRL_OTG,
    .otg_mode = USB_OTG_MODE_HOST,
    .target = USB_PHY_TARGET_INT,
    .gpio_conf = {
      .d_n = 19,  // GPIO19 pour D-
      .d_p = 20,  // GPIO20 pour D+
      .vp = -1,
      .vm = -1,
    }
  };

  ESP_ERROR_CHECK(usb_new_phy(&phy_config, &phy_hdl));
  
  const usb_host_config_t host_config = {
    .skip_phy_setup = false,
    .intr_flags = ESP_INTR_FLAG_LEVEL3
  };
  
  ESP_ERROR_CHECK(usb_host_install(&host_config));
  
  const usb_host_client_config_t client_config = {
    .is_synchronous = false,
    .max_num_event_msg = 5
  };
  
  ESP_ERROR_CHECK(usb_host_client_register(&client_config, &client_hdl));
  xTaskCreate(usb_event_task, "usb_events", 4096, this, 5, &usb_task_handle);
}

bool HostBox3Component::route_audio_to_usb() {
  // Ici, vous pouvez utiliser le composant media_player pour router l'audio
  // via une API d'ESPHome, mais cela nécessite une implémentation spécifique
  // pour gérer le flux audio vers un périphérique USB.
  return true; // Temporairement, jusqu'à implémentation complète
}

void HostBox3Component::usb_event_task(void *arg) {
  HostBox3Component *self = static_cast<HostBox3Component *>(arg);
  usb_device_info_t dev_info;
  
  while (1) {
    if (usb_host_get_device_info(self->client_hdl, &dev_info) == ESP_OK) {
      if (dev_info.connected && dev_info.class_type == USB_CLASS_AUDIO) {
        // Gérer l'événement de connexion du périphérique audio
        ESP_LOGI(TAG, "USB Audio device connected");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

}  // namespace host_box3
}  // namespace esphome


