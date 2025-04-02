#include "host_box3.h"
#include "esphome/core/log.h"

static const char *TAG = "host_box3";

namespace esphome {
namespace host_box3 {

HostBox3Component::HostBox3Component() 
  : audio_dev(nullptr), client_hdl(nullptr), phy_hdl(nullptr), usb_audio_initialized(false) {}

HostBox3Component::~HostBox3Component() {
  if (audio_dev) esp_audio_destroy(audio_dev);
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
  static uint32_t last_check = 0;
  if (millis() - last_check > 1000) {
    if (!usb_audio_initialized) {
      usb_audio_initialized = route_audio_to_usb();
    }
    last_check = millis();
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
  if (audio_dev) return true;

  esp_audio_cfg_t cfg = {
    .audio_codec = ESP_AUDIO_CODEC_DECODER,
    .i2s_port = -1, // Utilisation USB
    .vol_handle = NULL,
    .evt_que = nullptr,
    .cb_func = audio_event_handler,
    .user_ctx = this
  };

  if (esp_audio_create(&cfg, &audio_dev) != ESP_OK) {
    ESP_LOGE(TAG, "Audio pipeline creation failed");
    return false;
  }

  esp_audio_codec_config_t codec_cfg = {
    .type = ESP_AUDIO_CODEC_TYPE_DECODER,
    .dec_type = ESP_AUDIO_CODEC_DECODER_USB
  };

  return esp_audio_codec_set(audio_dev, &codec_cfg) == ESP_OK;
}

void HostBox3Component::usb_event_task(void *arg) {
  HostBox3Component *self = static_cast<HostBox3Component *>(arg);
  usb_device_info_t dev_info;
  
  while (1) {
    if (usb_host_get_device_info(self->client_hdl, &dev_info) == ESP_OK) {
      if (dev_info.connected && dev_info.class_type == USB_CLASS_AUDIO) {
        self->route_audio_to_usb();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void HostBox3Component::audio_event_handler(esp_audio_handle_t audio, 
                                         esp_audio_event_t event,
                                         void *user_ctx) {
  HostBox3Component *self = static_cast<HostBox3Component *>(user_ctx);
  
  switch (event.event_type) {
    case ESP_AUDIO_EVENT_PLAYBACK_STATUS:
      ESP_LOGI(TAG, "Playback status: %d", event.data.status);
      break;
      
    case ESP_AUDIO_EVENT_USB_PLUG_IN:
      self->usb_audio_initialized = true;
      ESP_LOGI(TAG, "USB Audio device connected");
      break;
      
    case ESP_AUDIO_EVENT_USB_PLUG_OUT:
      self->usb_audio_initialized = false;
      ESP_LOGW(TAG, "USB Audio device disconnected");
      break;
      
    default:
      break;
  }
}

}  // namespace host_box3
}  // namespace esphome

