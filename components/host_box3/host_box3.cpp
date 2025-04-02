#include "host_box3.h"
#include "esphome/core/log.h"

static const char *TAG = "host_box3";

namespace esphome {
namespace host_box3 {

HostBox3Component::HostBox3Component() : client_hdl(NULL), uac_handle(NULL), phy_hdl(NULL), usb_audio_initialized(false) {}

HostBox3Component::~HostBox3Component() {
  // Libérer les ressources USB si nécessaire
  if (uac_handle != NULL) {
    uac_host_uninstall(uac_handle);
  }
  
  if (client_hdl != NULL) {
    usb_host_client_deregister(client_hdl);
  }
  
  // Désinstaller l'hôte USB
  usb_host_uninstall();
  
  // Libérer l'interface physique USB
  if (phy_hdl != NULL) {
    usb_del_phy(phy_hdl);
  }
  
  // Supprimer la tâche d'événements USB si elle existe
  if (usb_task_handle != NULL) {
    vTaskDelete(usb_task_handle);
  }
}

void HostBox3Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ESP32-S3-BOX3 USB Audio Host...");
  init_usb_audio();
}

void HostBox3Component::loop() {
  // Vérifier si l'audio USB est disponible
  if (!usb_audio_initialized) {
    // Tentative d'initialisation
    if (route_audio_to_usb()) {
      ESP_LOGI(TAG, "USB Audio successfully initialized");
      usb_audio_initialized = true;
    }
  }
}

void HostBox3Component::dump_config() {
  ESP_LOGCONFIG(TAG, "ESP32-S3-BOX3 USB Audio Host:");
  ESP_LOGCONFIG(TAG, "  USB Audio Initialized: %s", usb_audio_initialized ? "YES" : "NO");
}

void HostBox3Component::init_usb_audio() {
  ESP_LOGI(TAG, "Initializing USB Host for Audio...");
  
  // Configuration des broches USB (GPIO2 pour D+ et GPIO6 pour D-)
  const usb_phy_config_t phy_config = {
    .controller = USB_PHY_CTRL_OTG,
    .otg_mode = USB_OTG_MODE_HOST,
    .target = USB_PHY_TARGET_INT,
    .gpio_conf = {
      .d_n = 6,  // GPIO6 pour D-
      .d_p = 2,  // GPIO2 pour D+
      .vp = -1,
      .vm = -1,
    }
  };
  
  // Initialiser l'interface physique USB
  esp_err_t err = usb_new_phy(&phy_config, &phy_hdl);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB PHY initialization failed: %s", esp_err_to_name(err));
    return;
  }
  
  // Configuration USB Host
  usb_host_config_t host_config = {
    .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };
  
  // Initialisation du host USB
  err = usb_host_install(&host_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB Host installation failed: %s", esp_err_to_name(err));
    return;
  }
  
  // Création du client USB
  usb_host_client_config_t client_config = {
    .is_synchronous = false,
    .max_num_event_msg = 5,
  };
  
  err = usb_host_client_register(&client_config, &client_hdl);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "USB Host client registration failed: %s", esp_err_to_name(err));
    return;
  }
  
  // Création de la tâche pour les événements USB
  xTaskCreate(usb_event_task, "usb_events", 4096, this, 5, &usb_task_handle);
  
  ESP_LOGI(TAG, "USB Host initialized successfully");
}

bool HostBox3Component::route_audio_to_usb() {
  if (uac_handle != NULL) {
    // Déjà initialisé
    return true;
  }
  
  // Configuration du périphérique audio UAC (USB Audio Class)
  uac_host_config_t uac_config = {
    .playback_sampling_rates = {44100, 48000},  // Taux d'échantillonnage supportés
    .playback_ch_num = 2,                      // Stéréo
    .playback_bit_resolution = 16,             // 16 bits par échantillon
  };
  
  esp_err_t err = uac_host_install(&uac_config, &uac_handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "USB Audio Class installation failed: %s", esp_err_to_name(err));
    return false;
  }
  
  // Rerouter la sortie audio vers l'USB
  err = uac_host_set_playback_active(uac_handle, true);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to activate USB audio playback: %s", esp_err_to_name(err));
    return false;
  }
  
  ESP_LOGI(TAG, "Audio routed to USB successfully");
  return true;
}

void HostBox3Component::usb_event_task(void *arg) {
  HostBox3Component *self = static_cast<HostBox3Component *>(arg);
  usb_host_client_handle_t client_hdl = self->client_hdl;
  
  while (1) {
    usb_host_client_event_msg_t event_msg;
    
    // Attendre des événements USB
    esp_err_t err = usb_host_client_handle_events(client_hdl, &event_msg, portMAX_DELAY);
    if (err != ESP_OK) {
      continue;
    }
    
    switch (event_msg.event) {
      case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG, "USB device connected");
        // Tenter d'initialiser l'audio
        self->route_audio_to_usb();
        break;
        
      case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGW(TAG, "USB device disconnected");
        self->usb_audio_initialized = false;
        break;
        
      default:
        break;
    }
  }
}

}  // namespace host_box3
}  // namespace esphome
