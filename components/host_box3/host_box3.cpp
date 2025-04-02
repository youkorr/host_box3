#include "host_box3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace host_box3 {

static const char *TAG = "host_box3";

// Fonction pour router l'audio vers USB
bool HostBox3Component::route_audio_to_usb() {
    // Vérifie si un périphérique audio USB est déjà détecté
    if (!usb_device_info.dev_hdl) {
        ESP_LOGD(TAG, "No USB audio device detected.");
        return false;
    }

    // Logic to route the audio to USB
    ESP_LOGI(TAG, "Routing audio to USB device VID:0x%04X, PID:0x%04X", usb_device_info.vid, usb_device_info.pid);
    
    // Ajoute ici la logique nécessaire pour configurer l'audio vers l'USB en fonction de ton périphérique
    return true;
}

// Fonction pour traiter les événements USB audio
void HostBox3Component::process_usb_event(usb_audio_event_t *event) {
    switch (event->event_id) {
        case USB_AUDIO_DEVICE_CONNECTED:
            ESP_LOGI(TAG, "USB audio device connected.");
            // Traite la connexion du périphérique audio
            break;

        case USB_AUDIO_DEVICE_DISCONNECTED:
            ESP_LOGI(TAG, "USB audio device disconnected.");
            // Traite la déconnexion du périphérique audio
            break;

        case USB_AUDIO_STREAM_STARTED:
            ESP_LOGI(TAG, "USB audio stream started.");
            // Gère le début du flux audio
            break;

        case USB_AUDIO_STREAM_STOPPED:
            ESP_LOGI(TAG, "USB audio stream stopped.");
            // Gère l'arrêt du flux audio
            break;

        default:
            ESP_LOGW(TAG, "Unknown USB audio event.");
            break;
    }
}

}  // namespace host_box3
}  // namespace esphome




