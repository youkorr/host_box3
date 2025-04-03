#include "host_box3.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"

namespace esphome {
namespace host_box3 {

static const char *TAG = "host_box3";

HostBox3Component::HostBox3Component() : usb_audio_initialized(false), usb_task_handle(nullptr), event_queue(nullptr) {}

HostBox3Component::~HostBox3Component() {
    if (event_queue != nullptr) {
        vQueueDelete(event_queue);
    }
}

// Configuration initiale
void HostBox3Component::setup() {
    ESP_LOGI(TAG, "Configuration du HostBox3...");
    
    this->setup_gpio_for_usb();
    this->init_usb_audio();

    event_queue = xQueueCreate(10, sizeof(usb_audio_event_t));
    if (event_queue == nullptr) {
        ESP_LOGE(TAG, "Échec de la création de la file d'attente des événements USB");
    }
}

// Configuration des broches GPIO2 (D+) et GPIO6 (D-) en mode flottant
void HostBox3Component::setup_gpio_for_usb() {
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_19, GPIO_FLOATING);

    gpio_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_20, GPIO_FLOATING);

    ESP_LOGI(TAG, "GPIO19 (D+) et GPIO20 (D-) configurés en mode flottant.");
}

// Initialisation de l'USB audio
void HostBox3Component::init_usb_audio() {
    ESP_LOGI(TAG, "Initialisation de l'USB audio...");

    usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    if (usb_host_install(&host_config) == ESP_OK) {
        ESP_LOGI(TAG, "USB Host installé avec succès.");
    } else {
        ESP_LOGE(TAG, "Échec de l'installation de l'USB Host.");
    }
}

// Fonction appelée dans la boucle principale
void HostBox3Component::loop() {
    usb_audio_event_t event;
    if (xQueueReceive(event_queue, &event, 0)) {
        this->process_usb_event(&event);
    }
}

// Traitement des événements USB
void HostBox3Component::process_usb_event(usb_audio_event_t *event) {
    switch (event->event_id) {
        case USB_AUDIO_DEVICE_CONNECTED:
            ESP_LOGI(TAG, "Périphérique USB audio connecté.");
            this->process_device_connection((uint8_t)(uintptr_t)event->data);
            break;

        case USB_AUDIO_DEVICE_DISCONNECTED:
            ESP_LOGI(TAG, "Périphérique USB audio déconnecté.");
            this->process_device_disconnection();
            break;

        default:
            ESP_LOGW(TAG, "Événement USB non reconnu.");
            break;
    }
}

// Gestion de la connexion d'un périphérique USB audio
void HostBox3Component::process_device_connection(uint8_t dev_addr) {
    ESP_LOGI(TAG, "Connexion USB Audio détectée : Adresse %d", dev_addr);
    usb_audio_initialized = true;
}

// Gestion de la déconnexion d'un périphérique USB audio
void HostBox3Component::process_device_disconnection() {
    ESP_LOGI(TAG, "Déconnexion du périphérique USB Audio.");
    usb_audio_initialized = false;
}

// Callback pour les événements clients USB
void HostBox3Component::client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
    auto *component = static_cast<HostBox3Component *>(arg);
    
    usb_audio_event_t event;
    
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "Nouveau périphérique USB détecté.");
            event.event_id = USB_AUDIO_DEVICE_CONNECTED;
            event.data = (void *)(uintptr_t)event_msg->new_dev.address;
            break;

        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "Périphérique USB retiré.");
            event.event_id = USB_AUDIO_DEVICE_DISCONNECTED;
            event.data = nullptr;
            break;

        default:
            ESP_LOGW(TAG, "Événement USB inconnu reçu.");
            return;
    }

    if (component->event_queue != nullptr) {
        xQueueSend(component->event_queue, &event, 0);
    }
}

// Fonction pour rediriger l'audio vers l'USB (si nécessaire)
bool HostBox3Component::route_audio_to_usb() {
    if (!usb_audio_initialized) {
        ESP_LOGW(TAG, "Aucun périphérique USB Audio connecté.");
        return false;
    }

    ESP_LOGI(TAG, "Redirection de l'audio vers USB.");
    return true;
}

// Affichage de la configuration dans les logs
void HostBox3Component::dump_config() {
    ESP_LOGI(TAG, "HostBox3Component configuré.");
}

}  // namespace host_box3
}  // namespace esphome





