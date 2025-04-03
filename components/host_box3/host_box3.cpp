#include "host_box3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace host_box3 {

static const char *TAG = "host_box3";

HostBox3Component::HostBox3Component() : usb_audio_initialized(false), usb_status_sensor_(nullptr) {}

HostBox3Component::~HostBox3Component() {
    if (usb_task_handle) {
        vTaskDelete(usb_task_handle);
    }
}

void HostBox3Component::setup() {
    ESP_LOGI(TAG, "Configuration du périphérique USB Host");
    
    // Initialisation des broches D+ (GPIO2) et D- (GPIO6) en mode flottant
    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_19, GPIO_FLOATING);
    gpio_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_20, GPIO_FLOATING);

    // Initialisation de l'USB audio
    this->init_usb_audio();

    // Création du capteur de texte si défini
    if (usb_status_sensor_) {
        usb_status_sensor_->publish_state("USB en attente...");
    }
}

void HostBox3Component::loop() {
    if (usb_audio_initialized) {
        route_audio_to_usb();
    }
}

void HostBox3Component::dump_config() {
    ESP_LOGCONFIG(TAG, "HostBox3Component:");
}

void HostBox3Component::init_usb_audio() {
    ESP_LOGI(TAG, "Initialisation de l'USB Audio");

    usb_host_config_t config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&config));

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_callback,
            .callback_arg = this,
        }
    };

    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &client_hdl));

    usb_audio_initialized = true;
}

void HostBox3Component::process_usb_event(usb_audio_event_t *event) {
    switch (event->event_id) {
        case USB_AUDIO_DEVICE_CONNECTED:
            ESP_LOGI(TAG, "Périphérique USB connecté");
            this->update_usb_status("Connecté");
            process_device_connection(reinterpret_cast<uint8_t>(event->data));
            break;

        case USB_AUDIO_DEVICE_DISCONNECTED:
            ESP_LOGI(TAG, "Périphérique USB déconnecté");
            this->update_usb_status("Déconnecté");
            process_device_disconnection();
            break;

        default:
            ESP_LOGW(TAG, "Événement USB inconnu");
            break;
    }
}

void HostBox3Component::process_device_connection(uint8_t dev_addr) {
    ESP_LOGI(TAG, "Traitement de la connexion du périphérique %d", dev_addr);
}

void HostBox3Component::process_device_disconnection() {
    ESP_LOGI(TAG, "Traitement de la déconnexion du périphérique");
}

bool HostBox3Component::route_audio_to_usb() {
    ESP_LOGI(TAG, "Routage du son vers l'USB");
    return true;
}

void HostBox3Component::client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
    auto *component = static_cast<HostBox3Component *>(arg);
    
    usb_audio_event_t event;
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            event.event_id = USB_AUDIO_DEVICE_CONNECTED;
            event.data = (void *)(uintptr_t)event_msg->new_dev.address;
            break;

        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            event.event_id = USB_AUDIO_DEVICE_DISCONNECTED;
            event.data = nullptr;
            break;

        default:
            return;
    }

    component->process_usb_event(&event);
}

void HostBox3Component::update_usb_status(const std::string &status) {
    if (usb_status_sensor_) {
        usb_status_sensor_->publish_state(status);
    }
}

}  // namespace host_box3
}  // namespace esphome





