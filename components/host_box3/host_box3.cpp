#include "host_box3.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"

namespace esphome {
namespace host_box3 {

static const char *TAG = "host_box3";

HostBox3Component::HostBox3Component() : usb_audio_initialized(false), usb_task_handle(nullptr) {}

HostBox3Component::~HostBox3Component() {
    if (this->usb_task_handle) {
        vTaskDelete(this->usb_task_handle);
    }
}

void HostBox3Component::setup() {
    ESP_LOGI(TAG, "Setting up USB Host for ESP32-S3 Box 3...");

    // Configuration des broches USB en mode flottant
    gpio_set_pull_mode(GPIO2, GPIO_FLOATING);  // USB D+ 
    gpio_set_pull_mode(GPIO6, GPIO_FLOATING);  // USB D-
    gpio_set_pull_mode(GPIO19, GPIO_FLOATING); // USB-
    gpio_set_pull_mode(GPIO20, GPIO_FLOATING); // USB+

    // Initialisation de l'USB Host
    this->init_usb_audio();
}

void HostBox3Component::loop() {
    // Ici, on pourrait traiter les événements USB si nécessaire
}

void HostBox3Component::dump_config() {
    ESP_LOGCONFIG(TAG, "USB Host (ESP32-S3 Box 3) Config:");
    ESP_LOGCONFIG(TAG, "USB Audio Initialized: %s", this->usb_audio_initialized ? "Yes" : "No");
}

void HostBox3Component::init_usb_audio() {
    ESP_LOGI(TAG, "Initializing USB Audio...");

    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    if (usb_host_install(&host_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize USB Host.");
        return;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_events = 5,
        .async = {
            .client_event_callback = &HostBox3Component::client_event_callback,
            .callback_arg = this
        }
    };

    if (usb_host_client_register(&client_config, &this->client_hdl) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register USB Host client.");
        return;
    }

    ESP_LOGI(TAG, "USB Audio initialized successfully.");
    this->usb_audio_initialized = true;
}

void HostBox3Component::process_device_connection(uint8_t dev_addr) {
    ESP_LOGI(TAG, "USB device connected at address: %d", dev_addr);
}

void HostBox3Component::process_device_disconnection() {
    ESP_LOGI(TAG, "USB device disconnected.");
}

void HostBox3Component::client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
    auto *self = static_cast<HostBox3Component *>(arg);

    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV: {
            self->process_device_connection(event_msg->new_dev.address);
            break;
        }
        case USB_HOST_CLIENT_EVENT_DEV_GONE: {
            self->process_device_disconnection();
            break;
        }
        default:
            ESP_LOGW(TAG, "Unhandled USB event: %d", event_msg->event);
            break;
    }
}

}  // namespace host_box3
}  // namespace esphome








