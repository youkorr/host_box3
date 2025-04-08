#include "host_box3.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "usb/uac_host.h"
#include "bsp_board.h"

namespace esphome {
namespace host_box3 {

static const char *TAG = "host_box3";

void HostBox3Component::setup() {
    ESP_LOGI(TAG, "Setting up USB Host...");

    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << USB_DP_PIN) | (1ULL << USB_DM_PIN);
    gpio_config(&io_conf);

    init_usb_audio();
}

void HostBox3Component::init_usb_audio() {
    usb_host_config_t host_config = {};
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB Host: %s", esp_err_to_name(err));
        return;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .async = {
            .client_event_callback = HostBox3Component::client_event_callback_static,
            .callback_arg = this,
        },
    };

    err = usb_host_client_register(&client_config, &this->client_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register USB Host client: %s", esp_err_to_name(err));
        usb_host_uninstall();
        return;
    }

    ESP_LOGI(TAG, "USB Host initialized successfully");
    this->usb_audio_initialized = true;
}

void HostBox3Component::client_event_callback_static(const usb_host_client_event_msg_t *event_msg, void *arg) {
    static_cast<HostBox3Component *>(arg)->client_event_callback(event_msg);
}

void HostBox3Component::client_event_callback(const usb_host_client_event_msg_t *event_msg) {
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV: {
            usb_device_handle_t dev_hdl;
            esp_err_t err = usb_host_device_open(this->client_hdl, event_msg->new_dev.address, &dev_hdl);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to open USB device: %s", esp_err_to_name(err));
                return;
            }
            ESP_LOGI(TAG, "New USB device connected");
            break;
        }
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "USB device disconnected");
            break;
        default:
            break;
    }
}

}  // namespace host_box3
}  // namespace esphome















