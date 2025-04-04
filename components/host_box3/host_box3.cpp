#include "host_box3.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "usb/usb_host.h"

namespace esphome {
namespace host_box3 {

static const char *TAG = "host_box3";

// Broches USB pour ESP32-S3 Box 3
static const gpio_num_t USB_DP_PIN = GPIO_NUM_21; // D+
static const gpio_num_t USB_DM_PIN = GPIO_NUM_42; // D-

class HostBox3Component : public Component {
public:
    void setup() override;
    void loop() override;
    void dump_config() override;

private:
    bool usb_audio_initialized = false;
    usb_host_client_handle_t client_hdl = nullptr;
    usb_device_handle_t audio_dev_handle = nullptr;

    void init_usb_audio();
    void handle_usb_events(const usb_host_client_event_msg_t *event_msg);
};

void HostBox3Component::setup() {
    ESP_LOGI(TAG, "Setting up USB Host for ESP32-S3 Box 3...");

    // Configuration des broches USB Host
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << USB_DP_PIN) | (1ULL << USB_DM_PIN);
    gpio_config(&io_conf);

    // Initialiser le pilote USB Host
    init_usb_audio();
}

void HostBox3Component::init_usb_audio() {
    ESP_LOGI(TAG, "Initializing USB Audio...");

    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB Host: %s", esp_err_to_name(err));
        return;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .async = {
            .client_event_callback = [](usb_host_client_handle_t client_hdl, const usb_host_client_event_msg_t *event_msg, void *arg) {
                static_cast<HostBox3Component *>(arg)->handle_usb_events(event_msg);
            },
            .callback_arg = this,
        },
    };

    err = usb_host_client_register(&client_config, &client_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register USB Host client: %s", esp_err_to_name(err));
        usb_host_uninstall();
        return;
    }

    ESP_LOGI(TAG, "USB Host initialized successfully");
    usb_audio_initialized = true;
}

void HostBox3Component::handle_usb_events(const usb_host_client_event_msg_t *event_msg) {
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV: {
            usb_device_handle_t dev_hdl = event_msg->new_dev.dev_hdl;
            ESP_LOGI(TAG, "New USB device connected");

            // Vérifier si c'est un périphérique audio
            const usb_device_desc_t *device_desc;
            esp_err_t err = usb_host_get_device_descriptor(dev_hdl, &device_desc);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get device descriptor: %s", esp_err_to_name(err));
                usb_host_device_close(client_hdl, dev_hdl);
                return;
            }

            bool is_audio_device = false;
            // Parcourir les interfaces pour détecter un périphérique audio
            // Implémentez ici la logique pour vérifier les classes/interfaces audio

            if (is_audio_device) {
                ESP_LOGI(TAG, "USB Audio device found!");
                audio_dev_handle = dev_hdl;
            } else {
                usb_host_device_close(client_hdl, dev_hdl);
            }
            break;
        }

        case USB_HOST_CLIENT_EVENT_DEV_GONE: {
            usb_device_handle_t dev_hdl = event_msg->dev_gone.dev_hdl;
            if (audio_dev_handle == dev_hdl) {
                ESP_LOGI(TAG, "USB Audio device disconnected");
                usb_host_device_close(client_hdl, audio_dev_handle);
                audio_dev_handle = nullptr;
            }
            break;
        }

        default:
            break;
    }
}

void HostBox3Component::loop() {
    if (usb_audio_initialized) {
        usb_host_lib_handle_events(portMAX_DELAY, NULL);
    }
}

void HostBox3Component::dump_config() {
    ESP_LOGCONFIG(TAG, "USB Host Config:");
    ESP_LOGCONFIG(TAG, "USB Audio Initialized: %s", usb_audio_initialized ? "Yes" : "No");
    ESP_LOGCONFIG(TAG, "USB Audio Device Connected: %s", audio_dev_handle ? "Yes" : "No");
}

}  // namespace host_box3
}  // namespace esphome















