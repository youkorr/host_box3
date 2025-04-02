#include "host_box3.h"
#include "esphome/core/log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"
#include "freertos/queue.h"

static const char *TAG = "host_box3";

namespace esphome {
namespace host_box3 {

// Structure pour stocker les informations des périphériques USB
typedef struct {
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint16_t vid;
    uint16_t pid;
} usb_device_info_t;

// Variables globales pour le traitement USB
static QueueHandle_t usb_event_queue;
static usb_device_info_t usb_device_info;

HostBox3Component::HostBox3Component()
    : client_hdl(nullptr), usb_audio_initialized(false), usb_task_handle(nullptr) {
    usb_event_queue = xQueueCreate(10, sizeof(usb_audio_event_t));
}

HostBox3Component::~HostBox3Component() {
    if (client_hdl) {
        usb_host_client_deregister(client_hdl);
    }
    usb_host_uninstall();
    if (usb_task_handle) {
        vTaskDelete(usb_task_handle);
    }
    if (usb_event_queue) {
        vQueueDelete(usb_event_queue);
    }
}

void HostBox3Component::dump_config() {
    ESP_LOGCONFIG(TAG, "HostBox3Component:");
    ESP_LOGCONFIG(TAG, "  USB Host is %s", usb_audio_initialized ? "initialized" : "not initialized");
}

void HostBox3Component::setup() {
    ESP_LOGCONFIG(TAG, "Initializing ESP32-S3-BOX3 USB Audio Host...");
    init_usb_audio();
}

void HostBox3Component::loop() {
    if (!usb_audio_initialized) {
        if (route_audio_to_usb()) {
            ESP_LOGI(TAG, "USB Audio successfully initialized");
            usb_audio_initialized = true;
        }
    }

    usb_audio_event_t event;
    if (xQueueReceive(usb_event_queue, &event, 0) == pdTRUE) {
        process_usb_event(&event);
    }
}

void HostBox3Component::init_usb_audio() {
    ESP_LOGI(TAG, "Initializing USB Host...");

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(GPIO_NUM_6, GPIO_MODE_INPUT_OUTPUT);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB Host installation failed: %s", esp_err_to_name(err));
        return;
    }
    
    const usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_callback,
            .callback_arg = this,
        }
    };
    
    err = usb_host_client_register(&client_config, &client_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "USB Host client registration failed: %s", esp_err_to_name(err));
        usb_host_uninstall();
        return;
    }
    
    xTaskCreate(usb_event_task, "usb_events", 4096, this, 5, &usb_task_handle);
}

void HostBox3Component::client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg) {
    HostBox3Component *self = static_cast<HostBox3Component *>(arg);
    usb_audio_event_t evt;

    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            ESP_LOGI(TAG, "New USB device connected");
            evt.event_id = USB_AUDIO_DEVICE_CONNECTED;
            evt.data = (void *)(uintptr_t)event_msg->new_dev.address;
            xQueueSend(usb_event_queue, &evt, portMAX_DELAY);
            break;

        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            ESP_LOGI(TAG, "USB device disconnected");
            evt.event_id = USB_AUDIO_DEVICE_DISCONNECTED;
            evt.data = NULL;
            xQueueSend(usb_event_queue, &evt, portMAX_DELAY);
            break;

        default:
            break;
    }
}

void HostBox3Component::usb_event_task(void *arg) {
    HostBox3Component *self = static_cast<HostBox3Component *>(arg);
    ESP_LOGI(TAG, "USB Event Task started");
    while (1) {
        usb_host_client_handle_events(self->client_hdl, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

}  // namespace host_box3
}  // namespace esphome



