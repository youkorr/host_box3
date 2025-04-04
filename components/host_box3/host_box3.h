#pragma once

#include "esphome/core/component.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"

namespace esphome {
namespace host_box3 {

// Définition des broches USB pour ESP32-S3 Box 3
static const gpio_num_t USB_DP_PIN = GPIO_NUM_21; // D+
static const gpio_num_t USB_DM_PIN = GPIO_NUM_42; // D-

// Définitions pour les descripteurs USB
#define USB_CLASS_AUDIO 0x01
#define USB_SUBCLASS_AUDIOCONTROL 0x01
#define USB_SUBCLASS_AUDIOSTREAMING 0x02

// Définitions UAC (USB Audio Class)
#define UAC_FORMAT_TYPE_I 0x01
#define UAC_EP_GENERAL 0x01

// Définitions pour les endpoints USB
#define USB_B_ENDPOINT_ADDRESS_EP_DIR_OUT 0x00
#define USB_B_ENDPOINT_ADDRESS_EP_DIR_IN 0x80

// Structure pour le buffer audio
typedef struct {
    uint8_t *buffer;
    size_t size;
    SemaphoreHandle_t buffer_semaphore;
    QueueHandle_t data_queue;
} audio_buffer_t;

class HostBox3Component : public Component {
public:
    HostBox3Component();
    ~HostBox3Component();

    void setup() override;
    void loop() override;
    void dump_config() override;

    // Enregistrement du callback audio
    void register_audio_callback(void (*callback)(void*, void*, size_t), void* arg);

private:
    bool usb_audio_initialized;
    TaskHandle_t usb_task_handle;
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t audio_dev_handle;
    usb_transfer_t *audio_transfer;

    uint8_t usb_endpoint_addr;
    uint8_t usb_interface_num;
    uint8_t usb_alt_setting;

    audio_buffer_t audio_buffer;

    void (*audio_callback)(void*, void*, size_t);
    void *audio_callback_arg;

    void init_usb_audio();
    void configure_audio_device(usb_device_handle_t dev_hdl);
    void handle_usb_events(const usb_host_client_event_msg_t *event_msg);
    static void client_event_callback_static(const usb_host_client_event_msg_t *event_msg, void *arg);
    void client_event_callback(const usb_host_client_event_msg_t *event_msg);

    static void usb_task_function(void *arg);
};

}  // namespace host_box3
}  // namespace esphome











