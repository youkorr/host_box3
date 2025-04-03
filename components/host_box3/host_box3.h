#pragma once

#include "esphome/core/component.h"
#include "usb/usb_host.h"


namespace esphome {
namespace host_box3 {

#define SAMPLE_BUFFER_SIZE 512  // Taille du buffer audio en échantillons
#define USB_XFER_MAX_SIZE 1024  // Taille maximale du transfert USB

typedef struct {
    uint8_t* buffer;
    size_t buffer_size;
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
    
    // Méthode pour enregistrer un callback de flux audio
    void register_audio_callback(void (*callback)(void*, void*, size_t), void* arg);

private:
    void init_usb_audio();
    void configure_audio_device(uint8_t dev_addr);
    void send_audio_to_usb(uint8_t *data, size_t size);
    
    static void usb_task_function(void* arg);
    static void client_event_callback_static(const usb_host_client_event_msg_t *event_msg, void *arg);
    void client_event_callback(const usb_host_client_event_msg_t *event_msg);
    static void usb_transfer_callback(usb_transfer_t *transfer);
    static void i2s_audio_callback(void *arg, void *data, size_t size);

    bool usb_audio_initialized;
    TaskHandle_t usb_task_handle;
    usb_host_client_handle_t client_hdl;
    uint8_t audio_dev_addr;  // Adresse du périphérique audio connecté
    
    audio_buffer_t audio_buffer;
    usb_transfer_t *audio_transfer;
    uint8_t usb_endpoint_addr;
    uint8_t usb_interface_num;
    uint8_t usb_alt_setting;
    
    // Fonction de callback pour recevoir des données audio
    void (*audio_callback)(void*, void*, size_t);
    void *audio_callback_arg;
};

}  // namespace host_box3
}  // namespace esphome








