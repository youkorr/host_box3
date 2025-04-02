#pragma once

#include "esphome/core/component.h"
#include "usb/usb_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

namespace esphome {
namespace host_box3 {

// Définir les types d'événements USB audio
typedef enum {
    USB_AUDIO_DEVICE_CONNECTED,
    USB_AUDIO_DEVICE_DISCONNECTED,
    USB_AUDIO_STREAM_STARTED,
    USB_AUDIO_STREAM_STOPPED
} usb_audio_event_id_t;

// Structure pour stocker les événements USB audio
typedef struct {
    usb_audio_event_id_t event_id;
    void *data;
} usb_audio_event_t;

class HostBox3Component : public Component {
public:
    HostBox3Component();
    ~HostBox3Component();
    
    void setup() override;
    void loop() override;
    void dump_config() override;
    
private:
    usb_host_client_handle_t client_hdl;
    bool usb_audio_initialized;
    TaskHandle_t usb_task_handle;
    
    void init_usb_audio();
    bool route_audio_to_usb();
    void process_usb_event(usb_audio_event_t *event);
    void process_device_connection(uint8_t dev_addr);
    void process_device_disconnection();
    
    static void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg);
    static void usb_event_task(void *arg);
};

}  // namespace host_box3
}  // namespace esphome



