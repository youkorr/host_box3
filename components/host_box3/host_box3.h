#pragma once

#include "esphome/core/component.h"
#include "usb/usb_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

namespace esphome {
namespace host_box3 {

// Structure d'événement USB
typedef struct {
    int event_id;
    void *data;
} usb_audio_event_t;

class HostBox3Component : public Component {
public:
    HostBox3Component();
    ~HostBox3Component();

    void setup() override;
    void loop() override;
    void dump_config() override;
    
    // Définir la priorité du composant
    float get_setup_priority() const override { return setup_priority::LATE; }

private:
    // Initialisation de l'hôte USB audio
    void init_usb_audio();
    
    // Configuration du routage audio vers le périphérique USB
    bool route_audio_to_usb();
    
    // Callback pour les événements client USB
    static void client_event_callback(const usb_host_client_event_msg_t *event_msg, void *arg);
    
    // Traitement des événements USB
    void process_usb_event(usb_audio_event_t *event);
    
    // Gestion de la connexion/déconnexion des périphériques
    void process_device_connection(uint8_t dev_addr);
    void process_device_disconnection();
    
    // Tâche d'événement USB
    static void usb_event_task(void *arg);
    
    // Variables membres
    usb_host_client_handle_t client_hdl;
    bool usb_audio_initialized;
    TaskHandle_t usb_task_handle;
};

}  // namespace host_box3
}  // namespace esphome



