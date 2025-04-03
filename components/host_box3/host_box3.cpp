#include "host_box3.h"
#include "esphome/core/log.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "usb/usb_types_stack.h"
#include "usb/usb_types_ch9.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <cstring>

namespace esphome {
namespace host_box3 {

static const char *TAG = "host_box3";

// Définition des broches USB pour ESP32-S3 Box 3
static const gpio_num_t USB_DP_PIN = GPIO_NUM_20;  // D+
static const gpio_num_t USB_DM_PIN = GPIO_NUM_19;  // D-

// Classes USB audio
#define USB_CLASS_AUDIO              0x01
#define USB_SUBCLASS_AUDIOCONTROL    0x01
#define USB_SUBCLASS_AUDIOSTREAMING  0x02

// Définitions UAC (USB Audio Class)
#define UAC_FORMAT_TYPE_I            0x01
#define UAC_EP_GENERAL              0x01

HostBox3Component::HostBox3Component() 
    : usb_audio_initialized(false), 
      usb_task_handle(nullptr), 
      audio_dev_addr(0), 
      audio_transfer(nullptr),
      usb_endpoint_addr(0),
      usb_interface_num(0),
      usb_alt_setting(0),
      audio_callback(nullptr),
      audio_callback_arg(nullptr) {
    memset(&audio_buffer, 0, sizeof(audio_buffer_t));
}

HostBox3Component::~HostBox3Component() {
    if (this->usb_task_handle) {
        vTaskDelete(this->usb_task_handle);
    }
    
    if (audio_buffer.buffer) {
        free(audio_buffer.buffer);
    }
    
    if (audio_buffer.buffer_semaphore) {
        vSemaphoreDelete(audio_buffer.buffer_semaphore);
    }
    
    if (audio_buffer.data_queue) {
        vQueueDelete(audio_buffer.data_queue);
    }
    
    if (audio_transfer) {
        usb_host_transfer_free(audio_transfer);
    }
    
    if (usb_audio_initialized) {
        usb_host_client_deregister(this->client_hdl);
        usb_host_uninstall();
    }
}

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
    
    // Initialiser le buffer audio
    this->audio_buffer.buffer_size = SAMPLE_BUFFER_SIZE * 4; // 16-bit stéréo (2 octets * 2 canaux)
    this->audio_buffer.buffer = (uint8_t*)heap_caps_malloc(this->audio_buffer.buffer_size, MALLOC_CAP_DMA);
    if (!this->audio_buffer.buffer) {
        ESP_LOGE(TAG, "Échec de l'allocation du buffer audio");
        return;
    }
    
    this->audio_buffer.buffer_semaphore = xSemaphoreCreateBinary();
    this->audio_buffer.data_queue = xQueueCreate(4, sizeof(size_t));
    
    // Donner le sémaphore initialement
    xSemaphoreGive(this->audio_buffer.buffer_semaphore);
    
    // Initialisation de l'USB Host et création de la tâche USB
    xTaskCreate(usb_task_function, "usb_task", 4096, this, 5, &this->usb_task_handle);
    
    ESP_LOGI(TAG, "USB Host setup complete");
}

void HostBox3Component::register_audio_callback(void (*callback)(void*, void*, size_t), void* arg) {
    this->audio_callback = callback;
    this->audio_callback_arg = arg;
    ESP_LOGI(TAG, "Audio callback registered");
}

void HostBox3Component::usb_task_function(void* arg) {
    HostBox3Component* self = static_cast<HostBox3Component*>(arg);
    self->init_usb_audio();
    
    while (1) {
        // Traitement des événements USB
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        
        // Vérifier les événements de client
        usb_host_client_handle_events(self->client_hdl, portMAX_DELAY);
        
        // Délai court pour éviter d'utiliser 100% du CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void HostBox3Component::loop() {
    // Dans loop(), nous pouvons vérifier l'état de la connexion USB
    if (this->audio_dev_addr != 0) {
        // Vérifier s'il y a des données en attente dans la queue
        size_t data_size;
        if (xQueueReceive(this->audio_buffer.data_queue, &data_size, 0) == pdTRUE) {
            // Traiter les données si nécessaire
        }
    }
}

void HostBox3Component::dump_config() {
    ESP_LOGCONFIG(TAG, "USB Host (ESP32-S3 Box 3) Config:");
    ESP_LOGCONFIG(TAG, "USB Audio Initialized: %s", this->usb_audio_initialized ? "Yes" : "No");
    ESP_LOGCONFIG(TAG, "USB Audio Device Connected: %s", this->audio_dev_addr ? "Yes" : "No");
    if (this->audio_dev_addr) {
        ESP_LOGCONFIG(TAG, "  Interface: %d, Alt Setting: %d", this->usb_interface_num, this->usb_alt_setting);
        ESP_LOGCONFIG(TAG, "  Endpoint: 0x%02x", this->usb_endpoint_addr);
    }
}

void HostBox3Component::init_usb_audio() {
    ESP_LOGI(TAG, "Initializing USB Audio...");
    
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };
    
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB Host: %s", esp_err_to_name(err));
        return;
    }
    
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .async = {
            .client_event_callback = client_event_callback_static,
            .callback_arg = this
        }
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
    HostBox3Component* self = static_cast<HostBox3Component*>(arg);
    self->client_event_callback(event_msg);
}

void HostBox3Component::client_event_callback(const usb_host_client_event_msg_t *event_msg) {
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV: {
            uint8_t dev_addr = event_msg->new_dev.address;
            ESP_LOGI(TAG, "USB device connected, address: %d", dev_addr);
            
            // Obtenir les descripteurs du périphérique
            usb_device_info_t dev_info;
            if (usb_host_device_info(dev_addr, &dev_info) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get device info");
                return;
            }
            
            bool is_audio_device = false;
            
            // Vérifier si c'est un périphérique audio
            if (dev_info.dev_desc.bDeviceClass == USB_CLASS_PER_INTERFACE ||
                dev_info.dev_desc.bDeviceClass == USB_CLASS_AUDIO) {
                
                // Parcourir toutes les configurations
                const usb_config_desc_t* config_desc = &dev_info.config_desc;
                
                // Parcourir toutes les interfaces
                int offset = 0;
                for (int i = 0; i < config_desc->bNumInterfaces; i++) {
                    // Parcourir les alternate settings
                    for (int alt = 0; alt < USB_INTERFACE_MAX_ALT_SETTINGS; alt++) {
                        const usb_intf_desc_t* intf = usb_parse_interface_descriptor(config_desc, i, alt, &offset);
                        if (!intf) {
                            break; // Pas plus d'alternate settings
                        }
                        
                        // Vérifier si c'est une interface audio streaming
                        if (intf->bInterfaceClass == USB_CLASS_AUDIO && 
                            intf->bInterfaceSubClass == USB_SUBCLASS_AUDIOSTREAMING) {
                            
                            ESP_LOGI(TAG, "Found Audio Streaming interface: %d, Alt: %d", i, alt);
                            
                            // Chercher un endpoint OUT (pour la lecture)
                            for (int ep_idx = 0; ep_idx < intf->bNumEndpoints; ep_idx++) {
                                const usb_ep_desc_t* ep = usb_parse_endpoint_descriptor_by_index(config_desc, i, alt, ep_idx, &offset);
                                if (!ep) {
                                    continue;
                                }
                                
                                // Chercher un endpoint isochrone OUT
                                if ((ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_ISOC &&
                                    (ep->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) == USB_B_ENDPOINT_ADDRESS_EP_DIR_OUT) {
                                    
                                    ESP_LOGI(TAG, "Found Audio OUT endpoint: 0x%02x, Max Packet: %d",
                                             ep->bEndpointAddress, ep->wMaxPacketSize);
                                    
                                    this->usb_endpoint_addr = ep->bEndpointAddress;
                                    this->usb_interface_num = i;
                                    this->usb_alt_setting = alt;
                                    is_audio_device = true;
                                }
                            }
                        }
                    }
                }
            }
            
            if (is_audio_device) {
                ESP_LOGI(TAG, "USB Audio device found!");
                this->audio_dev_addr = dev_addr;
                configure_audio_device(dev_addr);
            }
            break;
        }
        
        case USB_HOST_CLIENT_EVENT_DEV_GONE: {
            uint8_t dev_addr = event_msg->dev_gone.dev_hdl;
            if (this->audio_dev_addr == dev_addr) {
                ESP_LOGI(TAG, "USB Audio device disconnected");
                this->audio_dev_addr = 0;
                this->usb_endpoint_addr = 0;
                
                // Libérer les ressources
                if (this->audio_transfer) {
                    usb_host_transfer_free(this->audio_transfer);
                    this->audio_transfer = nullptr;
                }
            }
            break;
        }
        
        default:
            break;
    }
}

void HostBox3Component::configure_audio_device(uint8_t dev_addr) {
    ESP_LOGI(TAG, "Configuring USB Audio device...");
    
    // Set Configuration (normalement configuration #1)
    esp_err_t err = usb_host_device_set_configuration(dev_addr, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set configuration: %s", esp_err_to_name(err));
        return;
    }
    
    // Réclamer l'interface audio
    err = usb_host_interface_claim(this->client_hdl, dev_addr, this->usb_interface_num, this->usb_alt_setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to claim interface: %s", esp_err_to_name(err));
        return;
    }
    
    // Allouer un transfert USB pour l'audio
    err = usb_host_transfer_alloc(USB_XFER_MAX_SIZE, 0, &this->audio_transfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate transfer: %s", esp_err_to_name(err));
        return;
    }
    
    // Configurer le transfert
    this->audio_transfer->device_handle = dev_addr;
    this->audio_transfer->bEndpointAddress = this->usb_endpoint_addr;
    this->audio_transfer->callback = usb_transfer_callback;
    this->audio_transfer->context = this;
    this->audio_transfer->timeout_ms = 0;  // Pas de timeout pour les transferts isochrones
    
    ESP_LOGI(TAG, "USB Audio device configured successfully");
    ESP_LOGI(TAG, "Ready to stream audio to USB endpoint 0x%02x", this->usb_endpoint_addr);
}

void HostBox3Component::i2s_audio_callback(void *arg, void *data, size_t size) {
    HostBox3Component *self = static_cast<HostBox3Component*>(arg);
    
    // Si nous avons un périphérique audio connecté, envoyer les données via USB
    if (self->audio_dev_addr != 0 && self->usb_endpoint_addr != 0) {
        self->send_audio_to_usb((uint8_t*)data, size);
    }
}

void HostBox3Component::send_audio_to_usb(uint8_t *data, size_t size) {
    // Vérifier si nous pouvons prendre le sémaphore du buffer
    if (xSemaphoreTake(this->audio_buffer.buffer_semaphore, 0) != pdTRUE) {
        // Le buffer est occupé, ignorer ces données
        return;
    }
    
    // S'assurer que la taille ne dépasse pas la capacité du buffer
    size_t copy_size = (size > this->audio_buffer.buffer_size) ? this->audio_buffer.buffer_size : size;
    
    // Copier les données dans le buffer
    memcpy(this->audio_buffer.buffer, data, copy_size);
    
    // Vérifier si le transfert USB est disponible
    if (!this->audio_transfer || this->audio_dev_addr == 0 || this->usb_endpoint_addr == 0) {
        xSemaphoreGive(this->audio_buffer.buffer_semaphore);
        return;
    }
    
    // Configurer le transfert USB
    this->audio_transfer->data_buffer = this->audio_buffer.buffer;
    this->audio_transfer->num_bytes = copy_size;
    
    // Soumettre le transfert USB
    esp_err_t err = usb_host_transfer_submit(this->audio_transfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to submit USB transfer: %s", esp_err_to_name(err));
        xSemaphoreGive(this->audio_buffer.buffer_semaphore);
    }
    // Le sémaphore sera rendu dans le callback de transfert
}

void HostBox3Component::usb_transfer_callback(usb_transfer_t *transfer) {
    HostBox3Component *self = static_cast<HostBox3Component*>(transfer->context);
    
    // Libérer le buffer pour le prochain transfert
    xSemaphoreGive(self->audio_buffer.buffer_semaphore);
    
    // Vérifier le statut du transfert
    if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGW(TAG, "USB transfer error: %d", transfer->status);
        return;
    }
    
    // Ajouter à la queue pour traitement ultérieur si nécessaire
    size_t size = transfer->actual_num_bytes;
    xQueueSend(self->audio_buffer.data_queue, &size, 0);
}

}  // namespace host_box3
}  // namespace esphome









