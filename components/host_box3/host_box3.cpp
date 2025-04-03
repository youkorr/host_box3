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
#define UAC_EP_GENERAL               0x01

// Définitions pour les descripteurs USB
#define USB_INTERFACE_MAX_ALT_SETTINGS 10
#define USB_B_ENDPOINT_ADDRESS_EP_DIR_OUT 0x00
#define USB_B_ENDPOINT_ADDRESS_EP_DIR_IN 0x80

HostBox3Component::HostBox3Component() 
    : usb_audio_initialized(false), 
      usb_task_handle(nullptr), 
      audio_dev_handle(nullptr),
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
        if (audio_dev_handle) {
            usb_host_interface_release(this->client_hdl, this->audio_dev_handle, this->usb_interface_num);
            usb_host_device_close(this->client_hdl, audio_dev_handle);
        }
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
    if (this->audio_dev_handle != nullptr) {
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
    ESP_LOGCONFIG(TAG, "USB Audio Device Connected: %s", this->audio_dev_handle ? "Yes" : "No");
    if (this->audio_dev_handle) {
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
            // We need to open the device before we can get a handle
            usb_device_handle_t dev_hdl;
            esp_err_t err = usb_host_device_open(this->client_hdl, event_msg->new_dev.address, &dev_hdl);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to open USB device: %s", esp_err_to_name(err));
                return;
            }
            
            ESP_LOGI(TAG, "USB device connected");
            
            // Pour ESP-IDF 5.1.5, utilisons les fonctions appropriées
            // Obtenir le descripteur de périphérique
            const usb_device_desc_t *device_desc;
            err = usb_host_get_device_descriptor(dev_hdl, &device_desc);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to get device descriptor: %s", esp_err_to_name(err));
                usb_host_device_close(this->client_hdl, dev_hdl);
                return;
            }
            
            bool is_audio_device = false;
            
            // Vérifier si c'est un périphérique audio
            if (device_desc->bDeviceClass == USB_CLASS_PER_INTERFACE ||
                device_desc->bDeviceClass == USB_CLASS_AUDIO) {
                
                // Obtenir le descripteur de configuration
                const usb_config_desc_t *config_desc;
                err = usb_host_get_active_config_descriptor(dev_hdl, &config_desc);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to get config descriptor: %s", esp_err_to_name(err));
                    usb_host_device_close(this->client_hdl, dev_hdl);
                    return;
                }
                
                // Parcourir les interfaces
                int offset = config_desc->bLength;
                const uint8_t *desc_end = (const uint8_t *)config_desc + config_desc->wTotalLength;
                const uint8_t *curr_desc = (const uint8_t *)config_desc + offset;
                
                while (curr_desc < desc_end) {
                    const usb_standard_desc_t *standard_desc = (const usb_standard_desc_t *)curr_desc;
                    
                    if (standard_desc->bLength == 0) {
                        break;  // Protection contre les descripteurs invalides
                    }
                    
                    if (standard_desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
                        const usb_intf_desc_t *intf_desc = (const usb_intf_desc_t *)standard_desc;
                        
                        // Vérifier si c'est une interface audio streaming
                        if (intf_desc->bInterfaceClass == USB_CLASS_AUDIO && 
                            intf_desc->bInterfaceSubClass == USB_SUBCLASS_AUDIOSTREAMING) {
                            
                            ESP_LOGI(TAG, "Found Audio Streaming interface: %d, Alt: %d", 
                                    intf_desc->bInterfaceNumber, intf_desc->bAlternateSetting);
                            
                            // Chercher les endpoints associés à cette interface
                            const uint8_t *ep_desc_ptr = curr_desc + intf_desc->bLength;
                            while (ep_desc_ptr < desc_end && ep_desc_ptr < curr_desc + 128) {
                                const usb_standard_desc_t *ep_standard_desc = (const usb_standard_desc_t *)ep_desc_ptr;
                                
                                if (ep_standard_desc->bLength == 0) {
                                    break;
                                }
                                
                                if (ep_standard_desc->bDescriptorType == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
                                    const usb_ep_desc_t *ep_desc = (const usb_ep_desc_t *)ep_standard_desc;
                                    
                                    // Chercher un endpoint isochrone OUT
                                    if ((ep_desc->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_ISOC &&
                                        (ep_desc->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) == USB_B_ENDPOINT_ADDRESS_EP_DIR_OUT) {
                                        
                                        ESP_LOGI(TAG, "Found Audio OUT endpoint: 0x%02x, Max Packet: %d",
                                                ep_desc->bEndpointAddress, ep_desc->wMaxPacketSize);
                                        
                                        this->usb_endpoint_addr = ep_desc->bEndpointAddress;
                                        this->usb_interface_num = intf_desc->bInterfaceNumber;
                                        this->usb_alt_setting = intf_desc->bAlternateSetting;
                                        is_audio_device = true;
                                        break;
                                    }
                                }
                                
                                ep_desc_ptr += ep_standard_desc->bLength;
                            }
                        }
                    }
                    
                    curr_desc += standard_desc->bLength;
                }
            }
            
            if (is_audio_device) {
                ESP_LOGI(TAG, "USB Audio device found!");
                this->audio_dev_handle = dev_hdl;
                configure_audio_device(dev_hdl);
            } else {
                // Not an audio device, close it
                usb_host_device_close(this->client_hdl, dev_hdl);
            }
            break;
        }
        
        case USB_HOST_CLIENT_EVENT_DEV_GONE: {
            usb_device_handle_t dev_hdl = event_msg->dev_gone.dev_hdl;
            if (this->audio_dev_handle == dev_hdl) {
                ESP_LOGI(TAG, "USB Audio device disconnected");
                
                // Release interface and close device
                usb_host_interface_release(this->client_hdl, this->audio_dev_handle, this->usb_interface_num);
                usb_host_device_close(this->client_hdl, this->audio_dev_handle);
                this->audio_dev_handle = nullptr;
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

void HostBox3Component::configure_audio_device(usb_device_handle_t dev_hdl) {
    ESP_LOGI(TAG, "Configuring USB Audio device...");
    
    // Utiliser un transfert de contrôle pour configurer le périphérique
    
    // Allouer un transfert de contrôle
    usb_transfer_t *ctrl_xfer;
    esp_err_t err = usb_host_transfer_alloc(64, 0, &ctrl_xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate control transfer: %s", esp_err_to_name(err));
        return;
    }
    
    // Configurer le transfert de contrôle pour SET_CONFIGURATION (1)
    ctrl_xfer->device_handle = dev_hdl;
    ctrl_xfer->bEndpointAddress = 0x00;  // Endpoint de contrôle (EP0)
    ctrl_xfer->callback = NULL;
    ctrl_xfer->context = NULL;
    ctrl_xfer->timeout_ms = 1000;
    ctrl_xfer->num_bytes = 8;  // Taille standard d'un paquet de setup
    
    // Setup packet pour SET_CONFIGURATION
    uint8_t bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | 
                            USB_BM_REQUEST_TYPE_TYPE_STANDARD |
                            USB_BM_REQUEST_TYPE_RECIP_DEVICE;
                            
    ctrl_xfer->data_buffer[0] = bmRequestType;
    ctrl_xfer->data_buffer[1] = USB_B_REQUEST_SET_CONFIGURATION;  // bRequest
    ctrl_xfer->data_buffer[2] = 1;   // wValue LSB (Configuration value = 1)
    ctrl_xfer->data_buffer[3] = 0;   // wValue MSB
    ctrl_xfer->data_buffer[4] = 0;   // wIndex LSB
    ctrl_xfer->data_buffer[5] = 0;   // wIndex MSB
    ctrl_xfer->data_buffer[6] = 0;   // wLength LSB
    ctrl_xfer->data_buffer[7] = 0;   // wLength MSB
    
    // Soumettre le transfert
    err = usb_host_transfer_submit_control(this->client_hdl, ctrl_xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to submit SET_CONFIGURATION control transfer: %s", esp_err_to_name(err));
        usb_host_transfer_free(ctrl_xfer);
        return;
    }
    
    // Attendre que le transfert soit terminé
    bool transfer_completed = false;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (ctrl_xfer->status != 0) {  // Status 0 généralement indique "en cours"
            transfer_completed = true;
            break;
        }
    }
    
    if (!transfer_completed || ctrl_xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGE(TAG, "SET_CONFIGURATION transfer failed or timed out: %d", ctrl_xfer->status);
        usb_host_transfer_free(ctrl_xfer);
        return;
    }
    
    // Maintenant, configurer l'alternate setting pour l'interface audio
    // Réutiliser le même transfert de contrôle
    
    // Setup packet pour SET_INTERFACE
    bmRequestType = USB_BM_REQUEST_TYPE_DIR_OUT | 
                    USB_BM_REQUEST_TYPE_TYPE_STANDARD |
                    USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
                    
    ctrl_xfer->data_buffer[0] = bmRequestType;
    ctrl_xfer->data_buffer[1] = USB_B_REQUEST_SET_INTERFACE;  // bRequest
    ctrl_xfer->data_buffer[2] = this->usb_alt_setting;   // wValue LSB (Alternate Setting)
    ctrl_xfer->data_buffer[3] = 0;   // wValue MSB
    ctrl_xfer->data_buffer[4] = this->usb_interface_num;  // wIndex LSB (Interface Number)
    ctrl_xfer->data_buffer[5] = 0;   // wIndex MSB
    ctrl_xfer->data_buffer[6] = 0;   // wLength LSB
    ctrl_xfer->data_buffer[7] = 0;   // wLength MSB
    
    // Reset du statut du transfert
    ctrl_xfer->status = 0;
    ctrl_xfer->actual_num_bytes = 0;
    
    // Soumettre le transfert
    err = usb_host_transfer_submit_control(this->client_hdl, ctrl_xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to submit SET_INTERFACE control transfer: %s", esp_err_to_name(err));
        usb_host_transfer_free(ctrl_xfer);
        return;
    }
    
    // Attendre que le transfert soit terminé
    transfer_completed = false;
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (ctrl_xfer->status != 0) {
            transfer_completed = true;
            break;
        }
    }
    
    if (!transfer_completed || ctrl_xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGE(TAG, "SET_INTERFACE transfer failed or timed out: %d", ctrl_xfer->status);
        usb_host_transfer_free(ctrl_xfer);
        return;
    }
    
    // Libérer le transfert de contrôle
    usb_host_transfer_free(ctrl_xfer);
    
    // Réclamer l'interface audio
    err = usb_host_interface_claim(this->client_hdl, dev_hdl, this->usb_interface_num, this->usb_alt_setting);
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
    this->audio_transfer->device_handle = dev_hdl;
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
    if (self->audio_dev_handle != nullptr && self->usb_endpoint_addr != 0) {
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
    memcpy(this->audio_buffer.buffer, data, copy_size);
    
    // Si nous avons un transfert USB actif, utilisons-le pour envoyer les données
    if (this->audio_transfer && this->audio_dev_handle) {
        // Copier les données dans le buffer du transfert
        if (copy_size <= USB_XFER_MAX_SIZE) {
            memcpy(this->audio_transfer->data_buffer, data, copy_size);
            this->audio_transfer->num_bytes = copy_size;
            
            // Soumettre le transfert
            esp_err_t err = usb_host_transfer_submit(this->audio_transfer);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to submit USB transfer: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGW(TAG, "Data too large for USB transfer: %d > %d", copy_size, USB_XFER_MAX_SIZE);
        }
    }
    
    // Ajouter les données à la queue pour d'autres traitements si nécessaire
    xQueueSend(this->audio_buffer.data_queue, &copy_size, portMAX_DELAY);
    
    // Relâcher le sémaphore du buffer
    xSemaphoreGive(this->audio_buffer.buffer_semaphore);
    
    ESP_LOGI(TAG, "Audio data sent to USB (size: %d bytes)", copy_size);
}

void HostBox3Component::usb_transfer_callback(usb_transfer_t *transfer) {
    HostBox3Component *self = static_cast<HostBox3Component*>(transfer->context);
    
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGI(TAG, "USB transfer completed successfully, length: %d", transfer->actual_num_bytes);
        
        // Process received data if needed
        if (self->audio_callback) {
            self->audio_callback(self->audio_callback_arg, transfer->data_buffer, transfer->actual_num_bytes);
        }
        
        // Resubmit the transfer for more data if we're reading from the device
        // For OUT transfers (writes), we don't automatically resubmit
        if ((transfer->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) != USB_B_ENDPOINT_ADDRESS_EP_DIR_OUT) {
            usb_host_transfer_submit(transfer);
        }
    } else {
        ESP_LOGW(TAG, "USB transfer error: %d", transfer->status);
    }
}

}  // namespace host_box3
}  // namespace esphome















