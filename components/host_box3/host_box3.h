#ifndef HOST_BOX3_H
#define HOST_BOX3_H

#include "esphome/core/component.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "usb/uac_host.h"

namespace esphome {
namespace host_box3 {

static const gpio_num_t USB_DP_PIN = GPIO_NUM_20; // D+
static const gpio_num_t USB_DM_PIN = GPIO_NUM_19; // D-

class HostBox3Component : public Component {
public:
    void setup() override;
    void loop() override;
    void dump_config() override;

private:
    bool usb_audio_initialized = false;
    TaskHandle_t usb_task_handle;
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t audio_dev_handle;

    static void client_event_callback_static(const usb_host_client_event_msg_t *event_msg, void *arg);
    void client_event_callback(const usb_host_client_event_msg_t *event_msg);

    void init_usb_audio();
};

}  // namespace host_box3
}  // namespace esphome

#endif  // HOST_BOX3_H











