#pragma once

#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace usb_host {

class USBHostTextSensor : public text_sensor::TextSensor, public Component {
public:
    void setup() override;
    void dump_config() override;
    void update_status(const std::string &status);
};

}  // namespace usb_host
}  // namespace esphome
