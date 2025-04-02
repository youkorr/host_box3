import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["media_player"]

host_box3_ns = cg.esphome_ns.namespace("host_box3")
HostBox3Component = host_box3_ns.class_("HostBox3Component", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HostBox3Component),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
