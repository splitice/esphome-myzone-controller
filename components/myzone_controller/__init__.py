import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, switch
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["switch"]

myzone_ns = cg.esphome_ns.namespace("myzone")
MyZoneController = myzone_ns.class_("MyZoneController", cg.Component, uart.UARTDevice)
MyZoneSwitch = myzone_ns.class_("MyZoneSwitch", switch.Switch)

CONF_ZONES = "zones"

ZONE_SCHEMA = switch.SWITCH_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(MyZoneSwitch),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MyZoneController),
            cv.Required(CONF_ZONES): cv.All(
                cv.ensure_list(ZONE_SCHEMA),
                cv.Length(min=1, max=5),
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    for i, zone_config in enumerate(config[CONF_ZONES]):
        zone_var = cg.new_Pvariable(zone_config[CONF_ID], var, i)
        await switch.register_switch(zone_var, zone_config)
        cg.add(var.add_zone_switch(i, zone_var))
