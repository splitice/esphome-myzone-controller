import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch, uart
from esphome.const import CONF_ID, CONF_NAME, CONF_UART_ID, ICON_FAN
from esphome import pins

AUTO_LOAD = ["switch"]
DEPENDENCIES = ["uart"]

CONF_RSE_PIN = "rse_pin"
CONF_ZONE_1 = "zone_1"
CONF_ZONE_2 = "zone_2"
CONF_ZONE_3 = "zone_3"
CONF_ZONE_4 = "zone_4"
CONF_ZONE_5 = "zone_5"

myzone_ns = cg.esphome_ns.namespace("myzone")
MyZoneController = myzone_ns.class_("MyZoneController", cg.Component, uart.UARTDevice)
MyZoneSwitch = myzone_ns.class_("MyZoneSwitch", switch.Switch)

ZONE_CONFS = [CONF_ZONE_1, CONF_ZONE_2, CONF_ZONE_3, CONF_ZONE_4, CONF_ZONE_5]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MyZoneController),
        cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_RSE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_ZONE_1, default={CONF_NAME: "Zone 1"}): switch.switch_schema(
            MyZoneSwitch, icon=ICON_FAN
        ),
        cv.Optional(CONF_ZONE_2, default={CONF_NAME: "Zone 2"}): switch.switch_schema(
            MyZoneSwitch, icon=ICON_FAN
        ),
        cv.Optional(CONF_ZONE_3, default={CONF_NAME: "Zone 3"}): switch.switch_schema(
            MyZoneSwitch, icon=ICON_FAN
        ),
        cv.Optional(CONF_ZONE_4, default={CONF_NAME: "Zone 4"}): switch.switch_schema(
            MyZoneSwitch, icon=ICON_FAN
        ),
        cv.Optional(CONF_ZONE_5, default={CONF_NAME: "Zone 5"}): switch.switch_schema(
            MyZoneSwitch, icon=ICON_FAN
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "myzone",
    baud_rate=250000,
    data_bits=8,
    parity="ODD",
    stop_bits=2,
    require_rx=True,
    require_tx=True,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_RSE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RSE_PIN])
        cg.add(var.set_rse_pin(pin))

    for zone_index, zone_conf in enumerate(ZONE_CONFS):
        zone = config[zone_conf]
        sw = cg.new_Pvariable(zone[CONF_ID], zone_index)
        await switch.register_switch(sw, zone)
        cg.add(sw.set_parent(var))
        cg.add(var.set_zone_switch(zone_index, sw))
