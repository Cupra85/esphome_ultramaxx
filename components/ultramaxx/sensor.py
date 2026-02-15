import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")

UltraMaXXComponent = ultramaxx_ns.class_(
    "UltraMaXXComponent",
    cg.Component,
    uart.UARTDevice,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UltraMaXXComponent),
        cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config["uart_id"])
    var = cg.new_Pvariable(config[CONF_ID], parent)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
