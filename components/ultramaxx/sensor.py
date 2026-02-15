import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

AUTO_LOAD = ["uart"]

ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")

UltraMaXXComponent = ultramaxx_ns.class_(
    "UltraMaXXComponent",
    cg.Component,
    uart.UARTDevice,
)

CONF_ULTRAMAXX_ID = "id"

# 👉 WICHTIG: Das registriert den YAML Block "ultramaxx:"
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UltraMaXXComponent),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    uart_parent = await cg.get_variable(config[uart.CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_parent)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
