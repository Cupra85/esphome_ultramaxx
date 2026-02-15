import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import *

DEPENDENCIES = ["uart"]

ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")
UltraMaXXComponent = ultramaxx_ns.class_(
    "UltraMaXXComponent", cg.PollingComponent, uart.UARTDevice
)

CONF_UART_ID = "uart_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UltraMaXXComponent),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

        cv.Optional("serial_number"): sensor.sensor_schema(),
        cv.Optional("total_energy"): sensor.sensor_schema(unit_of_measurement="kWh"),
        cv.Optional("total_volume"): sensor.sensor_schema(unit_of_measurement="m³"),
        cv.Optional("current_power"): sensor.sensor_schema(unit_of_measurement="kW"),
        cv.Optional("temp_flow"): sensor.sensor_schema(unit_of_measurement="°C"),
        cv.Optional("temp_return"): sensor.sensor_schema(unit_of_measurement="°C"),
        cv.Optional("temp_diff"): sensor.sensor_schema(unit_of_measurement="°C"),
        cv.Optional("meter_time"): text_sensor.text_sensor_schema(),
    }
).extend(cv.polling_component_schema("20s"))


async def to_code(config):
    parent = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    def add_sensor(key, fn):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(fn(sens))

    await add_sensor("serial_number", var.set_serial_number_sensor)
    await add_sensor("total_energy", var.set_total_energy_sensor)
    await add_sensor("total_volume", var.set_total_volume_sensor)
    await add_sensor("current_power", var.set_current_power_sensor)
    await add_sensor("temp_flow", var.set_temp_flow_sensor)
    await add_sensor("temp_return", var.set_temp_return_sensor)
    await add_sensor("temp_diff", var.set_temp_diff_sensor)

    if "meter_time" in config:
        ts = await text_sensor.new_text_sensor(config["meter_time"])
        cg.add(var.set_meter_time_sensor(ts))
