import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")
UltraMaXXComponent = ultramaxx_ns.class_(
    "UltraMaXXComponent", cg.PollingComponent, uart.UARTDevice
)

CONF_SERIAL_NUMBER = "serial_number"
CONF_TOTAL_ENERGY = "total_energy"
CONF_TOTAL_VOLUME = "total_volume"
CONF_CURRENT_POWER = "current_power"
CONF_TEMP_FLOW = "temp_flow"
CONF_TEMP_RETURN = "temp_return"
CONF_TEMP_DIFF = "temp_diff"
CONF_METER_TIME = "meter_time"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UltraMaXXComponent),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

            cv.Optional(CONF_SERIAL_NUMBER): sensor.sensor_schema(),
            cv.Optional(CONF_TOTAL_ENERGY): sensor.sensor_schema(unit_of_measurement="kWh"),
            cv.Optional(CONF_TOTAL_VOLUME): sensor.sensor_schema(unit_of_measurement="m³"),
            cv.Optional(CONF_CURRENT_POWER): sensor.sensor_schema(unit_of_measurement="W"),
            cv.Optional(CONF_TEMP_FLOW): sensor.sensor_schema(unit_of_measurement="°C"),
            cv.Optional(CONF_TEMP_RETURN): sensor.sensor_schema(unit_of_measurement="°C"),
            cv.Optional(CONF_TEMP_DIFF): sensor.sensor_schema(unit_of_measurement="°C"),
            cv.Optional(CONF_METER_TIME): text_sensor.text_sensor_schema(),
        }
    )
    .extend(cv.polling_component_schema("20s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_SERIAL_NUMBER in config:
        sens = await sensor.new_sensor(config[CONF_SERIAL_NUMBER])
        cg.add(var.set_serial_number_sensor(sens))

    if CONF_TOTAL_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_ENERGY])
        cg.add(var.set_total_energy_sensor(sens))

    if CONF_TOTAL_VOLUME in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_VOLUME])
        cg.add(var.set_total_volume_sensor(sens))

    if CONF_CURRENT_POWER in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT_POWER])
        cg.add(var.set_current_power_sensor(sens))

    if CONF_TEMP_FLOW in config:
        sens = await sensor.new_sensor(config[CONF_TEMP_FLOW])
        cg.add(var.set_temp_flow_sensor(sens))

    if CONF_TEMP_RETURN in config:
        sens = await sensor.new_sensor(config[CONF_TEMP_RETURN])
        cg.add(var.set_temp_return_sensor(sens))

    if CONF_TEMP_DIFF in config:
        sens = await sensor.new_sensor(config[CONF_TEMP_DIFF])
        cg.add(var.set_temp_diff_sensor(sens))

    if CONF_METER_TIME in config:
        ts = await text_sensor.new_text_sensor(config[CONF_METER_TIME])
        cg.add(var.set_meter_time_sensor(ts))
