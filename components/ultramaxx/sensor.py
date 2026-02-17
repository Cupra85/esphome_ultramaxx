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
            cv.Optional(CONF_TOTAL_ENERGY): sensor.sensor_schema(
                unit_of_measurement="MWh", accuracy_decimals=3
            ),
            cv.Optional(CONF_TOTAL_VOLUME): sensor.sensor_schema(
                unit_of_measurement="m³", accuracy_decimals=2
            ),
            cv.Optional(CONF_CURRENT_POWER): sensor.sensor_schema(
                unit_of_measurement="kW", accuracy_decimals=2
            ),
            cv.Optional(CONF_TEMP_FLOW): sensor.sensor_schema(
                unit_of_measurement="°C", accuracy_decimals=1
            ),
            cv.Optional(CONF_TEMP_RETURN): sensor.sensor_schema(
                unit_of_measurement="°C", accuracy_decimals=1
            ),
            cv.Optional(CONF_TEMP_DIFF): sensor.sensor_schema(
                unit_of_measurement="K", accuracy_decimals=2
            ),

            cv.Optional(CONF_METER_TIME): text_sensor.text_sensor_schema(),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    mapping = [
        (CONF_SERIAL_NUMBER, "set_serial_number_sensor", sensor.new_sensor),
        (CONF_TOTAL_ENERGY, "set_total_energy_sensor", sensor.new_sensor),
        (CONF_TOTAL_VOLUME, "set_total_volume_sensor", sensor.new_sensor),
        (CONF_CURRENT_POWER, "set_current_power_sensor", sensor.new_sensor),
        (CONF_TEMP_FLOW, "set_temp_flow_sensor", sensor.new_sensor),
        (CONF_TEMP_RETURN, "set_temp_return_sensor", sensor.new_sensor),
        (CONF_TEMP_DIFF, "set_temp_diff_sensor", sensor.new_sensor),
    ]

    for key, setter, factory in mapping:
        if key in config:
            s = await factory(config[key])
            cg.add(getattr(var, setter)(s))

    if CONF_METER_TIME in config:
        ts = await text_sensor.new_text_sensor(config[CONF_METER_TIME])
        cg.add(var.set_meter_time_sensor(ts))
