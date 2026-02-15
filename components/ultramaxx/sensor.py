import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import (
    CONF_ID, 
    CONF_UART_ID, 
    CONF_SERIAL_NUMBER,
    UNIT_KILOWATT_HOURS,
    UNIT_CUBIC_METER,
    UNIT_KILOWATT,
    UNIT_CELSIUS,
    DEVICE_CLASS_ENERGY,
    STATE_CLASS_TOTAL_INCREASING
)

# Namespace muss exakt zur ultramaxx.h passen
ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")
UltraMaXXComponent = ultramaxx_ns.class_(
    "UltraMaXXComponent", cg.PollingComponent, uart.UARTDevice
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

# ⭐ WICHTIG: Nutze sensor.sensor_schema() Konstanten für HA Energy Dashboard
PLATFORM_SCHEMA = sensor.sensor_schema().extend(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UltraMaXXComponent),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

            cv.Optional(CONF_SERIAL_NUMBER): sensor.sensor_schema(),
            cv.Optional("total_energy"): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT_HOURS,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                accuracy_decimals=3,
            ),
            cv.Optional("total_volume"): sensor.sensor_schema(
                unit_of_measurement=UNIT_CUBIC_METER,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                accuracy_decimals=3,
            ),
            cv.Optional("current_power"): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOWATT,
                accuracy_decimals=3,
            ),
            cv.Optional("temp_flow"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS),
            cv.Optional("temp_return"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS),
            cv.Optional("temp_diff"): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS),
            cv.Optional("meter_time"): text_sensor.text_sensor_schema(),
        }
    ).extend(cv.polling_component_schema("20s")).extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Hilfsfunktion für die Registrierung
    async def add_sensor(key, fn):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(fn(sens))

    await add_sensor(CONF_SERIAL_NUMBER, var.set_serial_number_sensor)
    await add_sensor("total_energy", var.set_total_energy_sensor)
    await add_sensor("total_volume", var.set_total_volume_sensor)
    await add_sensor("current_power", var.set_current_power_sensor)
    await add_sensor("temp_flow", var.set_temp_flow_sensor)
    await add_sensor("temp_return", var.set_temp_return_sensor)
    await add_sensor("temp_diff", var.set_temp_diff_sensor)

    if "meter_time" in config:
        ts = await text_sensor.new_text_sensor(config["meter_time"])
        cg.add(var.set_meter_time_sensor(ts))
