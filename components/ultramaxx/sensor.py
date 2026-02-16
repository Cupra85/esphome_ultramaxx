import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import (
    CONF_ID, 
    CONF_UART_ID, 
    CONF_UPDATE_INTERVAL,
    CONF_SERIAL_NUMBER,
    DEVICE_CLASS_ENERGY,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_KILOWATT_HOURS,
    UNIT_CUBIC_METER,
    UNIT_KILOWATT,
    UNIT_CELSIUS
)

# WICHTIG: Diese Abhängigkeiten sagen ESPHome, welche Module geladen sein müssen
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

# Namespace Definition (muss exakt zur ultramaxx.h passen)
ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")
UltraMaXXComponent = ultramaxx_ns.class_(
    "UltraMaXXComponent", cg.PollingComponent, uart.UARTDevice
)

# Definieren der Sensor-Keys
CONF_TOTAL_ENERGY = "total_energy"
CONF_TOTAL_VOLUME = "total_volume"
CONF_CURRENT_POWER = "current_power"
CONF_TEMP_FLOW = "temp_flow"
CONF_TEMP_RETURN = "temp_return"
CONF_TEMP_DIFF = "temp_diff"
CONF_METER_TIME = "meter_time"

# ⭐ DAS PLATFORM_SCHEMA
# Es muss PLATFORM_SCHEMA heißen, damit es unter "sensor:" in der YAML funktioniert.
PLATFORM_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UltraMaXXComponent),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

        cv.Optional(CONF_SERIAL_NUMBER): sensor.sensor_schema(
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_TOTAL_ENERGY): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            accuracy_decimals=3,
        ),
        cv.Optional(CONF_TOTAL_VOLUME): sensor.sensor_schema(
            unit_of_measurement=UNIT_CUBIC_METER,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            accuracy_decimals=3,
        ),
        cv.Optional(CONF_CURRENT_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT,
            accuracy_decimals=3,
        ),
        cv.Optional(CONF_TEMP_FLOW): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_TEMP_RETURN): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_TEMP_DIFF): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=2,
        ),
        cv.Optional(CONF_METER_TIME): text_sensor.text_sensor_schema(),
    }
).extend(cv.polling_component_schema("20s")).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    # UART parent abrufen
    parent = await cg.get_variable(config[CONF_UART_ID])
    # Neue Instanz der C++ Klasse erstellen
    var = cg.new_Pvariable(config[CONF_ID], parent)
    
    # Registrierung der Basiskomponenten (Polling & UART)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Hilfsfunktion zum Registrieren der Sensoren
    async def add_sensor(key, setter_fn):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(setter_fn(sens))

    # Sensoren verknüpfen (ruft die set_... Methoden in ultramaxx.h auf)
    await add_sensor(CONF_SERIAL_NUMBER, var.set_serial_number_sensor)
    await add_sensor(CONF_TOTAL_ENERGY, var.set_total_energy_sensor)
    await add_sensor(CONF_TOTAL_VOLUME, var.set_total_volume_sensor)
    await add_sensor(CONF_CURRENT_POWER, var.set_current_power_sensor)
    await add_sensor(CONF_TEMP_FLOW, var.set_temp_flow_sensor)
    await add_sensor(CONF_TEMP_RETURN, var.set_temp_return_sensor)
    await add_sensor(CONF_TEMP_DIFF, var.set_temp_diff_sensor)

    if CONF_METER_TIME in config:
        ts = await text_sensor.new_text_sensor(config[CONF_METER_TIME])
        cg.add(var.set_meter_time_sensor(ts))
