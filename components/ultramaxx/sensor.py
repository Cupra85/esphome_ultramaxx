import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID

# WICHTIG: Das hier muss exakt so bleiben
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

# ⭐ KORREKTUR: Wir nutzen cv.typed_schema, um ESPHome zu zwingen, 
# es als Sensor-Plattform zu akzeptieren
PLATFORM_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UltraMaXXComponent),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

        cv.Optional(CONF_SERIAL_NUMBER): sensor.sensor_schema(),
        cv.Optional(CONF_TOTAL_ENERGY): sensor.sensor_schema(unit_of_measurement="kWh"),
        cv.Optional(CONF_TOTAL_VOLUME): sensor.sensor_schema(unit_of_measurement="m³"),
        cv.Optional(CONF_CURRENT_POWER): sensor.sensor_schema(unit_of_measurement="kW"),
        cv.Optional(CONF_TEMP_FLOW): sensor.sensor_schema(unit_of_measurement="°C"),
        cv.Optional(CONF_TEMP_RETURN): sensor.sensor_schema(unit_of_measurement="°C"),
        cv.Optional(CONF_TEMP_DIFF): sensor.sensor_schema(unit_of_measurement="°C"),
        cv.Optional(CONF_METER_TIME): text_sensor.text_sensor_schema(),
    }
).extend(cv.polling_component_schema("20s")).extend(uart.UART_DEVICE_SCHEMA)
