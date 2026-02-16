import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID, CONF_UPDATE_INTERVAL

# WICHTIG: Diese Zeile stellt sicher, dass ESPHome weiß, dass es eine Sensor-Plattform ist
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")
UltraMaXXComponent = ultramaxx_ns.class_(
    "UltraMaXXComponent", cg.PollingComponent, uart.UARTDevice
)

# ⭐ DAS SCHEMA MUSS GENAU SO AUSSEHEN:
PLATFORM_SCHEMA = cv.Schema(
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
).extend(cv.polling_component_schema("20s")).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    # Den UART-Parent holen
    parent = await cg.get_variable(config[CONF_UART_ID])
    # Die C++ Instanz erstellen
    var = cg.new_Pvariable(config[CONF_ID], parent)
    
    # Als PollingComponent registrieren (wichtig für update_interval)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Sensoren registrieren
    if "serial_number" in config:
        sens = await sensor.new_sensor(config["serial_number"])
        cg.add(var.set_serial_number_sensor(sens))
    if "total_energy" in config:
        sens = await sensor.new_sensor(config["total_energy"])
        cg.add(var.set_total_energy_sensor(sens))
    if "total_volume" in config:
        sens = await sensor.new_sensor(config["total_volume"])
        cg.add(var.set_total_volume_sensor(sens))
    if "current_power" in config:
        sens = await sensor.new_sensor(config["current_power"])
        cg.add(var.set_current_power_sensor(sens))
    if "temp_flow" in config:
        sens = await sensor.new_sensor(config["temp_flow"])
        cg.add(var.set_temp_flow_sensor(sens))
    if "temp_return" in config:
        sens = await sensor.new_sensor(config["temp_return"])
        cg.add(var.set_temp_return_sensor(sens))
    if "temp_diff" in config:
        sens = await sensor.new_sensor(config["temp_diff"])
        cg.add(var.set_temp_diff_sensor(sens))
    if "meter_time" in config:
        ts = await text_sensor.new_text_sensor(config["meter_time"])
        cg.add(var.set_meter_time_sensor(ts))
