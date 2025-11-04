import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_TX_PIN,
    CONF_RX_PIN,
    CONF_UPDATE_INTERVAL,
)

DEPENDENCIES = ["sensor", "text_sensor"]
CODEOWNERS = ["@Cupra85"]

CONF_HEAT = "heat_energy_kwh"
CONF_VOLUME = "volume_m3"
CONF_FLOW_TEMP = "flow_temp_c"
CONF_RETURN_TEMP = "return_temp_c"
CONF_POWER = "power_w"
CONF_FLOW = "flow_lh"
CONF_DEBUG = "debug_frame"
CONF_INVERT = "invert_lines"

wmz_ns = cg.esphome_ns.namespace("wmz_mbus_custom")
WMZComponent = wmz_ns.class_("WMZComponent", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(WMZComponent),
    cv.Required(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
    cv.Required(CONF_RX_PIN): pins.internal_gpio_input_pin_number,
    cv.Optional(CONF_INVERT, default=False): cv.boolean,
    cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
    cv.Optional(CONF_HEAT): sensor.sensor_schema(unit_of_measurement="kWh"),
    cv.Optional(CONF_VOLUME): sensor.sensor_schema(unit_of_measurement="m³"),
    cv.Optional(CONF_FLOW_TEMP): sensor.sensor_schema(unit_of_measurement="°C"),
    cv.Optional(CONF_RETURN_TEMP): sensor.sensor_schema(unit_of_measurement="°C"),
    cv.Optional(CONF_POWER): sensor.sensor_schema(unit_of_measurement="W"),
    cv.Optional(CONF_FLOW): sensor.sensor_schema(unit_of_measurement="l/h"),
    cv.Optional(CONF_DEBUG): text_sensor.text_sensor_schema(),
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID],
                           config[CONF_TX_PIN],
                           config[CONF_RX_PIN],
                           config[CONF_INVERT],
                           config[CONF_UPDATE_INTERVAL])
    await cg.register_component(var, config)

    if CONF_HEAT in config:
        sens = await sensor.new_sensor(config[CONF_HEAT])
        cg.add(var.heat_energy_kwh_, sens)
    if CONF_VOLUME in config:
        sens = await sensor.new_sensor(config[CONF_VOLUME])
        cg.add(var.volume_m3_, sens)
    if CONF_FLOW_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_FLOW_TEMP])
        cg.add(var.flow_temp_c_, sens)
    if CONF_RETURN_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_RETURN_TEMP])
        cg.add(var.return_temp_c_, sens)
    if CONF_POWER in config:
        sens = await sensor.new_sensor(config[CONF_POWER])
        cg.add(var.power_w_, sens)
    if CONF_FLOW in config:
        sens = await sensor.new_sensor(config[CONF_FLOW])
        cg.add(var.flow_lh_, sens)
    if CONF_DEBUG in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DEBUG])
        cg.add(var.debug_frame_, sens)
