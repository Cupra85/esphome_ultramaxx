import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

wmz_ns = cg.esphome_ns.namespace("wmz_mbus_custom")
WMZComponent = wmz_ns.class_("WMZComponent", cg.PollingComponent)

CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"

FIELDS = {
    "heat_energy_kwh": "heat_energy_kwh_",
    "volume_m3": "volume_m3_",
    "flow_temp_c": "flow_temp_c_",
    "return_temp_c": "return_temp_c_",
    "power_w": "power_w_",
    "flow_lh": "flow_lh_",
}

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(WMZComponent),
    cv.Required(CONF_TX_PIN): cv.int_range(min=0),
    cv.Required(CONF_RX_PIN): cv.int_range(min=0),
    cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
    **{cv.Optional(k): sensor.sensor_schema() for k in FIELDS.keys()},
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_TX_PIN],
        config[CONF_RX_PIN],
        config[CONF_UPDATE_INTERVAL]
    )
    await cg.register_component(var, config)
    for key, attr in FIELDS.items():
        if key in config:
            sens = getattr(var, attr)
            await sensor.register_sensor(sens, config[key])
