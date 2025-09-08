import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import CONF_UART_ID, CONF_UPDATE_INTERVAL

DEPENDENCIES = ['uart']

wmz_mbus_ns = cg.esphome_ns.namespace('wmz_mbus_custom')
WMZMbusCustom = wmz_mbus_ns.class_('WMZMbusCustom', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(WMZMbusCustom),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_UPDATE_INTERVAL, default="86400s"): cv.update_interval,
    cv.Optional("heat_energy_kwh"): sensor.sensor_schema(),
    cv.Optional("volume_m3"): sensor.sensor_schema(),
    cv.Optional("flow_m3h"): sensor.sensor_schema(),
    cv.Optional("flow_temp_c"): sensor.sensor_schema(),
    cv.Optional("return_temp_c"): sensor.sensor_schema(),
    cv.Optional("power_kw"): sensor.sensor_schema(),
    cv.Optional("temp_dif_c"): sensor.sensor_schema(),
    cv.Optional("op_time_days"): sensor.sensor_schema(),
    cv.Optional("month_end_values_kwh"): sensor.sensor_schema(),
}).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    cg.add(var.set_uart(config[CONF_UART_ID]))
    cg.add(var.set_update_interval(config[cv.UPDATE_INTERVAL]))

    if "heat_energy_kwh" in config:
        sens = yield sensor.new_sensor(config["heat_energy_kwh"])
        cg.add(var.set_heat_energy(sens))
    if "volume_m3" in config:
        sens = yield sensor.new_sensor(config["volume_m3"])
        cg.add(var.set_volume(sens))
    if "flow_m3h" in config:
        sens = yield sensor.new_sensor(config["flow_m3h"])
        cg.add(var.set_flow(sens))
    if "flow_temp_c" in config:
        sens = yield sensor.new_sensor(config["flow_temp_c"])
        cg.add(var.set_flow_temp(sens))
    if "return_temp_c" in config:
        sens = yield sensor.new_sensor(config["return_temp_c"])
        cg.add(var.set_return_temp(sens))
    if "power_kw" in config:
        sens = yield sensor.new_sensor(config["power_kw"])
        cg.add(var.set_power(sens))
    if "temp_dif_c" in config:
        sens = yield sensor.new_sensor(config["temp_dif_c"])
        cg.add(var.set_temp_dif(sens))
    if "op_time_days" in config:
        sens = yield sensor.new_sensor(config["op_time_days"])
        cg.add(var.set_op_time(sens))
    if "month_end_values_kwh" in config:
        sens = yield sensor.new_sensor(config["month_end_values_kwh"])
        cg.add(var.set_month_end(sens))

    cg.add(var)
