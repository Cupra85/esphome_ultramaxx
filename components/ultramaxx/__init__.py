
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart

CODEOWNERS = ["@deinname"]
DEPENDENCIES = ["uart"]
ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")

CONF_ULTRAMAXX_ID = "ultramaxx_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ultramaxx_ns.class_("UltraMaXXComponent", cg.Component)),
}).extend(cv.COMPONENT_SCHEMA)
