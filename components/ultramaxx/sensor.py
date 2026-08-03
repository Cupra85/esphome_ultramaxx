import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID
from esphome.const import (
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE_DELTA,
    DEVICE_CLASS_VOLUME,
    DEVICE_CLASS_VOLUME_FLOW_RATE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_CUBIC_METER,
    UNIT_KELVIN,
    UNIT_KILOWATT,
    UNIT_KILOWATT_HOURS,
    UNIT_LITRE_PER_HOUR,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

ultramaxx_ns = cg.esphome_ns.namespace("ultramaxx")
UltraMaXXComponent = ultramaxx_ns.class_(
    "UltraMaXXComponent",
    cg.PollingComponent,
    uart.UARTDevice,
)

CONF_SERIAL_NUMBER = "serial_number"
CONF_TOTAL_ENERGY = "total_energy"
CONF_TOTAL_VOLUME = "total_volume"
CONF_POWER = "power"
CONF_FLOW = "flow"
CONF_TEMP_FLOW = "temp_flow"
CONF_TEMP_RETURN = "temp_return"
CONF_TEMP_DIFF = "temp_diff"
CONF_METER_TIME = "meter_time"
CONF_OPERATING_TIME = "operating_time"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_SOFTWARE_VERSION = "software_version"
CONF_ACCESS_COUNTER = "access_counter"
CONF_STATUS_TEXT = "status_text"


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(UltraMaXXComponent),

            # Bleibt ausdrücklich erhalten.
            # UART_DEVICE_SCHEMA stellt uart_id zusätzlich für UART-Geräte bereit.
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

            # Seriennummer bleibt entsprechend deinem bestehenden Code
            # ein numerischer Sensor.
            cv.Optional(CONF_SERIAL_NUMBER):
                sensor.sensor_schema(
                    accuracy_decimals=0,
                    icon="mdi:barcode-scan",
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),

            # Kumulierter Energiezähler.
            cv.Optional(CONF_TOTAL_ENERGY):
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_KILOWATT_HOURS,
                    accuracy_decimals=0,
                    device_class=DEVICE_CLASS_ENERGY,
                    state_class=STATE_CLASS_TOTAL_INCREASING,
                    icon="mdi:transmission-tower",
                ),

            # Kumulierter Volumenzähler.
            cv.Optional(CONF_TOTAL_VOLUME):
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_CUBIC_METER,
                    accuracy_decimals=2,
                    device_class=DEVICE_CLASS_VOLUME,
                    state_class=STATE_CLASS_TOTAL_INCREASING,
                    icon="mdi:water",
                ),

            # Aktuelle thermische Leistung.
            cv.Optional(CONF_POWER):
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_KILOWATT,
                    accuracy_decimals=1,
                    device_class=DEVICE_CLASS_POWER,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:flash",
                ),

            # Aktueller Volumenstrom.
            cv.Optional(CONF_FLOW):
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_LITRE_PER_HOUR,
                    accuracy_decimals=0,
                    device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:pipe",
                ),

            # Vorlauftemperatur.
            cv.Optional(CONF_TEMP_FLOW):
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_CELSIUS,
                    accuracy_decimals=1,
                    device_class=DEVICE_CLASS_TEMPERATURE,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:thermometer-plus",
                ),

            # Rücklauftemperatur.
            cv.Optional(CONF_TEMP_RETURN):
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_CELSIUS,
                    accuracy_decimals=1,
                    device_class=DEVICE_CLASS_TEMPERATURE,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:thermometer-minus",
                ),

            # Temperaturdifferenz wird laut UltraMaXX-Protokoll in Kelvin
            # übertragen. Die numerische Differenz ist zu °C identisch.
            cv.Optional(CONF_TEMP_DIFF):
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_KELVIN,
                    accuracy_decimals=2,
                    device_class=DEVICE_CLASS_TEMPERATURE_DELTA,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:thermometer-lines",
                ),

            # Kumulierter Betriebszeitzähler in Tagen.
            cv.Optional(CONF_OPERATING_TIME):
                sensor.sensor_schema(
                    unit_of_measurement="d",
                    accuracy_decimals=0,
                    device_class=DEVICE_CLASS_DURATION,
                    state_class=STATE_CLASS_TOTAL_INCREASING,
                    icon="mdi:calendar-start",
                ),

            # Firmwareversion ist ein Diagnosewert und kein Messwert.
            cv.Optional(CONF_FIRMWARE_VERSION):
                sensor.sensor_schema(
                    accuracy_decimals=0,
                    icon="mdi:chip",
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),

            # Softwareversion ist ein Diagnosewert und kein Messwert.
            cv.Optional(CONF_SOFTWARE_VERSION):
                sensor.sensor_schema(
                    accuracy_decimals=0,
                    icon="mdi:chip",
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),

            # Position 15: Number of reading / Access counter.
            #
            # Dieser Wert bleibt ein normaler numerischer Sensor.
            # Er wird nicht als total_increasing gekennzeichnet, da der
            # Zugriffszähler überlaufen beziehungsweise zurückgesetzt
            # werden kann.
            cv.Optional(CONF_ACCESS_COUNTER):
                sensor.sensor_schema(
                    accuracy_decimals=0,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:timer-check",
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),

            # Position 16: dekodierter Fehlerstatus als Text.
            cv.Optional(CONF_STATUS_TEXT):
                text_sensor.text_sensor_schema(
                    icon="mdi:alert",
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),

            # Datum und Uhrzeit aus dem UltraMaXX-Telegramm.
            cv.Optional(CONF_METER_TIME):
                text_sensor.text_sensor_schema(
                    icon="mdi:timer-cog",
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),
        }
    )
    .extend(cv.polling_component_schema("86400s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


# Der UltraMaXX wird fest mit 2400 Baud ausgelesen.
# TX wird für REQ_UD2 benötigt, RX für den Longframe.
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ultramaxx",
    baud_rate=2400,
    require_tx=True,
    require_rx=True,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    mapping = [
        (CONF_SERIAL_NUMBER, "set_serial_number_sensor"),
        (CONF_TOTAL_ENERGY, "set_total_energy_sensor"),
        (CONF_TOTAL_VOLUME, "set_total_volume_sensor"),
        (CONF_POWER, "set_current_power_sensor"),
        (CONF_FLOW, "set_flow_sensor"),
        (CONF_TEMP_FLOW, "set_temp_flow_sensor"),
        (CONF_TEMP_RETURN, "set_temp_return_sensor"),
        (CONF_TEMP_DIFF, "set_temp_diff_sensor"),
        (CONF_OPERATING_TIME, "set_operating_time_sensor"),
        (CONF_FIRMWARE_VERSION, "set_firmware_version_sensor"),
        (CONF_SOFTWARE_VERSION, "set_software_version_sensor"),
        (CONF_ACCESS_COUNTER, "set_access_counter_sensor"),
    ]

    for key, setter in mapping:
        if key in config:
            s = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(s))

    if CONF_METER_TIME in config:
        ts = await text_sensor.new_text_sensor(config[CONF_METER_TIME])
        cg.add(var.set_meter_time_sensor(ts))

    if CONF_STATUS_TEXT in config:
        st = await text_sensor.new_text_sensor(config[CONF_STATUS_TEXT])
        cg.add(var.set_status_text_sensor(st))
