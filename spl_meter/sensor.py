import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, UNIT_DECIBEL

DEPENDENCIES = ["esp32"]

spl_meter_ns = cg.esphome_ns.namespace("spl_meter")
I2SSPLMeter = spl_meter_ns.class_("I2SSPLMeter", sensor.Sensor, cg.PollingComponent)

CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_DECIBEL,
    icon="mdi:volume-high",        # use the literal icon name (no const)
    accuracy_decimals=1,
).extend({
    cv.Required("bclk_pin"): cv.int_,
    cv.Required("lrck_pin"): cv.int_,
    cv.Required("data_in_pin"): cv.int_,
    cv.Optional("sample_rate", default=16000): cv.int_,
    cv.Optional("window_ms", default=125): cv.int_,
    cv.Optional("calibration_offset_db", default=100.0): cv.float_,
}).extend(cv.polling_component_schema("1s"))

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config["bclk_pin"],
        config["lrck_pin"],
        config["data_in_pin"],
        config["sample_rate"],
        config["window_ms"],
        config["calibration_offset_db"],
    )
    await cg.register_component(var, config)
    await sensor.register_sensor(var, config)
    cg.add_include("spl_meter.h")  # ensure the header is compiled in
