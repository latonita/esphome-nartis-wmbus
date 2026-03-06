import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import (
    NartisWmbusComponent,
    CONF_NARTIS_WMBUS_ID,
    nartis_wmbus_ns,
)
from .sensor import CONF_OBIS_CODE, CONF_CLASS_ID, CONF_ATTRIBUTE, validate_obis_code

AUTO_LOAD = ["nartis_wmbus"]

NartisWmbusTextSensor = nartis_wmbus_ns.class_(
    "NartisWmbusTextSensor", text_sensor.TextSensor
)

CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(
        NartisWmbusTextSensor,
    ).extend(
        {
            cv.GenerateID(CONF_NARTIS_WMBUS_ID): cv.use_id(NartisWmbusComponent),
            cv.Required(CONF_OBIS_CODE): validate_obis_code,
            cv.Optional(CONF_CLASS_ID, default=8): cv.int_range(min=1, max=255),
            cv.Optional(CONF_ATTRIBUTE, default=2): cv.int_range(min=1, max=255),
        }
    ),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_NARTIS_WMBUS_ID])
    var = await text_sensor.new_text_sensor(config)

    cg.add(var.set_obis_code(config[CONF_OBIS_CODE]))
    cg.add(var.set_class_id(config[CONF_CLASS_ID]))
    cg.add(var.set_attribute(config[CONF_ATTRIBUTE]))

    cg.add(component.register_sensor(var))
