import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from . import (
    NartisWmbusComponent,
    CONF_NARTIS_WMBUS_ID,
    nartis_wmbus_ns,
)

CONF_OBIS_CODE = "obis_code"
CONF_CLASS_ID = "class_id"
CONF_ATTRIBUTE = "attribute"

NartisWmbusSensor = nartis_wmbus_ns.class_("NartisWmbusSensor", sensor.Sensor)


def validate_obis_code(value):
    value = cv.string_strict(value)
    parts = value.split(".")
    if len(parts) != 6:
        raise cv.Invalid("OBIS code must have 6 dot-separated parts, e.g. '1.0.32.7.0.255'")
    for p in parts:
        try:
            v = int(p)
            if v < 0 or v > 255:
                raise cv.Invalid(f"OBIS code part '{p}' must be 0-255")
        except ValueError:
            raise cv.Invalid(f"OBIS code part '{p}' is not a valid integer")
    return value


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        NartisWmbusSensor,
    ).extend(
        {
            cv.GenerateID(CONF_NARTIS_WMBUS_ID): cv.use_id(NartisWmbusComponent),
            cv.Required(CONF_OBIS_CODE): validate_obis_code,
            cv.Optional(CONF_CLASS_ID, default=3): cv.int_range(min=1, max=255),
            cv.Optional(CONF_ATTRIBUTE, default=2): cv.int_range(min=1, max=255),
        }
    ),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_NARTIS_WMBUS_ID])
    var = await sensor.new_sensor(config)

    cg.add(var.set_obis_code(config[CONF_OBIS_CODE]))
    cg.add(var.set_class_id(config[CONF_CLASS_ID]))
    cg.add(var.set_attribute(config[CONF_ATTRIBUTE]))

    cg.add(component.register_sensor(var))
