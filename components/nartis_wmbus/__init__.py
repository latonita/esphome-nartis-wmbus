import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

CODEOWNERS = ["@latonita"]

DEPENDENCIES = []
AUTO_LOAD = ["sensor", "text_sensor"]

MULTI_CONF = False

CONF_NARTIS_WMBUS_ID = "nartis_wmbus_id"

CONF_PIN_SDIO = "pin_sdio"
CONF_PIN_SCLK = "pin_sclk"
CONF_PIN_CSB = "pin_csb"
CONF_PIN_FCSB = "pin_fcsb"
CONF_PIN_GPIO1 = "pin_gpio1"
CONF_CHANNEL = "channel"
CONF_DECRYPTION_KEY = "decryption_key"
CONF_SYSTEM_TITLE = "system_title"
CONF_MODE = "mode"

# Default AES-128 key from firmware ROM 0x23848: "ZCZfuT666iRdgPNH"
DEFAULT_KEY = "5A435A66755436363669526467504E48"

# Default DLMS system title from EEPROM factory default (placeholder)
DEFAULT_SYSTEM_TITLE = "1122334455667788"


def validate_key(value):
    value = cv.string_strict(value)
    if len(value) != 32:
        raise cv.Invalid("Decryption key must be 32 hex characters (16 bytes)")
    try:
        return [int(value[i : i + 2], 16) for i in range(0, 32, 2)]
    except ValueError as exc:
        raise cv.Invalid("Decryption key must be hex values from 00 to FF") from exc


def validate_system_title(value):
    value = cv.string_strict(value)
    if len(value) != 16:
        raise cv.Invalid("System title must be 16 hex characters (8 bytes)")
    try:
        return [int(value[i : i + 2], 16) for i in range(0, 16, 2)]
    except ValueError as exc:
        raise cv.Invalid("System title must be hex values from 00 to FF") from exc

nartis_wmbus_ns = cg.esphome_ns.namespace("nartis_wmbus")
NartisWmbusComponent = nartis_wmbus_ns.class_(
    "NartisWmbusComponent", cg.PollingComponent
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NartisWmbusComponent),
            cv.Required(CONF_PIN_SDIO): pins.gpio_output_pin_schema,
            cv.Required(CONF_PIN_SCLK): pins.gpio_output_pin_schema,
            cv.Required(CONF_PIN_CSB): pins.gpio_output_pin_schema,
            cv.Required(CONF_PIN_FCSB): pins.gpio_output_pin_schema,
            cv.Optional(CONF_PIN_GPIO1): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_CHANNEL, default=1): cv.int_range(min=0, max=3),
            cv.Optional(CONF_DECRYPTION_KEY, default=DEFAULT_KEY): validate_key,
            cv.Optional(CONF_SYSTEM_TITLE, default=DEFAULT_SYSTEM_TITLE): validate_system_title,
            cv.Optional(CONF_MODE, default="session"): cv.one_of(
                "session", "listen", "sniffer", lower=True
            ),
            cv.Optional(
                CONF_UPDATE_INTERVAL, default="60s"
            ): cv.update_interval,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin_sdio = await cg.gpio_pin_expression(config[CONF_PIN_SDIO])
    cg.add(var.set_pin_sdio(pin_sdio))

    pin_sclk = await cg.gpio_pin_expression(config[CONF_PIN_SCLK])
    cg.add(var.set_pin_sclk(pin_sclk))

    pin_csb = await cg.gpio_pin_expression(config[CONF_PIN_CSB])
    cg.add(var.set_pin_csb(pin_csb))

    pin_fcsb = await cg.gpio_pin_expression(config[CONF_PIN_FCSB])
    cg.add(var.set_pin_fcsb(pin_fcsb))

    if CONF_PIN_GPIO1 in config:
        pin_gpio1 = await cg.gpio_pin_expression(config[CONF_PIN_GPIO1])
        cg.add(var.set_pin_gpio1(pin_gpio1))

    cg.add(var.set_channel(config[CONF_CHANNEL]))

    key = ", ".join(str(b) for b in config[CONF_DECRYPTION_KEY])
    cg.add(var.set_decryption_key(cg.RawExpression(f"{{{key}}}")))

    title = ", ".join(str(b) for b in config[CONF_SYSTEM_TITLE])
    cg.add(var.set_system_title(cg.RawExpression(f"{{{title}}}")))

    mode_map = {"session": 0, "listen": 1, "sniffer": 2}
    cg.add(var.set_mode(mode_map[config[CONF_MODE]]))

    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
