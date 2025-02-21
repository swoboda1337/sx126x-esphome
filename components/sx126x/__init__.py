from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import spi
import esphome.config_validation as cv
from esphome.const import CONF_DATA, CONF_FREQUENCY, CONF_ID
from esphome.core import TimePeriod

MULTI_CONF = True
CODEOWNERS = ["@swoboda1337"]
DEPENDENCIES = ["spi"]

CONF_BANDWIDTH = "bandwidth"
CONF_BITRATE = "bitrate"
CONF_BITSYNC = "bitsync"
CONF_CODING_RATE = "coding_rate"
CONF_CRC_ENABLE = "crc_enable"
CONF_DEVIATION = "deviation"
CONF_DIO1_PIN = "dio1_pin"
CONF_MODULATION = "modulation"
CONF_ON_PACKET = "on_packet"
CONF_PA_PIN = "pa_pin"
CONF_PA_POWER = "pa_power"
CONF_PA_RAMP = "pa_ramp"
CONF_PAYLOAD_LENGTH = "payload_length"
CONF_PREAMBLE_ERRORS = "preamble_errors"
CONF_PREAMBLE_POLARITY = "preamble_polarity"
CONF_PREAMBLE_SIZE = "preamble_size"
CONF_RST_PIN = "rst_pin"
CONF_BUSY_PIN = "busy_pin"
CONF_RX_FLOOR = "rx_floor"
CONF_RX_START = "rx_start"
CONF_SHAPING = "shaping"
CONF_SPREADING_FACTOR = "spreading_factor"
CONF_SYNC_VALUE = "sync_value"
CONF_TCXO_VOLTAGE = "tcxo_voltage"
CONF_TCXO_DELAY = "tcxo_delay"

sx126x_ns = cg.esphome_ns.namespace("sx126x")
SX126x = sx126x_ns.class_("SX126x", cg.Component, spi.SPIDevice)
SX126xBw = sx126x_ns.enum("SX126xBw")
SX126xPacketType = sx126x_ns.enum("SX126xPacketType")
SX126xTcxoCtrl =  sx126x_ns.enum("SX126xTcxoCtrl")
SX126xPaConfig = sx126x_ns.enum("SX126xPaConfig")
SX126xPaRamp = sx126x_ns.enum("SX126xPaRamp")
SX126xLoraCr = sx126x_ns.enum("SX126xLoraCr")

BW = {
    "2_6kHz": SX126xBw.SX126X_BW_2_6,
    "3_1kHz": SX126xBw.SX126X_BW_3_1,
    "3_9kHz": SX126xBw.SX126X_BW_3_9,
    "5_2kHz": SX126xBw.SX126X_BW_5_2,
    "6_3kHz": SX126xBw.SX126X_BW_6_3,
    "7_8kHz": SX126xBw.SX126X_BW_7_8,
    "10_4kHz": SX126xBw.SX126X_BW_10_4,
    "12_5kHz": SX126xBw.SX126X_BW_12_5,
    "15_6kHz": SX126xBw.SX126X_BW_15_6,
    "20_8kHz": SX126xBw.SX126X_BW_20_8,
    "25_0kHz": SX126xBw.SX126X_BW_25_0,
    "31_3kHz": SX126xBw.SX126X_BW_31_3,
    "41_7kHz": SX126xBw.SX126X_BW_41_7,
    "50_0kHz": SX126xBw.SX126X_BW_50_0,
    "62_5kHz": SX126xBw.SX126X_BW_62_5,
    "83_3kHz": SX126xBw.SX126X_BW_83_3,
    "100_0kHz": SX126xBw.SX126X_BW_100_0,
    "125_0kHz": SX126xBw.SX126X_BW_125_0,
    "166_7kHz": SX126xBw.SX126X_BW_166_7,
    "200_0kHz": SX126xBw.SX126X_BW_200_0,
    "250_0kHz": SX126xBw.SX126X_BW_250_0,
    "500_0kHz": SX126xBw.SX126X_BW_500_0,
}

CODING_RATE = {
    "CR_4_5": SX126xLoraCr.LORA_CR_4_5,
    "CR_4_6": SX126xLoraCr.LORA_CR_4_6,
    "CR_4_7": SX126xLoraCr.LORA_CR_4_7,
    "CR_4_8": SX126xLoraCr.LORA_CR_4_8,
}

MOD = {
    "LORA": SX126xPacketType.PACKET_TYPE_LORA,
    "GFSK": SX126xPacketType.PACKET_TYPE_GFSK,
    "LRHSS": SX126xPacketType.PACKET_TYPE_LRHSS,
}

PA_PIN = {
    "RFO": SX126xPaConfig.PA_PIN_RFO,
    "BOOST": SX126xPaConfig.PA_PIN_BOOST,
}

TCXO_VOLTAGE = {
    "1_6V": SX126xTcxoCtrl.TCXO_CTRL_1_6V,
    "1_7V": SX126xTcxoCtrl.TCXO_CTRL_1_7V,
    "1_8V": SX126xTcxoCtrl.TCXO_CTRL_1_8V,
    "2_2V": SX126xTcxoCtrl.TCXO_CTRL_2_2V,
    "2_4V": SX126xTcxoCtrl.TCXO_CTRL_2_4V,
    "2_7V": SX126xTcxoCtrl.TCXO_CTRL_2_7V,
    "3_0V": SX126xTcxoCtrl.TCXO_CTRL_3_0V,
    "3_3V": SX126xTcxoCtrl.TCXO_CTRL_3_3V,
    "NONE": SX126xTcxoCtrl.TCXO_CTRL_NONE,
}

RAMP = {
    "10us": SX126xPaRamp.PA_RAMP_10,
    "12us": SX126xPaRamp.PA_RAMP_12,
    "15us": SX126xPaRamp.PA_RAMP_15,
    "20us": SX126xPaRamp.PA_RAMP_20,
    "25us": SX126xPaRamp.PA_RAMP_25,
    "31us": SX126xPaRamp.PA_RAMP_31,
    "40us": SX126xPaRamp.PA_RAMP_40,
    "50us": SX126xPaRamp.PA_RAMP_50,
    "62us": SX126xPaRamp.PA_RAMP_62,
    "100us": SX126xPaRamp.PA_RAMP_100,
    "125us": SX126xPaRamp.PA_RAMP_125,
    "250us": SX126xPaRamp.PA_RAMP_250,
    "500us": SX126xPaRamp.PA_RAMP_500,
    "1000us": SX126xPaRamp.PA_RAMP_1000,
    "2000us": SX126xPaRamp.PA_RAMP_2000,
    "3400us": SX126xPaRamp.PA_RAMP_3400,
}

SHAPING = {
    "CUTOFF_BR_X_2": SX126xPaRamp.CUTOFF_BR_X_2,
    "CUTOFF_BR_X_1": SX126xPaRamp.CUTOFF_BR_X_1,
    "GAUSSIAN_BT_0_3": SX126xPaRamp.GAUSSIAN_BT_0_3,
    "GAUSSIAN_BT_0_5": SX126xPaRamp.GAUSSIAN_BT_0_5,
    "GAUSSIAN_BT_1_0": SX126xPaRamp.GAUSSIAN_BT_1_0,
    "NONE": SX126xPaRamp.SHAPING_NONE,
}

SendPacketAction = sx126x_ns.class_(
    "SendPacketAction", automation.Action, cg.Parented.template(SX126x)
)
SetModeTxAction = sx126x_ns.class_("SetModeTxAction", automation.Action)
SetModeRxAction = sx126x_ns.class_("SetModeRxAction", automation.Action)
SetModeStandbyAction = sx126x_ns.class_("SetModeStandbyAction", automation.Action)


def validate_raw_data(value):
    if isinstance(value, str):
        return value.encode("utf-8")
    if isinstance(value, str):
        return value
    if isinstance(value, list):
        return cv.Schema([cv.hex_uint8_t])(value)
    raise cv.Invalid(
        "data must either be a string wrapped in quotes or a list of bytes"
    )


def validate_config(config):
    if config[CONF_MODULATION] == "LORA":
        bws = [
            "7_8kHz",
            "10_4kHz",
            "15_6kHz",
            "20_8kHz",
            "31_3kHz",
            "41_7kHz",
            "62_5kHz",
            "125_0kHz",
            "250_0kHz",
            "500_0kHz",
        ]
        if config[CONF_BANDWIDTH] not in bws:
            raise cv.Invalid(
                f"Bandwidth {config[CONF_BANDWIDTH]} is not available with LORA"
            )
        if CONF_DIO1_PIN not in config:
            raise cv.Invalid("Cannot use LoRa without dio1_pin")
        if config[CONF_PREAMBLE_SIZE] > 0 and config[CONF_PREAMBLE_SIZE] < 6:
            raise cv.Invalid("Minimum preamble size is 6 with LORA")
        if config[CONF_SPREADING_FACTOR] == 6 and config[CONF_PAYLOAD_LENGTH] == 0:
            raise cv.Invalid("Payload length must be set when spreading factor is 6")
    else:
        if config[CONF_BANDWIDTH] == "500_0kHz":
            raise cv.Invalid(
                f"Bandwidth {config[CONF_BANDWIDTH]} is only available with LORA"
            )
        if config[CONF_PAYLOAD_LENGTH] > 64:
            raise cv.Invalid("Payload length must be >= 64 with FSK/OOK")
        if config[CONF_PAYLOAD_LENGTH] > 0 and CONF_DIO1_PIN not in config:
            raise cv.Invalid("Cannot use packet mode without dio1_pin")
        if CONF_BITRATE not in config:
            if config[CONF_PAYLOAD_LENGTH] > 0:
                raise cv.Invalid("Cannot use packet mode without setting bitrate")
            if CONF_BITSYNC in config and config[CONF_BITSYNC]:
                raise cv.Invalid("Bitsync is true but bitrate is not configured")
        elif CONF_BITSYNC not in config:
            raise cv.Invalid(
                "Bitrate is configured but not bitsync; add 'bitsync: true'"
            )
    if config[CONF_PA_PIN] == "RFO" and config[CONF_PA_POWER] > 15:
        raise cv.Invalid("PA power must be <= 15 dbm when using the RFO pin")
    if config[CONF_PA_PIN] == "BOOST" and config[CONF_PA_POWER] < 2:
        raise cv.Invalid("PA power must be >= 2 dbm when using the BOOST pin")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SX126x),
            cv.Optional(CONF_BANDWIDTH, default="125_0kHz"): cv.enum(BW),
            cv.Optional(CONF_BITRATE): cv.int_range(min=500, max=300000),
            cv.Optional(CONF_BITSYNC): cv.boolean,
            cv.Required(CONF_BUSY_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_CODING_RATE, default="CR_4_5"): cv.enum(CODING_RATE),
            cv.Optional(CONF_CRC_ENABLE, default=False): cv.boolean,
            cv.Optional(CONF_DEVIATION, default=5000): cv.int_range(min=0, max=100000),
            cv.Optional(CONF_DIO1_PIN): pins.internal_gpio_input_pin_schema,
            cv.Required(CONF_FREQUENCY): cv.int_range(min=137000000, max=1020000000),
            cv.Required(CONF_MODULATION): cv.enum(MOD),
            cv.Optional(CONF_ON_PACKET): automation.validate_automation(single=True),
            cv.Optional(CONF_PA_PIN, default="BOOST"): cv.enum(PA_PIN),
            cv.Optional(CONF_PA_POWER, default=17): cv.int_range(min=0, max=17),
            cv.Optional(CONF_PA_RAMP, default="40us"): cv.enum(RAMP),
            cv.Optional(CONF_PAYLOAD_LENGTH, default=0): cv.int_range(min=0, max=256),
            cv.Optional(CONF_PREAMBLE_ERRORS, default=0): cv.int_range(min=0, max=31),
            cv.Optional(CONF_PREAMBLE_POLARITY, default=0xAA): cv.All(
                cv.hex_int, cv.one_of(0xAA, 0x55)
            ),
            cv.Optional(CONF_PREAMBLE_SIZE, default=0): cv.int_range(min=0, max=65535),
            cv.Required(CONF_RST_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_RX_FLOOR, default=-94): cv.float_range(min=-128, max=-1),
            cv.Optional(CONF_RX_START, default=True): cv.boolean,
            cv.Optional(CONF_SHAPING, default="NONE"): cv.enum(SHAPING),
            cv.Optional(CONF_SPREADING_FACTOR, default=7): cv.int_range(min=6, max=12),
            cv.Optional(CONF_SYNC_VALUE, default=[]): cv.ensure_list(cv.hex_uint8_t),
            cv.Optional(CONF_TCXO_VOLTAGE, default="NONE"): cv.enum(TCXO_VOLTAGE),
            cv.Optional(CONF_TCXO_DELAY, default="5ms"): cv.All(
                cv.positive_time_period_microseconds,
                cv.Range(max=TimePeriod(microseconds=262144000)),
            ),
        },
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(True, 8e6, "mode0")),
    validate_config,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    if CONF_ON_PACKET in config:
        await automation.build_automation(
            var.get_packet_trigger(),
            [
                (cg.std_vector.template(cg.uint8), "x"),
                (cg.float_, "rssi"),
                (cg.float_, "snr"),
            ],
            config[CONF_ON_PACKET],
        )
    if CONF_DIO1_PIN in config:
        dio1_pin = await cg.gpio_pin_expression(config[CONF_DIO1_PIN])
        cg.add(var.set_dio1_pin(dio1_pin))
    rst_pin = await cg.gpio_pin_expression(config[CONF_RST_PIN])
    cg.add(var.set_rst_pin(rst_pin))
    busy_pin = await cg.gpio_pin_expression(config[CONF_BUSY_PIN])
    cg.add(var.set_busy_pin(busy_pin))
    cg.add(var.set_bandwidth(config[CONF_BANDWIDTH]))
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_deviation(config[CONF_DEVIATION]))
    cg.add(var.set_modulation(config[CONF_MODULATION]))
    cg.add(var.set_pa_pin(config[CONF_PA_PIN]))
    cg.add(var.set_pa_ramp(config[CONF_PA_RAMP]))
    cg.add(var.set_pa_power(config[CONF_PA_POWER]))
    cg.add(var.set_shaping(config[CONF_SHAPING]))
    if CONF_BITRATE in config:
        cg.add(var.set_bitrate(config[CONF_BITRATE]))
    else:
        cg.add(var.set_bitrate(4800))
    if CONF_BITSYNC in config:
        cg.add(var.set_bitsync(config[CONF_BITSYNC]))
    else:
        cg.add(var.set_bitsync(False))
    cg.add(var.set_crc_enable(config[CONF_CRC_ENABLE]))
    cg.add(var.set_payload_length(config[CONF_PAYLOAD_LENGTH]))
    cg.add(var.set_preamble_size(config[CONF_PREAMBLE_SIZE]))
    cg.add(var.set_preamble_polarity(config[CONF_PREAMBLE_POLARITY]))
    cg.add(var.set_preamble_errors(config[CONF_PREAMBLE_ERRORS]))
    cg.add(var.set_coding_rate(config[CONF_CODING_RATE]))
    cg.add(var.set_spreading_factor(config[CONF_SPREADING_FACTOR]))
    cg.add(var.set_sync_value(config[CONF_SYNC_VALUE]))
    cg.add(var.set_rx_floor(config[CONF_RX_FLOOR]))
    cg.add(var.set_rx_start(config[CONF_RX_START]))
    cg.add(var.set_tcxo_voltage(config[CONF_TCXO_VOLTAGE]))
    cg.add(var.set_tcxo_delay(config[CONF_TCXO_DELAY]))


SET_MODE_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(SX126x),
    }
)


@automation.register_action(
    "sx126x.set_mode_tx", SetModeTxAction, SET_MODE_ACTION_SCHEMA
)
@automation.register_action(
    "sx126x.set_mode_rx", SetModeRxAction, SET_MODE_ACTION_SCHEMA
)
@automation.register_action(
    "sx126x.set_mode_standby", SetModeStandbyAction, SET_MODE_ACTION_SCHEMA
)
async def set_mode_action_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    return var


SEND_PACKET_ACTION_SCHEMA = cv.maybe_simple_value(
    {
        cv.GenerateID(): cv.use_id(SX126x),
        cv.Required(CONF_DATA): cv.templatable(validate_raw_data),
    },
    key=CONF_DATA,
)


@automation.register_action(
    "sx126x.send_packet", SendPacketAction, SEND_PACKET_ACTION_SCHEMA
)
async def send_packet_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    data = config[CONF_DATA]
    if isinstance(data, bytes):
        data = list(data)
    if cg.is_template(data):
        templ = await cg.templatable(data, args, cg.std_vector.template(cg.uint8))
        cg.add(var.set_data_template(templ))
    else:
        cg.add(var.set_data_static(data))
    return var
