# esphome/components/carrier_uart_ir/climate.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, climate
from esphome.const import CONF_ID

CODEOWNERS = ["@your_nick"]  # 

carrier_uart_ir_ns = cg.esphome_ns.namespace("carrier_uart_ir")
CarrierUartBridge = carrier_uart_ir_ns.class_(
    "CarrierUartBridge",
    climate.Climate,
    cg.Component,
    uart.UARTDevice,
)

CONF_UART_ID = "uart_id"

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(CarrierUartBridge),
        cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    }
)


async def to_code(config):
    uart_comp = await cg.get_variable(config[CONF_UART_ID])

    var = cg.new_Pvariable(config[CONF_ID], uart_comp)
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
