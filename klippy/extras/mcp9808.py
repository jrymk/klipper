# Support for I2C based MCP9808 temperature sensors
#
# Copyright (C) 2023  jerrymk <github.com/jrymk>
# HEAVILY based on LM75 temperature sensor module by Boleslaw Ciesielski
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import bus

MCP9808_CHIP_ADDR = 0x30
MCP9808_I2C_SPEED = 400000
MCP9808_REGS = {
    'REGPTR': 0x00,
    'CONF': 0x01,
    'TUPPER': 0x02,
    'TLOWER': 0x03,
    'TCRIT': 0x04,
    'TAMBIENT': 0x05,
    'MANUFACTURERID': 0x06,
    'DEVICEID': 0x07,
    'RES': 0x08,
}
MCP9808_REPORT_TIME = .5
MCP9808_MIN_REPORT_TIME = .25  # resolution 0.0625 typical Tconv


class MCP9808:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(config, MCP9808_CHIP_ADDR,
                                           MCP9808_I2C_SPEED)
        self.mcu = self.i2c.get_mcu()
        self.report_time = config.getfloat('mcp9808_report_time', MCP9808_REPORT_TIME,
                                           minval=MCP9808_MIN_REPORT_TIME)
        self.temp = self.min_temp = self.max_temp = 0.0
        self.sample_timer = self.reactor.register_timer(self._sample_mcp9808)
        self.printer.add_object("mcp9808 " + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        self._init_mcp9808()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def degrees_from_sample(self, x):
        deg = (((x[0] & 15) << 8) + x[1])
        if (x[0] & 16) != 0:
            deg = deg - (1 << 13)
        return deg / 16

    def _init_mcp9808(self):
        # Check and report the chip ID but ignore errors since many
        # chips don't have it
        try:
            manufacturerid = self.read_register('MANUFACTURERID', 1)[1]
            deviceid = self.read_register('DEVICEID', 1)[0]
            logging.info("mcp9808: Manufacturer ID %#x, Device ID %#x" %
                         (manufacturerid, deviceid))
            self.write_register('RES', 3)  # set resolution to 0.0625 degrees
        except:
            pass

    def _sample_mcp9808(self, eventtime):
        try:
            sample = self.read_register('TAMBIENT', 2)
            self.temp = self.degrees_from_sample(sample)
        except Exception:
            logging.exception("mcp9808: Error reading data")
            self.temp = 0.0
            return self.reactor.NEVER

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "MCP9808 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + self.report_time

    def read_register(self, reg_name, read_len):
        # read a single register
        regs = [MCP9808_REGS[reg_name]]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])

    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = MCP9808_REGS[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
        }


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("MCP9808", MCP9808)
