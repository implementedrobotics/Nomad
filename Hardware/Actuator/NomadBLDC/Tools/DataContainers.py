"""Nomad Class Handler for Interaction with NomadBLDC Board"""

#  DataContainers.py
#
#  Created on: March 28, 2020
#      Author: Quincy Jones
# 
#  Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
#  Permission is hereby granted, free of charge, to any person obtaining a
#  copy of this software and associated documentation files (the "Software"),
#  to deal in the Software without restriction, including without limitation
#  the rights to use, copy, modify, merge, publish, distribute, sublicense,
#  and/or sell copies of the Software, and to permit persons to whom the Software
#  is furnished to do so, subject to the following conditions:
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
#  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
#  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


import struct
from dataclasses import dataclass

@dataclass
class LogPacket:
    log_length: int = None
    log_string: str = None

    @classmethod
    def unpack(cls, data):
        return struct.unpack(f"<{data[0]}s", data[1:])[0].decode("utf-8")

@dataclass
class DeviceStats:
    __fmt = "<BBIffff"
    fault: int = None
    control_status: int = None
    uptime: int = None
    voltage_bus: float = None
    driver_temp: float = None
    fet_temp: float = None
    motor_temp: float = None

    @classmethod
    def unpack(cls, data):
        unpacked = struct.unpack(cls.__fmt, data)
        return DeviceStats(*unpacked)

@dataclass
class DeviceInfo:
    fw_major: int = None
    fw_minor: int = None
    device_id: 'typing.Any' = None

@dataclass
class MotorConfig:
    __fmt = "<I8fiffi"
    num_pole_pairs: int = None
    phase_resistance: float = None
    phase_inductance_d: float = None
    phase_inductance_q: float = None
    K_v: float = None
    flux_linkage: float = None
    K_t: float = None
    K_t_out: float = None
    gear_ratio: float = None
    phase_order: int = None
    calib_current: float = None
    calib_voltage: float = None
    calibrated: int = None

    @classmethod
    def unpack(cls, data):
        unpacked = struct.unpack(cls.__fmt, data)
        return MotorConfig(*unpacked)

