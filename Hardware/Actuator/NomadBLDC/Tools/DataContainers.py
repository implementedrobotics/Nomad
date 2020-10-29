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
from typing import List, ClassVar


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
    current_bus: float = None
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
    __packet : ClassVar[struct.Struct] = struct.Struct('<I10fiffi')
    num_pole_pairs: int = None
    continuous_current_max: float = None
    continuous_current_tau: float = None
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

    def pack(self):
        return self.__packet.pack(self.num_pole_pairs,
        self.continuous_current_max,
        self.continuous_current_tau,
        self.phase_resistance,
        self.phase_inductance_d,
        self.phase_inductance_q,
        self.K_v,
        self.flux_linkage,
        self.K_t,
        self.K_t_out,
        self.gear_ratio,
        self.phase_order,
        self.calib_current,
        self.calib_voltage,
        self.calibrated)

    @classmethod
    def unpack(cls, data):
        unpacked = cls.__packet.unpack(data)
        return MotorConfig(*unpacked)


@dataclass
class ControllerConfig:
    #__fmt = "<16fI"
    __packet : ClassVar[struct.Struct] = struct.Struct('<16fI')
    k_d: float = None
    k_q: float = None
    k_i_d: float = None
    k_i_q: float = None
    alpha: float = None
    overmodulation: float = None
    velocity_limit: float = None
    position_limit: float = None
    torque_limit: float = None
    current_limit: float = None
    current_bandwidth: float = None
    K_p_min: float = None
    K_p_max: float = None
    K_d_min: float = None
    K_d_max: float = None
    pwm_freq: float = None
    foc_ccl_divider: int = None

    def pack(self):
        return self.__packet.pack(self.k_d,
        self.k_q,
        self.k_i_d,
        self.k_i_q,
        self.alpha,
        self.overmodulation,
        self.velocity_limit,
        self.position_limit,
        self.torque_limit,
        self.current_limit,
        self.current_bandwidth,
        self.K_p_min,
        self.K_p_max,
        self.K_d_min,
        self.K_d_max,
        self.pwm_freq,
        self.foc_ccl_divider)

    @classmethod
    def unpack(cls, data):
        unpacked = cls.__packet.unpack(data)
        return ControllerConfig(*unpacked)

@dataclass
class EncoderConfig:
    #__fmt = "<ffi128b"
    __packet : ClassVar[struct.Struct] = struct.Struct('<ffi128b')
    offset_elec: float = None
    offset_mech: float = None
    cpr: int = None
    #direction: int = None
    offset_lut: List[int] = None

    def pack(self):
        return self.__packet.pack(self.offset_elec,
        self.offset_mech,
        self.cpr,
        *self.offset_lut)

    @classmethod
    def unpack(cls, data):
        unpacked = cls.__packet.unpack(data)
        return EncoderConfig(unpacked[0], unpacked[1], unpacked[2], [*unpacked[3:]])

@dataclass
class MotorState:
    #__fmt = "<16fI"
    __packet : ClassVar[struct.Struct] = struct.Struct('<9f')
    I_a: float = None
    I_b: float = None
    I_c: float = None
    #V_a: float = None
    #V_b: float = None
    #V_c: float = None
    theta_mech: float = None
    theta_mech_dot: float = None
    theta_mech_true: float = None
    theta_elec: float = None
    theta_elec_dot: float = None
    windings_temp: float = None


    def pack(self):
        return self.__packet.pack(self.I_a,
        self.I_b,
        self.I_c,
        #self.V_a,
        #self.V_b,
        #self.V_c,
        self.theta_mech,
        self.theta_mech_dot,
        self.theta_mech_true,
        self.theta_elec,
        self.theta_elec_dot,
        self.windings_temp)

    @classmethod
    def unpack(cls, data):
        unpacked = cls.__packet.unpack(data)
        return MotorState(*unpacked)

@dataclass
class ControllerState:
    __packet : ClassVar[struct.Struct] = struct.Struct('<23fI')
    I_d: float = None
    I_q: float = None
    I_d_ref: float = None
    I_q_ref: float = None
    d_int: float = None
    q_int: float = None
    V_d: float = None
    V_q: float = None
    V_d_ref: float = None
    V_q_ref: float = None
    Voltage_bus: float = None
    Pos_ref: float = None
    Vel_ref: float = None
    K_p: float = None
    K_d: float = None
    T_ff: float = None
    dtc_A: float = None
    dtc_B: float = None
    dtc_C: float = None
    I_rms: float = None
    I_max: float = None
    I_bus: float = None
    fet_temp: float = None
    timeout: int = None

    def pack(self):
        return self.__packet.pack(self.I_d,
        self.I_q,
        self.I_c,
        self.I_d_ref,
        self.I_q_ref,
        self.d_int,
        self.q_int,
        self.V_d,
        self.V_q,
        self.V_d_ref,
        self.V_q_ref,
        self.Voltage_bus,
        self.Pos_ref,
        self.Vel_ref,
        self.K_p,
        self.K_d,
        self.T_ff,
        self.dtc_A,
        self.dtc_B,
        self.dtc_C,
        self.I_rms,
        self.I_max,
        self.I_bus,
        self.fet_temp,
        self.timeout)

    @classmethod
    def unpack(cls, data):
        unpacked = cls.__packet.unpack(data)
        return ControllerState(*unpacked)

@dataclass
class EncoderState:
    __packet : ClassVar[struct.Struct] = struct.Struct('<2i5f')
    position_raw: int = None
    num_rotations: int = None
    position_elec: float = None
    position_mech: float = None
    velocity_elec: float = None
    velocity_elec_filt: float = None
    velocity_mech: float = None

    def pack(self):
        return self.__packet.pack(self.position_raw,
        self.num_rotations,
        self.position_elec,
        self.position_mech,
        self.velocity_elec,
        self.velocity_elec_filt,
        self.velocity_mech)

    @classmethod
    def unpack(cls, data):
        unpacked = cls.__packet.unpack(data)
        return EncoderState(*unpacked)

@dataclass
class FloatMeasurement:
    __fmt = "<Bf"
    status: int = None
    measurement: float = None

    @classmethod
    def unpack(cls, data):
        unpacked = struct.unpack(cls.__fmt, data)
        return FloatMeasurement(*unpacked)

@dataclass
class IntMeasurement:
    __fmt = "<BI"
    status: int = None
    measurement: int = None

    @classmethod
    def unpack(cls, data):
        unpacked = struct.unpack(cls.__fmt, data)
        return IntMeasurement(*unpacked)
