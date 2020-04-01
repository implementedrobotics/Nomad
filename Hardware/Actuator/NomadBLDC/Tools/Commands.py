"""Nomad Class Handler for Interaction with NomadBLDC Board"""

#  Commands.py
#
#  Created on: March 20, 2020
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


from enum import IntEnum
import struct
import threading
from DataContainers import *

#from collections import namedtuple
#from typing import NamedTuple

class CommandID(IntEnum):
    # Info/Status Commands
    DEVICE_INFO_READ = 1
    DEVICE_STATS_READ = 2
    DEVICE_CONFIG_READ = 3

    # Configuration Commands
    MEASURE_RESISTANCE = 4
    MEASURE_INDUCTANCE = 5
    MEASURE_PHASE_ORDER = 6
    MEASURE_ENCODER_OFFSET = 7
    ZERO_MECHANICAL_OFFSET = 8

    # Motor Control Commands
    ENABLE_CURRENT_CONTROL = 9
    ENABLE_VOLTAGE_CONTROL = 10
    ENABLE_TORQUE_CONTROL  = 11
    ENABLE_SPEED_CONTROL   = 12
    ENABLE_IDLE_MODE       = 13

    READ_MOTOR_STATE = 14
    READ_CONTROLLER_STATE = 15
    READ_POSITION_STATE = 16
    READ_MOTOR_CONFIG = 17
    READ_CONTROLLER_CONFIG = 18
    READ_ENCODER_CONFIG = 19
    WRITE_MOTOR_CONFIG = 20
    WRITE_CONTROLLER_CONFIG = 21
    WRITE_POSITION_CONFIG = 22
    WRITE_FLASH = 23

    # Device Control Commands
    DEVICE_RESTART = 24
    DEVICE_ABORT   = 25

    
    # Set Points
    SEND_VOLTAGE_SETPOINT = 26
    SEND_TORQUE_SETPOINT = 27

    # Status
    LOGGING_OUTPUT = 100
    CALIBRATE_MOTOR = 101

class CommandHandler:
    def __init__(self):
        self.timeout = 10

        # Events
        self.device_info_received = threading.Event()
        self.device_stats_received = threading.Event()
        self.motor_config_received = threading.Event()
        self.controller_config_received = threading.Event()
        self.encoder_config_received = threading.Event()
        self.resistance_measurement_complete = threading.Event()
        self.inductance_measurement_complete = threading.Event()
        self.phase_order_measurement_complete = threading.Event()
        self.encoder_offset_measurement_complete = threading.Event()
        self.measurement = None
        self.device_stats = None
        self.motor_config = None
        self.encoder_config = None
        self.controller_config = None
        self.logger_cb = None
    
    def set_logger_cb(self, cb):
        self.logger_cb = cb 

    # Mechnical Offset
    def zero_mechanical_offset(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.ZERO_MECHANICAL_OFFSET, 0))
        transport.send_packet(command_packet)

    # Measure Encoder Offset
    def measure_motor_encoder_offset(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.MEASURE_ENCODER_OFFSET, 0))
        transport.send_packet(command_packet)

        # Wait for device response
        self.encoder_offset_measurement_complete.wait(30)
        if(self.encoder_offset_measurement_complete.is_set()):
            self.encoder_offset_measurement_complete.clear() # Clear Flag
            return self.measurement

    # Measure Motor Phase Order
    def measure_motor_phase_order(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.MEASURE_PHASE_ORDER, 0))
        transport.send_packet(command_packet)

        # Wait for device response
        self.phase_order_measurement_complete.wait(20)
        if(self.phase_order_measurement_complete.is_set()):
            self.phase_order_measurement_complete.clear() # Clear Flag
            return self.measurement

        return None

    # Measure Motor Resistance
    def measure_motor_resistance(self, transport, calib_voltage=3, calib_current=15):
        command_packet = bytearray(struct.pack("<BB", CommandID.MEASURE_RESISTANCE, 0))
        transport.send_packet(command_packet)

        # Wait for device response
        self.resistance_measurement_complete.wait(20)
        if(self.resistance_measurement_complete.is_set()):
            self.resistance_measurement_complete.clear() # Clear Flag
            return self.measurement

        return None

    # Measure Motor Inductance
    def measure_motor_inductance(self, transport, calib_voltage=3, calib_current=15):
        command_packet = bytearray(struct.pack("<BB", CommandID.MEASURE_INDUCTANCE, 0))
        transport.send_packet(command_packet)

        # Wait for device response
        self.inductance_measurement_complete.wait(20)
        if(self.inductance_measurement_complete.is_set()):
            self.inductance_measurement_complete.clear() # Clear Flag
            return self.measurement

        return None


    # Save Configuration
    def save_configuration(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.WRITE_FLASH, 0))
        transport.send_packet(command_packet)

    # Load Configuration
    def load_configuration(self, transport):

        ################ Load Motor Config
        command_packet = bytearray(struct.pack("<BB", CommandID.READ_MOTOR_CONFIG, 0))
        transport.send_packet(command_packet)

        self.motor_config = None
        self.controller_config = None
        self.encoder_config = None

        # Wait for device response
        self.motor_config_received.wait(5)
        if(self.motor_config_received.is_set()):
            self.motor_config_received.clear() # Clear Flag
        ####################

        ############### Load Controller Encoder Config
        command_packet = bytearray(struct.pack("<BB", CommandID.READ_CONTROLLER_CONFIG, 0))
        transport.send_packet(command_packet)

        # Wait for device response
        self.controller_config_received.wait(5)
        if(self.controller_config_received.is_set()):
            self.controller_config_received.clear() # Clear Flag
        ###################

        ################ Load Position Encoder Config
        command_packet = bytearray(struct.pack("<BB", CommandID.READ_ENCODER_CONFIG, 0))
        transport.send_packet(command_packet)

        # Wait for device response
        self.encoder_config_received.wait(5)
        if(self.encoder_config_received.is_set()):
            self.encoder_config_received.clear() # Clear Flag
        ###################


        return (self.motor_config, self.controller_config, self.encoder_config)

    # Calibrate motor
    def calibrate_motor(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.CALIBRATE_MOTOR, 0))
        transport.send_packet(command_packet)
        return True
    
    # Restart device
    def restart_device(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.DEVICE_RESTART, 0))
        transport.send_packet(command_packet)
        return True

    # Voltage Control Mode
    def start_voltage_control(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.ENABLE_VOLTAGE_CONTROL, 0))
        transport.send_packet(command_packet)
        return True

    # Torque Control Mode
    def start_torque_control(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.ENABLE_TORQUE_CONTROL, 0))
        transport.send_packet(command_packet)
        return True

    # Enter Idle Mode
    def enter_idle_mode(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.ENABLE_IDLE_MODE, 0))
        transport.send_packet(command_packet)
        return True

    def set_voltage_setpoint(self, transport, v_d, v_q):
        command_packet = bytearray(struct.pack("<BBff", CommandID.SEND_VOLTAGE_SETPOINT, 8, v_d, v_q))
        transport.send_packet(command_packet)
        return True
        
    def set_torque_setpoint(self, transport, k_p, k_d, pos, vel, tau_ff):
        command_packet = bytearray(struct.pack("<BBfffff", CommandID.SEND_TORQUE_SETPOINT, 20, k_p, k_d, pos, vel, tau_ff))
        transport.send_packet(command_packet)
        return True

    # Device Stats
    def get_device_stats(self, transport):

        command_packet = bytearray(struct.pack("<BB", CommandID.DEVICE_STATS_READ, 0))
        transport.send_packet(command_packet)

        # Wait for device response
        self.device_stats_received.wait(5)
        if(self.device_stats_received.is_set()):
            self.device_stats_received.clear() # Clear Flag
            return self.device_stats

        return None

    # Read device info.  Blocking.
    def read_device_info(self, transport):
        command_packet = bytearray(struct.pack("<BB", CommandID.DEVICE_INFO_READ, 0))
        transport.send_packet(command_packet)

        # Wait for device response
        self.device_info_received.wait(5)
        if(self.device_info_received.is_set()):
            self.device_info_received.clear() # Clear Flag
            return self.device_info
        
        return None

    def process_packet(self, packet):
        comm_id = packet[0]
        if(comm_id == CommandID.DEVICE_INFO_READ):
            self.device_info = DeviceInfo()
            self.device_info.fw_major = packet[2]
            self.device_info.fw_minor = packet[3]
            self.device_info.device_id = packet[4:16].hex()
            self.device_info_received.set()

        elif(comm_id == CommandID.DEVICE_STATS_READ):
            self.device_stats = DeviceStats.unpack(packet[2:])
            self.device_stats_received.set()

        elif(comm_id == CommandID.LOGGING_OUTPUT):
            if(self.logger_cb is not None):
                log_string = struct.unpack(f"<{packet[1]}s", packet[2:])[0].decode("utf-8")
                self.logger_cb(log_string)

        elif(comm_id == CommandID.READ_MOTOR_CONFIG):
            self.motor_config = MotorConfig.unpack(packet[2:])
            self.motor_config_received.set()
        
        elif(comm_id == CommandID.READ_ENCODER_CONFIG):
            self.encoder_config = EncoderConfig.unpack(packet[2:])
            self.encoder_config_received.set()
        
        elif(comm_id == CommandID.READ_CONTROLLER_CONFIG):
            self.controller_config = ControllerConfig.unpack(packet[2:])
            self.controller_config_received.set()
            #print(self.controller_config)

        elif(comm_id == CommandID.MEASURE_RESISTANCE):
            self.measurement = FloatMeasurement.unpack(packet[2:])
            self.resistance_measurement_complete.set()

        elif(comm_id == CommandID.MEASURE_INDUCTANCE):
            self.measurement = FloatMeasurement.unpack(packet[2:])
            self.inductance_measurement_complete.set()

        elif(comm_id == CommandID.MEASURE_PHASE_ORDER):
            self.measurement = IntMeasurement.unpack(packet[2:])
            self.phase_order_measurement_complete.set()

        elif(comm_id == CommandID.MEASURE_ENCODER_OFFSET):
            self.measurement = FloatMeasurement.unpack(packet[2:])
            self.encoder_offset_measurement_complete.set()




