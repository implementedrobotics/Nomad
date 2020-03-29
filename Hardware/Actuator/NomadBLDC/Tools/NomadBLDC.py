"""Nomad Class Handler for Interaction with NomadBLDC Board"""

#  NomadBLDC.py
#
#  Created on: March 18, 2020
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


import sys
import glob
import time
import struct
from SerialHandler import SerialHandler
from Commands import *
from DataContainers import *
from LogPrint import LogPrint as logger

# Class Wrapper to interface with NomadBLDC control board
class NomadBLDC:
    def __init__(self):
        self.transport = None # Serial Transport
        self.commands = CommandHandler()
        self.connected = False
        self.port = None
        self.device_info = None
        self.motor_config = MotorConfig()
        self.controller_config = None
        self.encoder_config = None



    def measure_motor_phase_order(self):
        if(not self.connected):
            return False
        
        return self.commands.measure_motor_phase_order(self.transport)  

    def measure_motor_resistance(self):
        if(not self.connected):
            return False
        
        return self.commands.measure_motor_resistance(self.transport)   
    
    def measure_motor_inductance(self):
        if(not self.connected):
            return False
        
        return self.commands.measure_motor_inductance(self.transport)   

    def load_configuration(self):
        if(not self.connected):
            return False

        self.motor_config = self.commands.load_configuration(self.transport)
        if(self.motor_config is None):
            return False
        return True

    def calibrate_motor(self):
        if(not self.connected):
            return False
        
        return self.commands.calibrate_motor(self.transport)

    def restart_device(self):
        if(not self.connected):
            return False
        
        return self.commands.restart_device(self.transport)

    def start_voltage_control(self):
        if(not self.connected):
            return False

        return self.commands.start_voltage_control(self.transport)

    def start_torque_control(self):
        if(not self.connected):
            return False

        return self.commands.start_torque_control(self.transport)

    def enter_idle_mode(self):
        if(not self.connected):
            return False

        return self.commands.enter_idle_mode(self.transport)

    def set_voltage_setpoint(self, v_d, v_q):
        if(not self.connected):
            return False
        return self.commands.set_voltage_setpoint(self.transport, v_d, v_q)

    def set_torque_setpoint(self, k_p, k_d, pos, vel, tau_ff):
        if(not self.connected):
            return False
        return self.commands.set_torque_setpoint(self.transport, k_p, k_d, pos, vel, tau_ff)

    def get_device_stats(self):
        if(not self.connected):
            return False
        return self.commands.get_device_stats(self.transport)

    # Connect to Nomad Board
    def connect(self):

        baud = 115200 # TODO: Auto or from UI
        ports = SerialHandler.get_available_ports() # Get available ports
        
        for self.port in ports: # Loop ports and try to read firmware version
            logger.print_info(f'Connecting to port: {self.port} [{baud} baud]')
            self.transport = SerialHandler(self.port, baud, self.commands.process_packet)
            self.device_info = self.commands.read_device_info(self.transport)
            if(self.device_info  is None): # No device infor received.  Continue to next.
                self.transport.close() # Close transport
                continue

            logger.print_pass(f"Connected to NomadBLDC[ID: {self.device_info .device_id}] running Firmware v{self.device_info .fw_major}.{self.device_info .fw_minor} on {self.port}")
            self.connected = True
            return True

        logger.print_fail("Failed to find a Nomad BLDC Controller")
        return False # Failed to connect

    def disconnect(self):
        if(self.connected):
            self.transport.close()
            self.connected = False


# # TODO: Verify compatible firmware
#nomad = NomadBLDC()
#nomad.test()
# if(nomad.connect()):
#     logger.print("Nomad Connected")
# else:
#     logger.print_fail("Failed to find Nomad Controller")

# time.sleep(1.0)
# nomad.disconnect()
#nomad.connect()
#print(nomad.disconnect())
