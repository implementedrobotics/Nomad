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
import math
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
        self.encoder_config = EncoderConfig()
        self.controller_config = ControllerConfig()

        self.motor_state = MotorState()
        self.controller_state = ControllerState()
        self.encoder_state = EncoderState()



    def zero_mechanical_offset(self):
        if(not self.connected):
            return False
        return self.commands.zero_mechanical_offset(self.transport)    

    def measure_motor_encoder_offset(self):
        if(not self.connected):
            return False
        return self.commands.measure_motor_encoder_offset(self.transport)

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

    def save_configuration(self):
        if(not self.connected):
            return False

        return self.commands.save_configuration(self.transport, self.motor_config, self.controller_config, self.encoder_config)
        # TODO: Error check here?

    def load_configuration(self):
        if(not self.connected):
            return False

        load_config = self.commands.load_configuration(self.transport)
        self.motor_config = load_config[0]
        self.controller_config = load_config[1]
        self.encoder_config = load_config[2]

        if(self.motor_config is None or self.encoder_config is None or self.controller_config is None):
            return False
        return True
    
    def read_state(self):
        if(not self.connected):
            return False

        state_config = self.commands.read_state(self.transport)
        if(state_config[0] is not None):
            self.motor_state = state_config[0]
            #self.motor_state.I_a = 10

        if(state_config[1] is not None):
            self.controller_state = state_config[1]
        
        if(state_config[2] is not None):
            self.encoder_state = state_config[2]


        #self.controller_state = state_config[1]
        #self.encoder_state = state_config[2]

        if(self.motor_state is None or self.controller_state is None or self.encoder_state is None):
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

    def start_current_control(self):
        if(not self.connected):
            return False
        return self.commands.start_current_control(self.transport)

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

    def set_current_setpoint(self, i_d, i_q):
        if(not self.connected):
            return False
        return self.commands.set_current_setpoint(self.transport, i_d, i_q)

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
            logger.print_info(f'Info: {self.device_info}')
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

    def dq0(self, theta, I_a, I_b, I_c):
        cf = math.cos(theta)
        sf = math.sin(theta)

        d = 0.6666667 * (cf * I_a + (0.86602540378 * sf - 0.5 * cf) * I_b + (-0.86602540378 * sf - 0.5 * cf) * I_c)
        q = 0.6666667 * (-sf * I_a - (-0.86602540378 * cf - 0.5 * sf) * I_b - (0.86602540378 * cf - 0.5 * sf) * I_c)
        return (d, q)




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
