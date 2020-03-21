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
from SerialHandler import get_available_ports
from Commands import CommandHandler

# Class Wrapper to interface with NomadBLDC control board
class NomadBLDC:
    def __init__(self):
        self.transport = None # Serial Transport
        self.commands = CommandHandler()

    # Connect to Nomad Board
    def connect(self):

        baud = 115200 # TODO: Auto or from UI
        ports = get_available_ports() # Get available ports
        
        for port in ports: # Loop ports and try to read firmware version
            print(f'Connecting to port: {port} [{baud} baud]')
            self.transport = SerialHandler(port, baud)
            self.commands.read_firmware_version(self.transport)



nomad = NomadBLDC()
nomad.connect()

#nomad.connect()
#print(nomad.disconnect())
