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
import serial
import serialstruct
import time
import random
import struct
# Class Wrapper to interface with NomadBLDC control board
# TODO Autobaud?
class NomadBLDC:
    def __init__(self, timeout=0.5, baud=921600):
        self.timeout = 0.5 # Timeout
        self.baud = baud # Baud Rate
        self.connected = False # Connected Flag
        self.sp = None # Serial Port

    # Connect to Nomad.  Try to Autofind port
    def connect(self):

        # Get Available Serial Ports
        ports = self.__get_available_ports()
        
        if ports.count == 0: # No Ports, Bail
            return False

        # Loop found ports and try to connect to Nomad 
        for port in ports:
            print(f'Connecting to port: {port} [{self.baud} baud]')
            self.sp = serial.Serial(port, baudrate=self.baud, timeout=self.timeout, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
            print(self.sp.name)
            print(self.sp.is_open)

            if(not self.sp.is_open): # Could not open.  Skip and Continue
                continue

            # Send Connect Command
            CONNECT_STRUCT = struct.Struct("<Ib")
            RESPONSE_STRUCT = struct.Struct("!B")
            packet = CONNECT_STRUCT.pack(1, 0x01)
            HEADER = b'\x01\x02\x03\x04\x05'
            print(RESPONSE_STRUCT.size)
            self.sp.write(HEADER)
            self.sp.write(packet)
            time.sleep(1)
            response = self.sp.read(RESPONSE_STRUCT.size)
            response = RESPONSE_STRUCT.unpack(response)[0]
            if(response == 0x02):
                print("CONNECTED!")
                self.connected = True
            else:
                self.connected = False
            return self.connected
        return True
        
    # Disconnect Port
    def disconnect(self):
        if self.sp.is_open:
            self.sp.close()

        time.sleep(0.5) # Give it some time to shutdown clean 
        self.connected = False
        return not self.sp.is_open

    def __get_available_ports(self):
        """ Lists serial port names

            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'): # WINDOWS
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'): #LINUX
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'): #MAC
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result




#nomad = NomadBLDC(timeout=0.5, baud=921600)
#nomad.connect()
#print(nomad.disconnect())
