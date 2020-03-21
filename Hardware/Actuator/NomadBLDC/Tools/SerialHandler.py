"""Serial Handler for Interaction with NomadBLDC Board"""

#  SerialHandler.py
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


import sys
import glob
import serial

import time
import random
import struct
import threading

from crc import crc16
from HDLCHandler import HDLCHandler

FRAME_BOUNDARY = 0x7E
CONTROL_ESCAPE = 0x7D
PACKET_SIZE_LIMIT = 256

#Serial Handler Interface
class SerialHandler:
    def __init__(self, port, baud=115200):
        self.port = port # Port
        self.baud = baud # Baud Rate
        self.connected = False # Connected Flag
        self.uart = serial.Serial(port, baudrate=baud, timeout=1) # Serial Port
        self.hdlc = HDLCHandler()
        self.start_recv_thread()
    def close(self):
        # Close Port
        self.uart.close()

    def send_packet(self, packet):

        #print(f'Sending Packet: {packet} with length: {len(packet)}')
        if (len(packet) >= PACKET_SIZE_LIMIT):
            raise NotImplementedError(f"Can not send packet size {len(packet)} > {PACKET_SIZE_LIMIT}")

        # Compute CRC16
        crc = bytearray(struct.pack("<H", crc16(packet)))

        # Format Packet for Transport
        hdlc_frame = bytearray()
        hdlc_frame.append(FRAME_BOUNDARY)
        hdlc_frame += packet
        hdlc_frame += crc
        hdlc_frame.append(FRAME_BOUNDARY)
        self.uart.write(hdlc_frame)
        #print(hdlc_frame)

    def start_recv_thread(self):
        def recv_packet():
            while(True):   
                if (self.uart.inWaiting()>0):
                    byte = self.uart.read(1)
                    #print(byte)
                    self.hdlc.process_byte(byte)
        
        t = threading.Thread(target=recv_packet)
        t.start()


def get_available_ports():
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
