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

from LogPrint import LogPrint as logger

# TODO: Unify with Framing Handler
PACKET_SIZE_LIMIT = 256

#Serial Handler Interface
class SerialHandler:
    def __init__(self, port, baud, packet_cb=None):
        self.port = port # Port
        self.baud = baud # Baud Rate
        self.connected = False # Connected Flag
        self.uart = serial.Serial(port, baudrate=baud, timeout=1) # Serial Port
        self.hdlc = HDLCHandler(packet_cb)
        self.close_event = threading.Event()
        #self.packet_cb = packet_cb
        self.start_read_task()

    def close(self):
        # Close Port
        self.close_event.set()
        self.uart.close()

    def send_packet(self, packet):

        #print(f'Sending Packet: {packet} with length: {len(packet)}')
        if (len(packet) >= PACKET_SIZE_LIMIT):
            raise NotImplementedError(f"Can not send packet size {len(packet)} > {PACKET_SIZE_LIMIT}")

        hdlc_frame = self.hdlc.frame_packet(packet)
        self.uart.write(hdlc_frame)
        print(hdlc_frame)

    def start_read_task(self):

        def recv_packet():
            while(not self.close_event.is_set()):   
                if (self.uart.inWaiting() > 0): # Got Serial Data
                    byte = self.uart.read(1)
                    print(byte)
                    self.hdlc.process_byte(byte) # Send to HDLC to process packet
            logger.print_info("Serial Receive Thread Terminated")
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
