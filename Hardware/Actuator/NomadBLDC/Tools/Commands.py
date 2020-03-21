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
from dataclasses import dataclass

#from collections import namedtuple
#from typing import NamedTuple

@dataclass
class DeviceInfo:
    fw_major: int = None
    fw_minor: int = None
    device_id: 'typing.Any' = None

class CommandID(IntEnum):
    DEVICE_INFO_READ = 1
    
class CommandHandler:
    def __init__(self):
        self.timeout = 10

        # Events
        self.device_info_received = threading.Event()

    
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

        # Unpack packet
        # Find


