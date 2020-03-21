"""HDLC Handler for Interaction with NomadBLDC Board"""

#  HDLCHandler.py
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

import serial

import time
import random
import struct

from crc import crc16

FRAME_BOUNDARY = 0x7E
CONTROL_ESCAPE = 0x7D
PACKET_SIZE_LIMIT = 256

#Serial Handler Interface
class HDLCHandler:
    def __init__(self):
        self.frame_offset = 0
        self.frame_chksum = 0
        self.receive_buffer = bytearray(10)
        self.in_escape = False

    def frame_packet(self, packet):
         # Compute CRC16
        crc = bytearray(struct.pack("<H", crc16(packet)))

        # Format Packet for Transport
        hdlc_frame = bytearray()
        hdlc_frame.append(FRAME_BOUNDARY)
        hdlc_frame += packet
        hdlc_frame += crc
        hdlc_frame.append(FRAME_BOUNDARY)
        return hdlc_frame

    def process_byte(self, byte: bytes):
        
        if (byte[0] == FRAME_BOUNDARY):
            #print("GOT FRAME BOUNDARY!")
            # Check for End Frame + Validity
            if(self.frame_offset >= 2):
               # print(f'Got Command {self.receive_buffer[0]} and {self.receive_buffer[1]}')
                if ((self.frame_offset - 4) == self.receive_buffer[1]):
                    # Length matches.  Now verify checksum
                    #self.frame_chksum = bytearray(struct.pack("<H", crc16(self.receive_buffer[0:self.frame_offset-2])))
                    self.frame_chksum = crc16(self.receive_buffer[0:self.frame_offset-2])
                    #print(self.frame_chksum)
                    #print(self.receive_buffer[self.frame_offset-2:self.frame_offset])
                    sent_chksum = struct.unpack("<H", self.receive_buffer[self.frame_offset-2:self.frame_offset])[0]
                    #print("Successful Packet Receive")
                    #print(self.receive_buffer[0:self.frame_offset-2])
                    #print(sent_chksum)
                    if(self.frame_chksum == sent_chksum):
                        print("COMMAND SUCCESS")

            # Reset and look for next frame
            self.frame_offset = 0
            self.frame_chksum = 0
            return

        # TODO: Escape Control Sequences
        # Copy to buffer
        self.receive_buffer[self.frame_offset] = byte[0]
        self.frame_offset += 1

        if (self.frame_offset >= PACKET_SIZE_LIMIT): # Overflow Packet Limit,
            self.frame_offset = 0 # Reset
            self.frame_chksum = 0
        #print(self.receive_buffer)


