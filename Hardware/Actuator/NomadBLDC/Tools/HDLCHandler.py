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
HDLC_BYTE_INVERT = 0x20
PACKET_SIZE_LIMIT = 256

#Serial Handler Interface
class HDLCHandler:
    def __init__(self, packet_cb):
        self.frame_offset = 0
        self.frame_chksum = 0
        self.receive_buffer = bytearray(5048)
        self.in_escape = False
        self.packet_cb = packet_cb

    def frame_packet(self, packet: bytearray):
        # Must use control escape sequence for the reserved bytes
        escaped_packet = bytearray()
        for byte in packet:
            if(byte == FRAME_BOUNDARY or byte == CONTROL_ESCAPE):
                #print("Escaping Packet!")
                escaped_packet.append(CONTROL_ESCAPE)
                escaped_packet.append(byte ^ HDLC_BYTE_INVERT)
            else:
                escaped_packet.append(byte)
         #   print(f"Byte: {hex(byte)}")
        #print("OUT")

         # Compute CRC16
        crc = bytearray(struct.pack("<H", crc16(packet)))

        # Format Packet for Transport
        hdlc_frame = bytearray()
        hdlc_frame.append(FRAME_BOUNDARY)
        hdlc_frame += escaped_packet
        hdlc_frame += crc
        hdlc_frame.append(FRAME_BOUNDARY)

        #print(len(hdlc_frame))
        return hdlc_frame

    def process_byte(self, byte_in: bytes):
        byte = byte_in[0]
        if (byte == FRAME_BOUNDARY and not self.in_escape):
           # print("GOT FRAME BOUNDARY!")
            # Check for End Frame + Validity
            if(self.frame_offset >= 2):
            #    print(f'Got Command {self.receive_buffer[0]} and {self.receive_buffer[1]}')
             #   print((self.frame_offset - 4))
              #  print(self.receive_buffer[1])
                if ((self.frame_offset - 4) == self.receive_buffer[1]):
                    # Length matches.  Now verify checksum
                    self.frame_chksum = bytearray(struct.pack("<H", crc16(self.receive_buffer[0:self.frame_offset-2])))
                    #print(f"Check Sum on {self.receive_buffer[0:self.frame_offset-2]}")
                    packet = self.receive_buffer[0:self.frame_offset-2]
                    self.frame_chksum = crc16(packet)
                    #print(self.frame_chksum)
                    #print(self.receive_buffer[self.frame_offset-2:self.frame_offset])
                    sent_chksum = struct.unpack("<H", self.receive_buffer[self.frame_offset-2:self.frame_offset])[0]
                    #print("Successful Packet Receive")
                    #print(self.receive_buffer[0:self.frame_offset-2])
                    #print(sent_chksum)
                    if(self.frame_chksum == sent_chksum and self.packet_cb is not None):
                        self.packet_cb(packet)
                        #print(f"Got packet SUCCESS: {packet}")
                        #print("COMMAND SUCCESS")

            # Reset and look for next frame
            self.frame_offset = 0
            self.frame_chksum = 0
            return

        # Handle Escape Sequence
        if(self.in_escape is True):
            byte = byte ^ HDLC_BYTE_INVERT
            self.in_escape = False
        elif(byte == CONTROL_ESCAPE):
            self.in_escape = True
            return

        # Copy to buffer
        self.receive_buffer[self.frame_offset] = byte
        self.frame_offset += 1

        if (self.frame_offset >= PACKET_SIZE_LIMIT): # Overflow Packet Limit,
            self.frame_offset = 0 # Reset
            self.frame_chksum = 0
        #print(self.receive_buffer)


