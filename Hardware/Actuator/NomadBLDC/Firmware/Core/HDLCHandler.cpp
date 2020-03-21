/*
 * SerialHandler.cpp
 *
 *  Created on: March 19, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

// Primary Include
#include "HDLCHandler.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "CommandHandler.h"
#include "CRC16.h"

// HDLC Asynchronous Framing
#define FRAME_BOUNDARY 0x7E
#define CONTROL_ESCAPE 0X7D
#define ESCAPE_INVERT 0X20

#define PACKET_SIZE_LIMIT 256

// HDLC Handler Class
HDLCHandler::HDLCHandler() : frame_offset_(0), in_escape_(false)
{
}

void HDLCHandler::ProcessByte(uint8_t byte)
{
    if (byte == FRAME_BOUNDARY)
    {
        // Check for End Frame + Validity
        if (frame_offset_ >= 2) // Need atleast 3 bytes for a valid frame, (BEGIN, CMD, LENGTH)
        {
            // Command = receive_buffer_[0]
            // Payload Length = receive_buffer_[1]
            printf("Got Packet: Command %d.  Length %d, Offset %d\n\r", receive_buffer_[0], receive_buffer_[1], frame_offset_-4);

            // Fast early out on packet length
            if ((frame_offset_ - 4) == receive_buffer_[1])
            {
                // Length matches now verify checksum
                uint16_t sent_chksum = (receive_buffer_[frame_offset_ - 1] << 8) | (receive_buffer_[frame_offset_ - 2] & 0xff);
                frame_chksum_ = CRC16::Compute(receive_buffer_, frame_offset_ - 2);
                if (frame_chksum_ == sent_chksum)
                {
                    // Execute Command Callback
                    printf("SUCCESSFUL PACKET SEND!\r\n");
                    CommandHandler::ProcessPacket(receive_buffer_, frame_offset_-2);
                }
            }

            // TODO: If invalid do we add support for an ack?
        }

        // Reset and look for next Frame
        frame_offset_ = 0;
        frame_chksum_ = 0;
        return;
    }

    // Handle Escape Sequences
    if (in_escape_)
    {
        byte ^= ESCAPE_INVERT;
        in_escape_ = false;
    }
    else if (byte == CONTROL_ESCAPE)
    {
        in_escape_ = true;
        return; // Return here to read real byte on next run
    }

    // Copy to buffer
    receive_buffer_[frame_offset_++] = byte;

    //frame_offset_++;

    if (frame_offset_ >= PACKET_SIZE_LIMIT) // Overflow Packet Limit,
    {
        frame_offset_ = 0; // Reset
        frame_chksum_ = 0;
    }
}
