/*
 * hdlc.cpp
 *
 *  Created on: OCtober 13, 2020
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
#include <Protocols/hdlc.h>


// C++ Includes
#include <functional>

// C System Files
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Project Includes
#include <Utilities/crc16.h>
#include <Logger.h>
#include <main.h> // STM32 Driver Includes

    
HDLCFrame::HDLCFrame() : frame_offset_(0), in_escape_(0)
{
}

void HDLCFrame::RegisterFrameHandler(const std::function<void(const uint8_t*, size_t)>& handler)
{
    frame_handler_ = handler;
}
uint16_t HDLCFrame::Encode(const uint8_t *packet, uint16_t length, uint8_t* buffer_out)
{
    if (length >= kPacketSizeLimit)
    {
        return 0;
    }

    // Compute CRC16 for Packet
    frame_chksum_ = crc16_compute(packet, length);

    uint32_t buffer_offset = 0;
    buffer_out[buffer_offset++] = kFrameBoundary;

    // Process and Escape Packet
    for (uint32_t i = 0; i < length; i++)
    {
        uint8_t data = packet[i];
        if ((data == kFrameBoundary) || (data == kControlEscape))
        {
            buffer_out[buffer_offset++] = kControlEscape;
            buffer_out[buffer_offset++] = data ^ kEscapeInvert;
        }
        else // Not Escaped
        {
            buffer_out[buffer_offset++] = data;
        }
    }

    // Copy in CRC16
    memcpy(buffer_out + buffer_offset, (uint8_t *)(&frame_chksum_), sizeof(uint16_t));

    // Add Frame Boundary
    buffer_offset += 2;
    buffer_out[buffer_offset++] = kFrameBoundary;

    return buffer_offset;
}

bool HDLCFrame::Decode(const uint8_t *data, uint16_t length)
{
    
    bool found_frame = false;
    for (; length > 0; --length, ++data)
    {
        uint8_t byte = *data;
        if (byte == kFrameBoundary && !in_escape_)
        {
            // Check for End Frame + Validity
            if (frame_offset_ >= 2) // Need atleast 3 bytes for a valid frame, (BEGIN, CMD, LENGTH)
            {
                // Command = receive_buffer_[0]
                // Payload Length = receive_buffer_[1]
                // Fast early out on packet length
                if ((frame_offset_ - 4) == rx_buffer_[1])
                {
                    // Length matches now verify checksum
                    uint16_t sent_chksum = (rx_buffer_[frame_offset_ - 1] << 8) | (rx_buffer_[frame_offset_ - 2] & 0xff);
                    frame_chksum_ = crc16_compute(rx_buffer_, frame_offset_ - 2);
                    if (frame_chksum_ == sent_chksum)
                    {
                        // Execute Command Callback
                        frame_handler_(rx_buffer_, frame_offset_ - 2);
                        found_frame = true;
                    }
                }
                // TODO: If invalid do we add support for an ack?
            }
            // Reset and look for next Frame
            frame_offset_ = 0;
            frame_chksum_ = 0;
            //return found_frame;
            continue;
        }

        // Handle Escape Sequences
        if (in_escape_)
        {
            byte ^= kEscapeInvert;
            in_escape_ = 0;
        }
        else if (byte == kControlEscape)
        {
            in_escape_ = 1;
            //return found_frame; // Return here to read real byte on next run
            continue;
        }

        // Copy to buffer
        rx_buffer_[frame_offset_++] = byte;

        if (frame_offset_ >= kPacketSizeLimit) // Overflow Packet Limit,
        {
            frame_offset_ = 0; // Reset
            frame_chksum_ = 0;
        }
    }
    return found_frame;
}
