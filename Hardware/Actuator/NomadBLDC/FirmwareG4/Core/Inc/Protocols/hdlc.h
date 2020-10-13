/*
 * hdlc.h
 *
 *  Created on: September 15, 2020
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

#ifndef CORE_PROTOCOLS_HDLC_H_
#define CORE_PROTOCOLS_HDLC_H_

// C System Files
#include <stdlib.h>
#include <stdint.h>

// C++ System Files
#include <functional>

// STM32 System Files
#include <main.h> 

// Project Includes

class HDLCFrame
{

public:

    // Const Statics
    static const uint16_t kFrameBoundary = 0x7E;
    static const uint16_t kControlEscape = 0x7D;
    static const uint16_t kEscapeInvert = 0x20;
    static const uint16_t kPacketSizeLimit = 256;

    // Constructor
    HDLCFrame();

    // Encode/Decode Functions
    uint16_t Encode(const uint8_t *data, uint16_t length, uint8_t *buffer_out);
    bool Decode(const uint8_t *data, uint16_t length);

    // Frame Handler Callback Registration
    void RegisterFrameHandler(const std::function<void(const uint8_t *, size_t)> &handler);

protected:
    uint16_t frame_offset_;               // Store location to where we are in the frame buffer
    uint16_t frame_chksum_;               // Checksum value for frame
    uint8_t tx_buffer_[kPacketSizeLimit]; // Frame buffer.
    uint8_t rx_buffer_[kPacketSizeLimit]; // Frame buffer.
    uint8_t in_escape_;                   // Are we currently in escape?

    // HDLC Frame Command Callback
    std::function<void(const uint8_t *, size_t)> frame_handler_ = [=](const uint8_t *, size_t) {};
};
#endif // CORE_PROTOCOLS_HDLC_H_
