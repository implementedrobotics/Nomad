/*
 * HDLCHandler.h
 *
 *  Created on: March 20, 2020
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

#ifndef CORE_HDLC_HANDLER_H_
#define CORE_HDLC_HANDLER_H_

// C System Files

// C++ System Files
#include <string>
#include <vector>

// Project Includes
#include "mbed.h"

class HDLCHandler
{
private:
    uint16_t frame_offset_;
    uint16_t frame_chksum_;
    uint8_t receive_buffer_[512]; // Frame buffer.  Support 255
    uint8_t transmit_buffer_ [512]; // Frame buffer out
    bool in_escape_; // Are we currently in escape?

public:
    HDLCHandler();
    void ProcessByte(uint8_t byte);
    bool SendPacket(uint8_t *packet, uint32_t length);

};

#endif // CORE_SERIAL_HANDLER_H_