/*
 * SerialHandler.h
 *
 *  Created on: March 18, 2020
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

#ifndef CORE_SERIAL_HANDLER_H_
#define CORE_SERIAL_HANDLER_H_

// C System Files

// C++ System Files
#include <string>
#include <vector>

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "HDLCHandler.h"


// Entry point to facilitate transition to C++ for RTOS Task
void comms_thread_entry();

struct Packet_t
{
    uint32_t length;
    char data[16];
};

struct Command_t
{
    uint32_t command;
};

struct Reponse_t
{
    uint8_t value;
};

class SerialHandler
{

public:
    SerialHandler(Serial *uart);
    Serial* GetSerial() { return serial_; }
    void Interrupt();

private:
    Serial *serial_;
};

#endif // CORE_SERIAL_HANDLER_H_