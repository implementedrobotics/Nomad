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
#include "SerialHandler.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "motor_controller_interface.h"


// HDLC Handler    
HDLCHandler hdlc;

// Buffer Queue
Queue<uint8_t, 10> byte_queue_;


// Comms Event Loops
void comms_thread_entry()
{
    while (true)
    {
        osEvent evt = byte_queue_.get();
        if (evt.status != osEventMessage) {
            printf("queue->get() returned %02x status\n\r", evt.status);
        } else {
            hdlc.ProcessByte(evt.value.v);
        }
    }
}

// Serial Handler
SerialHandler::SerialHandler(Serial *uart)
{
    serial_ = uart;                        // UART Handler
    serial_->attach(callback(this, &SerialHandler::Interrupt)); // Attach Serial Interrupt
}

Serial *SerialHandler::serial_ = 0;

void SerialHandler::Interrupt()
{
    while (serial_->readable())
    {
        //hdlc_.ProcessByte(serial_->getc());
        byte_queue_.put((uint8_t*)serial_->getc());
    }
}