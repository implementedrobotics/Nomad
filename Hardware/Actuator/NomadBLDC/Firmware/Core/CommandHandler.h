/*
 * CommandHandler.h
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

#ifndef COMMAND_HANDLER_H_
#define COMMAND_HANDLER_H_

// C System Files

// C++ System Files
#include <string>
#include <vector>

// Project Includes
#include "mbed.h"
#include "HDLCHandler.h"


class CommandHandler
{

public:

    typedef enum
    {
        // Info/Status Commands
        COMM_DEVICE_INFO = 1,

        // Configuration Commands
        COMM_CALIB_MOTOR = 2,
        COMM_ZERO_POSITION = 3,

        // Motor Control Modes
        COMM_ENABLE_CURRENT_CONTROL = 4,
        COMM_ENABLE_VOLTAGE_CONTROL = 5,
        COMM_ENABLE_TORQUE_CONTROL = 6,
        COMM_ENABLE_SPEED_CONTROL = 7,
        COMM_ENABLE_IDLE_MODE = 8,

        // Device Control
        COMM_DEVICE_RESTART = 9,
        COMM_DEVICE_ABORT = 10,

        // Set points
        COMM_VOLTAGE_SETPOINT = 11,
        COMM_TORQUE_SETPOINT = 12,
    } command_t;

    CommandHandler();
    static void ProcessPacket(const uint8_t *packet_buffer, uint16_t packet_length);

private:


};

#endif // COMMAND_HANDLER_H_