/*
 * CommandHandler.cpp
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

// Primary Include
#include "CommandHandler.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "Core/nomad_common.h"


HDLCHandler hdlc_out;

// Command Handler Class
CommandHandler::CommandHandler()
{
}

void CommandHandler::ProcessPacket(const uint8_t *packet_buffer, uint16_t packet_length)
{
    //printf("PROCESS PACKET: %d \n\r", packet_buffer[0]);

    command_t command = static_cast<command_t>(packet_buffer[0]);
    switch(command)
    {
        case COMM_FW_VERSION:
            //printf("READ FIRMWARE!\n\r");
            uint8_t response[4];
            response[0] = COMM_FW_VERSION;
            response[1] = 2;
            response[2] = VERSION_MAJOR;
            response[3] = VERSION_MINOR;
            hdlc_out.SendPacket(response, 4);
            break;
        default:
            break;
    }

}
