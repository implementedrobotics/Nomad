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

struct Device_info_t
{
    uint8_t comm_id;            // Command ID
    uint8_t packet_length;     // Packet Length
    uint8_t fw_major;          // Firmware Version Major
    uint8_t fw_minor;          // Firmware Version Minor
    uint32_t uid1;             // Device Unique ID 1
    uint32_t uid2;             // Device Unique ID 2
    uint32_t uid3;             // Device Unique ID 3
};
// Command Handler Class
CommandHandler::CommandHandler()
{
}

void CommandHandler::ProcessPacket(const uint8_t *packet_buffer, uint16_t packet_length)
{
    //printf("PROCESS PACKET: %d \n\r", packet_buffer[0]);

    command_t command = static_cast<command_t>(packet_buffer[0]);
    switch (command)
    {
    case COMM_DEVICE_INFO:
    {
        //printf("READ FIRMWARE!\n\r");

        // Get Unique Device ID Offset Register
        unsigned long *uid = (unsigned long *)0x1FFF7A10;
        Device_info_t info;

        info.comm_id = COMM_DEVICE_INFO;
        info.packet_length = sizeof(Device_info_t) - 2; // First 2 bytes don't count for packet length[CommandID/PacketLength]
        info.fw_major = VERSION_MAJOR;
        info.fw_minor = VERSION_MINOR;
        info.uid1 = uid[0];
        info.uid2 = uid[1];
        info.uid3 = uid[2];

        // Send it
        hdlc_out.SendPacket((uint8_t *)&info, sizeof(Device_info_t));
        break;
    }
    default:
        break;
    }
}
