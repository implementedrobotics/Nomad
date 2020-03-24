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
#include "motor_controller_interface.h"
#include "Core/nomad_common.h"

HDLCHandler hdlc_out;
#define PACKET_DATA_OFFSET 2
struct Device_info_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    uint8_t fw_major;          // Firmware Version Major
    uint8_t fw_minor;          // Firmware Version Minor
    uint32_t uid1;             // Device Unique ID 1
    uint32_t uid2;             // Device Unique ID 2
    uint32_t uid3;             // Device Unique ID 3
};


struct Motor_setpoint_t
{
    float v_d;                 // V_d Setpoint
    float v_q;                 // V_q Setpoint
};

struct Motor_torque_setpoint_t
{
    float k_p;                 // V_d Setpoint
    //float k_d;                 // V_q Setpoint
    float pos;
    //float vel;
    //float tau_ff;
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
        info.packet_length = sizeof(Device_info_t) - PACKET_DATA_OFFSET; // First 2 bytes don't count for packet length[CommandID/PacketLength]
        info.fw_major = VERSION_MAJOR;
        info.fw_minor = VERSION_MINOR;
        info.uid1 = uid[0];
        info.uid2 = uid[1];
        info.uid3 = uid[2];

        // Send it
        hdlc_out.SendPacket((uint8_t *)&info, sizeof(Device_info_t));
        break;
    }
    case COMM_CALIB_MOTOR:
    {
        measure_motor_parameters();
        break;
    }
    case COMM_ENABLE_VOLTAGE_CONTROL:
    {
        start_voltage_control();
        break;
    }
    case COMM_ENABLE_TORQUE_CONTROL:
    {
        zero_encoder_offset(); // TODO: Remove this
        start_torque_control();
        break;
    }
    case COMM_ENABLE_IDLE_MODE:
    {
        enter_idle();
        break;
    }
    case COMM_DEVICE_RESTART:
    {
        reboot_system();
        break;
    }
    case COMM_VOLTAGE_SETPOINT:
    {
        Motor_setpoint_t *sp = (Motor_setpoint_t *)(packet_buffer+PACKET_DATA_OFFSET);
        //printf("SET POINT VOLTAGE: %f\n\r");
        set_voltage_control_ref(sp->v_d, sp->v_q);
    }
    case COMM_TORQUE_SETPOINT:
    {
        Motor_torque_setpoint_t *sp = (Motor_torque_setpoint_t *)(packet_buffer+PACKET_DATA_OFFSET);
        set_torque_control_ref(sp->k_p, 0, sp->pos, 0, 0);
        //printf("\r\nXX\r\n");
    }
    default:
        break;
    }
}
