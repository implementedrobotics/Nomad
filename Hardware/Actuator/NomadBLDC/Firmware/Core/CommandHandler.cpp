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
#include "MotorController.h"
#include "Motor.h"
#include "motor_controller_interface.h"
#include "Core/nomad_common.h"
#include "Core/Logger.h"

HDLCHandler hdlc_out;
#define PACKET_DATA_OFFSET 2

struct __attribute__((__packed__)) Measurement_info_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    uint8_t status;            // Status
    float measurement;         // Measurement Value
};

struct __attribute__((__packed__)) Device_info_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    uint8_t fw_major;          // Firmware Version Major
    uint8_t fw_minor;          // Firmware Version Minor
    uint32_t uid1;             // Device Unique ID 1
    uint32_t uid2;             // Device Unique ID 2
    uint32_t uid3;             // Device Unique ID 3
};

struct __attribute__((__packed__)) Device_stats_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    uint8_t fault;             // Controller fault
    uint8_t control_status;    // Controller status/mode
    uint32_t uptime;           // Device Uptime
    float voltage_bus;         // System Bus Voltage
    float driver_temp;         // Gate Driver Temp
    float fet_temp;            // FET Temp
    float motor_temp;          // Motor Temp
};

struct __attribute__((__packed__)) Motor_config_packet_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    Motor::Config_t config;    // Motor config
};

struct __attribute__((__packed__)) Motor_setpoint_t
{
    float v_d;                 // V_d Setpoint
    float v_q;                 // V_q Setpoint
};

struct __attribute__((__packed__)) Motor_torque_setpoint_t
{
    float k_p;                 // K_p position gain Setpoint
    float k_d;                 // K_d damping Setpoint
    float pos;                 // Output position Setpoint
    float vel;                 // Output velocity Setpoint
    float tau_ff;              // Torque Feed Forward
};

struct __attribute__((__packed__)) Log_command_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    char log_buffer[256];      // String Buffer
};

// Command Handler Class
CommandHandler::CommandHandler()
{
}
void CommandHandler::LogCommand(const std::string log_string)
{
    // Create Log Packet
    Log_command_t log;
    log.comm_id = COMM_LOGGING_OUTPUT;
    log.packet_length = log_string.size();
    
    // Copy in data
    memcpy(log.log_buffer, (uint8_t *)log_string.c_str(), log_string.size());
    
    // HDLC Frame and Send Packet
    hdlc_out.SendPacket((uint8_t *)&log, log_string.size() + PACKET_DATA_OFFSET);
}

void CommandHandler::SendMeasurementComplete(command_feedback_t fb, uint8_t status, float measurement)
{
    switch(fb)
    {
        case command_feedback_t::MEASURE_RESISTANCE_COMPLETE:
        {
            Measurement_info_t result;
            result.comm_id = COMM_MEASURE_RESISTANCE;
            result.packet_length = sizeof(Measurement_info_t) - PACKET_DATA_OFFSET; // First 2 bytes don't count for packet length[CommandID/PacketLength]
            result.status = status;
            result.measurement = measurement;
            // Send it
            hdlc_out.SendPacket((uint8_t *)&result, sizeof(Measurement_info_t));
            break;
        }
        case command_feedback_t::MEASURE_INDUCTANCE_COMPLETE:
        {
            Measurement_info_t result;
            result.comm_id = COMM_MEASURE_INDUCTANCE;
            result.packet_length = sizeof(Measurement_info_t) - PACKET_DATA_OFFSET; // First 2 bytes don't count for packet length[CommandID/PacketLength]
            result.status = status;
            result.measurement = measurement;
            // Send it
            hdlc_out.SendPacket((uint8_t *)&result, sizeof(Measurement_info_t));
            break;
        }
        default:
            break;
    }
}

void CommandHandler::ProcessPacket(const uint8_t *packet_buffer, uint16_t packet_length)
{
    command_t command = static_cast<command_t>(packet_buffer[0]);
    switch (command)
    {
    case COMM_DEVICE_INFO:
    {
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
    case COMM_DEVICE_STATS:
    {
        Device_stats_t stats;
        stats.comm_id = COMM_DEVICE_STATS;
        stats.packet_length = sizeof(Device_stats_t) - PACKET_DATA_OFFSET; // First 2 bytes don't count for packet length[CommandID/PacketLength]
        stats.uptime = HAL_GetTick()/1000;
        stats.fault = 0;
        stats.control_status = motor_controller->GetControlMode();
        stats.voltage_bus = motor_controller->state_.Voltage_bus;
        stats.driver_temp = 60.2f;
        stats.fet_temp = 100.0f;
        stats.motor_temp = 40.0f;


        // Send it
        hdlc_out.SendPacket((uint8_t *)&stats, sizeof(Device_stats_t));

        //Logger::Instance().Print("Phase Resistance Measurement Complete: \n\r");

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
        break;
    }
    case COMM_TORQUE_SETPOINT:
    {
        Motor_torque_setpoint_t *sp = (Motor_torque_setpoint_t *)(packet_buffer+PACKET_DATA_OFFSET);
        Logger::Instance().Print("Got: %f and %f\r\n", sp->k_p, sp->pos)
        set_torque_control_ref(sp->k_p, sp->k_d, sp->pos, sp->vel, sp->tau_ff);
        break;
        //printf("\r\nXX\r\n");
    }
    case COMM_MEASURE_RESISTANCE:
    {
        bool status = measure_motor_resistance();
        break;
    }
    case COMM_MEASURE_INDUCTANCE:
    {
        bool status = measure_motor_inductance();
        break;
    }


    // Read Configs
    case COMM_READ_MOTOR_CONFIG:
    {
        Motor_config_packet_t packet;
        packet.comm_id = COMM_READ_MOTOR_CONFIG;
        packet.packet_length = sizeof(Motor::Config_t);
        packet.config = motor->config_;
        //Logger::Instance().Print("Config Size: %d\n", sizeof(Motor::Config_t));
        //Logger::Instance().Print("Packet Size: %d\n", sizeof(packet));

        // Send it
        hdlc_out.SendPacket((uint8_t *)&packet, sizeof(Motor_config_packet_t));
        //Logger::Instance().Print("Packet Sent CONFIG: %d\n", (sizeof(Motor_config_packet_t)));
        //Motor::Config_t motor_config = motor->config_;

        break;
    }

    default:
        break;
    }
}
