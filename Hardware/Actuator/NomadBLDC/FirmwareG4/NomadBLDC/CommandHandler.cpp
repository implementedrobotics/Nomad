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
#include <cstring>

// C++ System Files

// Project Includes
#include <Peripherals/uart.h>

#include "MotorController.h"
#include "Motor.h"
#include "motor_controller_interface.h"
#include "nomad_hw.h"
// #include "Core/Logger.h"

#define PACKET_DATA_OFFSET 2

struct __attribute__((__packed__)) Measurement_info_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    uint8_t status;            // Status
    measurement_t measurement;         // Measurement Value
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

struct __attribute__((__packed__)) Position_config_packet_t
{
    uint8_t comm_id;                          // Command ID
    uint8_t packet_length;                    // Packet Length
    PositionSensorAS5x47::Config_t config;    // Position config
};

struct __attribute__((__packed__)) Position_state_packet_t
{
    uint8_t comm_id;               // Command ID
    uint8_t packet_length;         // Packet Length
    int32_t pos_raw;               // Sensor Raw Position (counts)
    int32_t num_rotations;         // Keep Track of Sensor Rotations
    float pos_elec;                // Sensor Electrical Position (radians)
    float pos_mech;                // Sensor Mechanical Position (radians)
    float vel_elec;                // Sensor Electrical Velocity (radians/sec)
    float vel_elec_filtered_;      // Sensor Filtered Electrical Velocity (radians/sec)
    float vel_mech;                // Sensor Mechanical Velocity (radians/sec)
};

struct __attribute__((__packed__)) Controller_config_packet_t
{
    uint8_t comm_id;                     // Command ID
    uint8_t packet_length;               // Packet Length
    MotorController::Config_t config;    // Controller config
};

struct __attribute__((__packed__)) Controller_state_packet_t
{
    uint8_t comm_id;                     // Command ID
    uint8_t packet_length;               // Packet Length
    MotorController::State_t state;      // Controller state
};

struct __attribute__((__packed__)) Motor_config_packet_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    Motor::Config_t config;    // Motor config
};

struct __attribute__((__packed__)) Motor_state_packet_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    Motor::State_t state;      // Motor state
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

struct __attribute__((__packed__)) Motor_current_setpoint_t
{
    float i_d;                 // I_d Setpoint
    float i_q;                 // I_q Setpoint
};

struct __attribute__((__packed__)) Log_command_t
{
    uint8_t comm_id;           // Command ID
    uint8_t packet_length;     // Packet Length
    char log_buffer[256];      // String Buffer
};

void CommandHandler::LogCommand(const std::string log_string)
{
    // Create Log Packet
    Log_command_t log;
    log.comm_id = COMM_LOGGING_OUTPUT;
    log.packet_length = log_string.size();
    
    // Copy in data
    memcpy(log.log_buffer, (uint8_t *)log_string.c_str(), log_string.size());
    
    // HDLC Frame and Send Packet
   // uart_send_data_hdlc((uint8_t *)&log, log_string.size() + PACKET_DATA_OFFSET);
   // uart_send_str(log_string.c_str());
}

void CommandHandler::SendMeasurementComplete(command_feedback_t fb, uint8_t status, measurement_t measurement)
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
            uart_send_data_hdlc((uint8_t *)&result, sizeof(Measurement_info_t));
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
            uart_send_data_hdlc((uint8_t *)&result, sizeof(Measurement_info_t));
            break;
        }
    case command_feedback_t::MEASURE_PHASE_ORDER_COMPLETE:
        {
            Measurement_info_t result;
            result.comm_id = COMM_MEASURE_PHASE_ORDER;
            result.packet_length = sizeof(Measurement_info_t) - PACKET_DATA_OFFSET; // First 2 bytes don't count for packet length[CommandID/PacketLength]
            result.status = status;
            result.measurement = measurement;
            
            // Send it
            uart_send_data_hdlc((uint8_t *)&result, sizeof(Measurement_info_t));
            break;
        }
    case command_feedback_t::MEASURE_ENCODER_OFFSET_COMPLETE:
        {
            Measurement_info_t result;
            result.comm_id = COMM_MEASURE_ENCODER_OFFSET;
            result.packet_length = sizeof(Measurement_info_t) - PACKET_DATA_OFFSET; // First 2 bytes don't count for packet length[CommandID/PacketLength]
            result.status = status;
            result.measurement = measurement;
            
            // Send it
            uart_send_data_hdlc((uint8_t *)&result, sizeof(Measurement_info_t));
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
        uart_send_data_hdlc((uint8_t *)&info, sizeof(Device_info_t));
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
        stats.fet_temp = motor_controller->state_.I_rms;
        stats.motor_temp = 80.0f;

        // Send it
        uart_send_data_hdlc((uint8_t *)&stats, sizeof(Device_stats_t));
        break;
    }
    case COMM_CALIB_MOTOR:
    {
        measure_motor_parameters();
        break;
    }
    case COMM_ENABLE_CURRENT_CONTROL:
    {
        start_current_control();
        break;
    }
    case COMM_ENABLE_VOLTAGE_CONTROL:
    {
        start_voltage_control();
        break;
    }
    case COMM_ENABLE_TORQUE_CONTROL:
    {
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
        //Logger::Instance().Print("Controller Ticks: %d\n", motor_controller->pwm_counter_period_ticks_);
        //Logger::Instance().Print("Controller Freq: %f\n", motor_controller->controller_loop_freq_);
        //Logger::Instance().Print("Controller Period: %f\n", motor_controller->controller_update_period_);
        //Logger::Instance().Print("Let's GO Period:\n");
        reboot_system();
        break;
    }
    case COMM_CURRENT_SETPOINT:
    {
        Motor_current_setpoint_t *sp = (Motor_current_setpoint_t *)(packet_buffer+PACKET_DATA_OFFSET);
        set_current_control_ref(sp->i_d, sp->i_q);
        break;
    }
    case COMM_VOLTAGE_SETPOINT:
    {
        Motor_setpoint_t *sp = (Motor_setpoint_t *)(packet_buffer+PACKET_DATA_OFFSET);
        set_voltage_control_ref(sp->v_d, sp->v_q);
        break;
    }
    case COMM_TORQUE_SETPOINT:
    {
        Motor_torque_setpoint_t *sp = (Motor_torque_setpoint_t *)(packet_buffer+PACKET_DATA_OFFSET);
        set_torque_control_ref(sp->k_p, sp->k_d, sp->pos, sp->vel, sp->tau_ff);
        break;
    }
    case COMM_MEASURE_RESISTANCE:
    {
        // TODO: Return Here?
        measure_motor_resistance();
        break;
    }
    case COMM_MEASURE_INDUCTANCE:
    {
        // TODO: Return Here?
        measure_motor_inductance();
        break;
    }
    case COMM_MEASURE_PHASE_ORDER:
    {
        // TODO: Return Here?
        measure_motor_phase_order();
        break;
    }
    case COMM_MEASURE_ENCODER_OFFSET:
    {
        // TODO: Return Here?
        measure_encoder_offset();
        break;
    }
    case COMM_ZERO_ENCODER_POSITION:
    {
        zero_encoder_offset();
        
        //Logger::Instance().Print("Zero Encoder: %f\n", motor->PositionSensor()->config_.offset_mech);
        break;
    }
    // Write Configs
    case COMM_WRITE_MOTOR_CONFIG:
    {
        Motor::Config_t *config = (Motor::Config_t *)(packet_buffer+PACKET_DATA_OFFSET);
        memcpy(&motor->config_, config, sizeof(Motor::Config_t));


       // Logger::Instance().Print("Motor Config Write Test: %f\n", motor->config_.phase_inductance_d);
       // Logger::Instance().Print("Motor Config Write Test: %f\n", motor->config_.phase_inductance_q);
       // Logger::Instance().Print("Motor Config Write Test: %f\n", motor->config_.phase_resistance);
        break;
        // TODO: Return status

    }

    case COMM_WRITE_CONTROLLER_CONFIG:
    {
        MotorController::Config_t *config = (MotorController::Config_t *)(packet_buffer+PACKET_DATA_OFFSET);
        memcpy(&motor_controller->config_, config, sizeof(MotorController::Config_t));

      //  Logger::Instance().Print("Controller Config Write Test: \n");
       // Logger::Instance().Print("Controller Config Write Test: %f\n", motor_controller->config_.pwm_freq);
       // Logger::Instance().Print("Controller Config Write Test: %f\n", motor_controller->config_.alpha);
        break;
        // TODO: Return status

    }

    case COMM_WRITE_POSITION_CONFIG:
    {
        PositionSensorAS5x47::Config_t *config = (PositionSensorAS5x47::Config_t *)(packet_buffer+PACKET_DATA_OFFSET);
        //memcpy(&motor->PositionSensor()->config_, config, 12); // Only copy first 12 bytes, do not want to pass encoder offset back and forth right now

        motor->PositionSensor()->config_.cpr = config->cpr;
        //Logger::Instance().Print("Encoder Config Write Test: %d\n", motor->PositionSensor()->config_.cpr);
        break;
        // TODO: Return status

    }

    // Read Configs
    case COMM_READ_MOTOR_CONFIG:
    {
        Motor_config_packet_t packet;
        packet.comm_id = COMM_READ_MOTOR_CONFIG;
        packet.packet_length = sizeof(Motor::Config_t);
        packet.config = motor->config_;

        // Send it
        uart_send_data_hdlc((uint8_t *)&packet, sizeof(Motor_config_packet_t));
        break;
    }
    case COMM_READ_CONTROLLER_CONFIG:
    {
        Controller_config_packet_t packet;
        packet.comm_id = COMM_READ_CONTROLLER_CONFIG;
        packet.packet_length = sizeof(MotorController::Config_t);
        packet.config = motor_controller->config_;

        // Send it
        uart_send_data_hdlc((uint8_t *)&packet, sizeof(Controller_config_packet_t));
        break;
    }
    case COMM_READ_POSITION_CONFIG:
    {
        Position_config_packet_t packet;
        packet.comm_id = COMM_READ_POSITION_CONFIG;
        packet.packet_length = sizeof(PositionSensorAS5x47::Config_t);
        packet.config = motor->PositionSensor()->config_;
    
        // Send it
        uart_send_data_hdlc((uint8_t *)&packet, sizeof(Position_config_packet_t));
        break;
    }

    // Read State
    case COMM_READ_MOTOR_STATE:
    {
        Motor_state_packet_t packet;
        packet.comm_id = COMM_READ_MOTOR_STATE;
        packet.packet_length = sizeof(Motor::State_t);
        packet.state = motor->state_;

        // Send it
        uart_send_data_hdlc((uint8_t *)&packet, sizeof(Motor_state_packet_t));
        break;
    }
    case COMM_READ_CONTROLLER_STATE:
    {
        Controller_state_packet_t packet;
        packet.comm_id = COMM_READ_CONTROLLER_STATE;
        packet.packet_length = sizeof(MotorController::State_t);
        packet.state = motor_controller->state_;

        // Send it
        uart_send_data_hdlc((uint8_t *)&packet, sizeof(Controller_state_packet_t));
        break;
    }
    case COMM_READ_POSITION_STATE:
    {
        PositionSensorAS5x47 *encoder = motor->PositionSensor();

        Position_state_packet_t packet;
        packet.comm_id = COMM_READ_POSITION_STATE;
        packet.packet_length = 28;
        packet.pos_raw = encoder->GetRawPosition();
        packet.num_rotations = encoder->GetNumRotations();
        packet.pos_elec = encoder->GetElectricalPosition();
        packet.pos_mech = encoder->GetMechanicalPosition();
        packet.vel_elec = encoder->GetElectricalVelocity();
        packet.vel_elec_filtered_ = encoder->GetElectricalVelocityFiltered();
        packet.vel_mech = encoder->GetMechanicalVelocity();

        // Send it
        uart_send_data_hdlc((uint8_t *)&packet, sizeof(Position_state_packet_t));
        break;
    }
    // Write Configs
    case COMM_WRITE_FLASH:
    {
        //Logger::Instance().Print("Writing\n");
        bool status = false;
        if(motor_controller->GetControlMode() == control_mode_type_t::IDLE_MODE)
        {
            status = save_configuration();
        }
        //bool status = save_configuration();

        uint8_t buffer[3];
        buffer[0] = COMM_WRITE_FLASH;
        buffer[1] = 1;
        buffer[2] = (uint32_t)status;

        // Send it
        uart_send_data_hdlc((uint8_t *)&buffer, 3);

        //Logger::Instance().Print("Write to Flash: %d", status);
        break;
    }

    default:
        break;
    }
}
