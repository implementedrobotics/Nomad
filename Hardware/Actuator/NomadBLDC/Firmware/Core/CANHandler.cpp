
/*
 * CANHandler.cpp
 *
 *  Created on: August 28, 2019
 *      Author: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
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
#include "CANHandler.h"

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "rtos.h"
#include "../math_ops.h"
#include "motor_controller_interface.h"
#include "MotorController.h"

// TODO: Make this variable -> Config.  Should match on both ends
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

CANHandler::CANHandler(PinName rx_pin, PinName tx_pin, uint32_t speed) : rx_pin_(rx_pin), tx_pin_(tx_pin), speed_(speed)
{

    // CAN Handler
    can_ = new CAN(rx_pin_, tx_pin_, speed_); // CAN Device

    // Setup Default Configs
    config_.can_id = 1;
    config_.can_master_id = 0;
    config_.can_timeout = 0;

    // Adjust Priority < Commutation Sampling Interrupt
    NVIC_SetPriority(CAN1_RX0_IRQn, 3);

    // Setup Filters
    can_->filter(config_.can_id << 21, 0xFFE00004, CANFormat::CANStandard, 0);

    tx_msg_.id = config_.can_id;
    tx_msg_.len = 6; // 6 Bytes(48-bits) = [Pos (16 bits) | Vel (12 bits) | T_hat (12 bits) | CAN ID (8 bits)]
    rx_msg_.len = 8; // 8 Bytes(64-bits) = [Pos Ref (16 bits) | Vel Ref (12 bits) | K_p (12 bits) | K_d (12 bits) | Torque (12 bits)]

    can_->attach(callback(this, &CANHandler::Interrupt)); // Attach 'CAN receive-complete' interrupt handler
}

void CANHandler::Interrupt()
{
    MotorController *controller = MotorController::GetInstance();
    if (controller == nullptr) // Not Up/Initted Yet
        return;

    can_->read(rx_msg_);
    printf("%df\n\r", rx_msg_.id);
    if ((rx_msg_.id == config_.can_id))
    {
        controller->state_.timeout = 0;
        if (((rx_msg_.data[0] == 0xFF) & (rx_msg_.data[1] == 0xFF) & (rx_msg_.data[2] == 0xFF) & (rx_msg_.data[3] == 0xFF) & (rx_msg_.data[4] == 0xFF) & (rx_msg_.data[5] == 0xFF) & (rx_msg_.data[6] == 0xFF) & (rx_msg_.data[7] == 0xFC)))
        {
            // Enable Motor Mode
        }
        else if (((rx_msg_.data[0] == 0xFF) & (rx_msg_.data[1] == 0xFF) & (rx_msg_.data[2] == 0xFF) & (rx_msg_.data[3] == 0xFF) & (rx_msg_.data[4] == 0xFF) & (rx_msg_.data[5] == 0xFF) & (rx_msg_.data[6] == 0xFF) & (rx_msg_.data[7] == 0xFD)))
        {
            // Enter IDLE Mode
        }
        else if (((rx_msg_.data[0] == 0xFF) & (rx_msg_.data[1] == 0xFF) & (rx_msg_.data[2] == 0xFF) & (rx_msg_.data[3] == 0xFF) & (rx_msg_.data[4] == 0xFF) & (rx_msg_.data[5] == 0xFF) & (rx_msg_.data[6] == 0xFF) & (rx_msg_.data[7] == 0xFE)))
        {
            // Zero Encoder
        }
        else if (controller->GetControlMode() == FOC_TORQUE_MODE)
        {
            Torque_msg_t command;
            Unpack(rx_msg_, command);
            set_torque_control_ref(command.K_p, command.K_d, command.Pos_ref, command.Vel_ref, command.Tau_ff);
        }
        PackReply(tx_msg_, controller->GetMotor()->state_.theta_mech, controller->GetMotor()->state_.theta_mech_dot, controller->state_.I_q_filtered * controller->GetMotor()->config_.K_t_out);
        can_->write(tx_msg_);
    }
}

// CAN Torque Command Packet Structure
// 16 bit position command, between -4*pi and 4*pi
// 12 bit velocity command, between -30 and + 30 rad/s
// 12 bit kp, between 0 and 500 N-m/rad
// 12 bit kd, between 0 and 100 N-m*s/rad
// 12 bit feed forward torque, between -18 and 18 N-m
// CAN Packet is 8 8-bit words
// Formatted as follows.  For each quantity, bit 0 is LSB
// 0: [position[15-8]]
// 1: [position[7-0]]
// 2: [velocity[11-4]]
// 3: [velocity[3-0], kp[11-8]]
// 4: [kp[7-0]]
// 5: [kd[11-4]]
// 6: [kd[3-0], torque[11-8]]
// 7: [torque[7-0]]
void CANHandler::Unpack(CANMessage &msg, Torque_msg_t &torque_command)
{
    int p_int = (msg.data[0] << 8) | msg.data[1];
    int v_int = (msg.data[2] << 4) | (msg.data[3] >> 4);
    int kp_int = ((msg.data[3] & 0xF) << 8) | msg.data[4];
    int kd_int = (msg.data[5] << 4) | (msg.data[6] >> 4);
    int t_int = ((msg.data[6] & 0xF) << 8) | msg.data[7];

    torque_command.Pos_ref = uint_to_float(p_int, P_MIN, P_MAX, 16);
    torque_command.Vel_ref = uint_to_float(v_int, V_MIN, V_MAX, 12);
    torque_command.K_p = uint_to_float(kp_int, KP_MIN, KP_MAX, 12);
    torque_command.K_d = uint_to_float(kd_int, KD_MIN, KD_MAX, 12);
    torque_command.Tau_ff = uint_to_float(t_int, T_MIN, T_MAX, 12);

    printf("Received   ");
    printf("%.3f  %.3f  %.3f  %.3f  %.3f", torque_command.Pos_ref, torque_command.Vel_ref, torque_command.K_p, torque_command.K_d, torque_command.Tau_ff);
    printf("\n\r");
}

// CAN Reply Packet Structure
// 16 bit position, between -4*pi and 4*pi
// 12 bit velocity, between -30 and + 30 rad/s
// 12 bit current, between -40 and 40;
// CAN Packet is 5 8-bit words
// Formatted as follows.  For each quantity, bit 0 is LSB
// 0: [position[15-8]]
// 1: [position[7-0]]
// 2: [velocity[11-4]]
// 3: [velocity[3-0], current[11-8]]
// 4: [current[7-0]]
void CANHandler::PackReply(CANMessage &msg, float pos, float vel, float torque)
{
    int pos_int = float_to_uint(pos, P_MIN, P_MAX, 16);
    int vel_int = float_to_uint(vel, V_MIN, V_MAX, 12);
    int torque_int = float_to_uint(torque, -T_MAX, T_MAX, 12);
    msg.data[0] = config_.can_id;
    msg.data[1] = pos_int >> 8;
    msg.data[2] = pos_int & 0xFF;
    msg.data[3] = vel_int >> 4;
    msg.data[4] = ((vel_int & 0xF) << 4) + (torque_int >> 8);
    msg.data[5] = torque_int & 0xFF;
}
