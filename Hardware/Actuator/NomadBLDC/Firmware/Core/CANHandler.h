/*
 * CanHandler.h
 *
 *  Created on: September 9, 2019
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

#ifndef CAN_HANDLER_H_
#define CAN_HANDLER_H_

// C System Files

// C++ System Files

// Project Includes
#include "mbed.h"
#include "CAN.h"

class CANHandler
{

public:

    // CAN Message Struct (Torque Mode)
    struct Torque_msg_t
    {
        float Pos_ref;               // Position Setpoint Reference
        float Vel_ref;               // Velocity Setpoint Reference
        float K_p;                   // Position Gain N*m/rad
        float K_d;                   // Velocity Gain N*m/rad/s
        float Tau_ff;                // Feed Forward Torque Value N*m
    };

    struct Config_t
    {
        uint32_t can_id;        // CAN BUS ID for this controller
        uint32_t can_master_id; // CAN BUS ID for 'master' contoller
        uint32_t can_timeout;   // CAN BUS Communication Timeout
    };

    CANHandler(PinName rx_pin, PinName tx_pin, uint32_t speed);
    void Interrupt();
    
    Config_t config_;

protected:

    CAN *can_;
    CANMessage rx_msg_;
    CANMessage tx_msg_;
    PinName rx_pin_;
    PinName tx_pin_;
    uint32_t speed_;

private:

    void PackReply(CANMessage &msg, float pos, float vel, float torque);
    void Unpack(CANMessage &msg, Torque_msg_t &torque_command);
    
};

#endif // CAN_HANDLER_H_