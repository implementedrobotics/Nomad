/*
 * StateEstimator.cpp
 *
 *  Created on: July 13, 2019
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
 */

// Primary Include
#include <Controllers/PrimaryControl.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <memory>

// Third-Party Includes
#include <zcm/zcm-cpp.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Controllers/LegController.hpp>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Common/Time.hpp>


namespace Controllers
{
namespace FSM
{

PrimaryControl::PrimaryControl(const std::string &name,
                               const long rt_period,
                               unsigned int rt_priority,
                               const int rt_core_id,
                               const unsigned int stack_size) : 
                               Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
{

    // Create Input/Output Messages
    leg_command_msg_.length = sizeof(Controllers::Locomotion::leg_controller_cmd_t);
    leg_command_msg_.data.resize(leg_command_msg_.length);

    memset(&leg_controller_cmd_, 0, sizeof(Controllers::Locomotion::leg_controller_cmd_t));

    // Create Ports
    // Primary Controller Input Port
    ///input_port_map_[InputPort::LEG_COMMAND] = std::make_shared<Realtime::Port>("LEG_COMMAND", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::BYTE, 1, rt_period_);

    // Primary Controller Output Ports
    output_port_map_[OutputPort::LEG_COMMAND] = std::make_shared<Realtime::Port>("LEG_COMMAND", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::BYTE, 1, rt_period);
}

void PrimaryControl::Run()
{
    // Get Control Inputs, Modes, Trajectory etc
    // bool imu_recv = GetInputPort(InputPort::IMU)->Receive(x_hat_in_); // Receive Setpoint
    // if (!imu_recv)
    // {
    //     std::cout << "[StateEstimator]: Receive Buffer Empty!" << std::endl;
    //     return;
    // }

    // 
    // Copy command to message
    memcpy(leg_command_msg_.data.data(), &leg_controller_cmd_, sizeof(Controllers::Locomotion::leg_controller_cmd_t));

    // Publish Leg Command
    bool send_status = GetOutputPort(OutputPort::LEG_COMMAND)->Send(leg_command_msg_);
    
    //std::cout << "[PrimaryControl]: Publishing: Send: " << send_status << std::endl;
}

void PrimaryControl::Setup()
{
    // Connect Input Ports
    //bool connect = GetInputPort(InputPort::)->Connect();  

    bool binded = GetOutputPort(OutputPort::LEG_COMMAND)->Bind();
    std::cout << "[PrimaryControl]: " << "Primary Conrol FSM Publisher Running!: " << binded << std::endl;
}

} // namespace FSM
} // namespace Controllers
