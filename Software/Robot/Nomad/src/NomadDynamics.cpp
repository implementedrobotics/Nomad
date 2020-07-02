/*
 * NomadDynamics.cpp
 *
 *  Created on: July 2, 2020
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
 */

// Primary Include
#include <Nomad/NomadDynamics.hpp>

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

namespace Robot
{
    namespace Nomad
    {
        namespace Dynamics
        {

            NomadDynamics::NomadDynamics(const std::string &name,
                                         const long rt_period,
                                         unsigned int rt_priority,
                                         const int rt_core_id,
                                         const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
            {

                // // Create Input/Output Messages
                // leg_command_msg_.length = sizeof(::Controllers::Locomotion::leg_controller_cmd_t);
                // leg_command_msg_.data.resize(leg_command_msg_.length);

                // control_mode_msg_.length = 1;
                // control_mode_msg_.data.resize(control_mode_msg_.length);

                // memset(&leg_controller_cmd_, 0, sizeof(::Controllers::Locomotion::leg_controller_cmd_t));

                // // Create Ports
                // // Primary Controller Input Port
                // input_port_map_[InputPort::CONTROL_MODE] = std::make_shared<Realtime::Port>("CONTROL_MODE", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::INT32, 1, rt_period_);

                // // Primary Controller Output Ports
                // output_port_map_[OutputPort::LEG_COMMAND] = std::make_shared<Realtime::Port>("LEG_COMMAND", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::BYTE, 1, rt_period);

                // // Create FSM
                // nomad_control_FSM_ = std::make_unique<Robot::Nomad::FSM::NomadControlFSM>();
            }

            void NomadDynamics::Run()
            {
                // // Get Control Inputs, Modes, Trajectory etc
                // // bool imu_recv = GetInputPort(InputPort::IMU)->Receive(x_hat_in_); // Receive Setpoint
                // // if (!imu_recv)
                // // {
                // //     std::cout << "[NomadControl]: Receive Buffer Empty!" << std::endl;
                // //     return;
                // // }
                // bool receive = GetInputPort(InputPort::CONTROL_MODE)->Receive(control_mode_msg_);

                // // Update Data
                // nomad_control_FSM_->GetData()->control_mode = control_mode_msg_.data[0];

                // // Run FSM
                // nomad_control_FSM_->Run(0);

                // // Get Desired Force Output to send out of leg controller
                // // Copy command to message
                // memcpy(leg_command_msg_.data.data(), &leg_controller_cmd_, sizeof(::Controllers::Locomotion::leg_controller_cmd_t));

                // // Publish Leg Command
                // bool send_status = GetOutputPort(OutputPort::LEG_COMMAND)->Send(leg_command_msg_);

                // //std::cout << "[NomadControl]: Publishing: Send: " << send_status << " : " <<  receive << std::endl;
            }

            void NomadDynamics::Setup()
            {
                // // Connect Input Ports
                // bool connect = GetInputPort(InputPort::CONTROL_MODE)->Connect();

                // bool binded = GetOutputPort(OutputPort::LEG_COMMAND)->Bind();

                // // Start FSM
                // nomad_control_FSM_->Start(Systems::Time::GetTime());

                // std::cout << "[NomadControl]: "
                //           << "Nomad Conrol FSM Publisher Running!: " << binded << std::endl;
            }
        } // namespace Dynamics
    }     // namespace Nomad
} // namespace Robot
