/*
 * NomadControl.cpp
 *
 *  Created on: July 1, 2020
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
#include <Nomad/NomadControl.hpp>

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

namespace Robot::Nomad::Controllers
{
    NomadControl::NomadControl(const std::string &name,
                               const long rt_period,
                               unsigned int rt_priority,
                               const int rt_core_id,
                               const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
    {

        // // Create Input/Output Messages
        // leg_command_msg_.length = sizeof(::Controllers::Locomotion::leg_controller_cmd_t);
        // leg_command_msg_.data.resize(leg_command_msg_.length);

        // Create Ports
        // Primary Controller Input Port
        input_port_map_[InputPort::TELEOP_DATA] = Communications::Port::CreateInput<teleop_data_t>("TELEOP_DATA", rt_period_);
        input_port_map_[InputPort::FULL_STATE] = Communications::Port::CreateInput<full_state_t>("FULL_STATE", rt_period_);

        // Primary Controller Output Ports
        output_port_map_[OutputPort::LEG_COMMAND] = Communications::Port::CreateOutput("LEG_COMMAND", rt_period_);

        // Create FSM
        nomad_control_FSM_ = std::make_unique<Robot::Nomad::FSM::NomadControlFSM>();
    }

    void NomadControl::Run()
    {

        if (GetInputPort(InputPort::TELEOP_DATA)->Receive(teleop_data_))
        {
        }
        if (GetInputPort(InputPort::FULL_STATE)->Receive(full_state_))
        {
        }

        // Update Data
        nomad_control_FSM_->GetData()->control_mode = teleop_data_.control_mode;
        nomad_control_FSM_->GetData()->nomad_state = full_state_;

        // Run FSM
        nomad_control_FSM_->Run(rt_period_*1e-3);


        std::cout << Systems::Time::GetTime<double>() << std::endl;

        {
            Systems::Time test;
            for(int i = 0; i < 100000; i++)
            {
                
            }
        }
        // Get Desired Force Output to send out of leg controller
        // Copy command to message
        //memcpy(leg_command_msg_.data.data(), &leg_controller_cmd_, sizeof(::Controllers::Locomotion::leg_controller_cmd_t));

        // Publish Leg Command
        //bool send_status = GetOutputPort(OutputPort::LEG_COMMAND)->Send(leg_command_msg_);

        //std::cout << "[NomadControl]: Publishing: Send: " << send_status << " : " <<  receive << std::endl;
    }

    void NomadControl::Setup()
    {
        bool binded = GetOutputPort(OutputPort::LEG_COMMAND)->Bind();

        // Start FSM
        nomad_control_FSM_->Start(Systems::Time::GetTime<double>());

        std::cout << "[NomadControl]: "
                  << "Nomad Control FSM Publisher Running!: " << binded << std::endl;
    }
} // namespace Robot::Nomad::Controllers
