/*
 * RemoteTeleop.cpp
 *
 *  Created on: July 17, 2019
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
#include <Nomad/OperatorInterface/RemoteTeleop.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>

// Third-Party Includes

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Common/Time.hpp>

namespace OperatorInterface::Teleop
{
    RemoteTeleop::RemoteTeleop(const std::string &name,
                               const long rt_period,
                               unsigned int rt_priority,
                               const int rt_core_id,
                               const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
    {
        // Create Ports
        output_port_map_[OutputPort::TELEOP_DATA] = Communications::Port::CreateOutput("TELEOP_DATA", rt_period_);

        // Create Gamepad Object
        // TODO: Needs to be a parameter for game pad device
        gamepad_ = std::make_shared<GamepadInterface>("/dev/input/js0");

        // Create FSM for game pad state
        gamepad_FSM_ = std::make_unique<GamepadTeleopFSM>(gamepad_);
    }

    void RemoteTeleop::Run()
    {
        // Run FSM
        gamepad_FSM_->Run(0);

        // Get Mode from FSM
        teleop_data_.control_mode = gamepad_FSM_->GetMode(); // Mode Type

        // Publish Messages
        GetOutputPort(OutputPort::TELEOP_DATA)->Send(teleop_data_);
    }

    void RemoteTeleop::Setup()
    {
        // Start FSM
        gamepad_FSM_->Start(Systems::Time::GetTime<double>());

        // TODO: Autobind NON-NULL output port
        GetOutputPort(OutputPort::TELEOP_DATA)->Bind();
    }
} // namespace OperatorInterface::Teleop
