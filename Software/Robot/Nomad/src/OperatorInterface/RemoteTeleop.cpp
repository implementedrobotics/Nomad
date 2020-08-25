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
    RemoteTeleop::RemoteTeleop(const double T_s)  : SystemBlock("Gamepad_Teleop", T_s)
    {
        // Create Ports
        output_port_map_[OutputPort::TELEOP_DATA] = Communications::Port::CreateOutput("TELEOP_DATA");

        // Create Gamepad Object
        // TODO: Needs to be a parameter for game pad device
        gamepad_ = std::make_shared<GamepadInterface>("/dev/input/js0");

        // Create FSM for game pad state
        gamepad_FSM_ = std::make_unique<GamepadTeleopFSM>(gamepad_);
    }


    void RemoteTeleop::UpdateStateOutputs()
    {
        // Receive Data
    }

    // Update function for stateless outputs
    void RemoteTeleop::UpdateStatelessOutputs()
    {
        // Run FSM
        gamepad_FSM_->Run(T_s_);

        // Get Mode from FSM
        teleop_data_.control_mode = gamepad_FSM_->GetMode(); // Mode Type

        // TODO: For now...  Switch control mode here and set setpoint data in teleop data
        // Basically for now this is a teleop handler specifically for the gamepad. 
        // Eventually need this more generic maybe in a trajectory generator?  We will need
        // one anyway for the more advanced controllers that have long horizon windows

        // TODO: Parameters for some of these values like robot limits

        // Also Stance Height Here.
        // Left Stick Y +/- adjust desired COM Height from "Base" stance height
        // Switch stand sit to A/Y
        // Up/Down change base stance height
        
        switch (gamepad_FSM_->GetMode() )
        {
        case GamepadState::STAND:
            std::cout << "stand---------------------------------------------------------------------------------" << std::endl;
            break;
        
        default:
        std::cout << "$*$*(%$(%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*---------------------------------------------------------------------------------" << std::endl;
            break;
        }

        // Publish Messages
        GetOutputPort(OutputPort::TELEOP_DATA)->Send(teleop_data_);
    }

    // Update function for next state from inputs
    void RemoteTeleop::UpdateState()
    {

    }

    void RemoteTeleop::Setup()
    {
        // Run base class setup
        SystemBlock::Setup();

        // Start FSM
        gamepad_FSM_->Start(Systems::Time::GetTime<double>());
    }
} // namespace OperatorInterface::Teleop
