/*
 * GamepadTeleopFSM.cpp
 *
 *  Created on: June 27, 2020
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

// C System Files

// C++ System Files
#include <iostream>

// Third Party Includes

// Project Include Files
#include <Nomad/FSM/NomadControlFSM.hpp>
#include <Nomad/FSM/OffState.hpp>
#include <Nomad/FSM/IdleState.hpp>
#include <Nomad/FSM/StandState.hpp>
#include <Nomad/NomadControl.hpp>

//#include <TransitionEvent.h>

namespace Robot::Nomad::FSM
{
    // Transition Events For States
    NomadControlTransitionEvent::NomadControlTransitionEvent(const std::string &name, std::shared_ptr<NomadControlData> data)
        : TransitionEvent(name), data_(data)
    {
        data_ = data;
    }

    NomadControlFSM::NomadControlFSM(Robot::Nomad::Controllers::NomadControl *control) : FiniteStateMachine("Nomad Primary Control FSM"), control_(control)
    {
        // Create Data Pointer
        data_ = std::make_unique<Robot::Nomad::FSM::NomadControlData>();

        _CreateFSM();
    }
    bool NomadControlFSM::Run(double dt)
    {
        // Now run base FSM code
        return FiniteStateMachine::Run(dt);
    }

    const std::shared_ptr<NomadControlData> &NomadControlFSM::GetData() const
    {
        return data_;
    }

    std::shared_ptr<Communications::Port> NomadControlFSM::GetOutputPort(const int port_id)
    {
        return control_->GetOutputPort(port_id);
    }

    std::shared_ptr<Communications::Port> NomadControlFSM::GetInputPort(const int port_id)
    {
        return control_->GetInputPort(port_id);
    }

    void NomadControlFSM::_CreateFSM()
    {
        std::cout << "[NomadControlFSM]: Creating FSM" << std::endl;

        ///////////////////////// Define Our States
        // Off
        std::shared_ptr<OffState> off = std::make_shared<OffState>();
        off->SetControllerData(data_);
        off->SetParentFSM(this);

        // Idle
        std::shared_ptr<IdleState> idle = std::make_shared<IdleState>();
        idle->SetControllerData(data_);
        idle->SetParentFSM(this);

        // Stand
        std::shared_ptr<StandState> stand = std::make_shared<StandState>();
        stand->SetControllerData(data_);
        stand->SetParentFSM(this);

        // // Sit
        // std::shared_ptr<SitState> sit = std::make_shared<SitState>();

        std::shared_ptr<CommandModeEvent> transition_idle = std::make_shared<CommandModeEvent>("IDLE TRANSITION", CONTROL_MODE::IDLE, data_);
        std::shared_ptr<CommandModeEvent> transition_stand = std::make_shared<CommandModeEvent>("STAND TRANSITION", CONTROL_MODE::STAND, data_);

        // Setup Transitions
        off->AddTransitionEvent(transition_idle, idle);
        idle->AddTransitionEvent(transition_stand, stand);
        // stand->AddTransitionEvent(down_event, sit);
        // sit->AddTransitionEvent(up_event, stand);

        // Add the states to the FSM
        AddState(off);
        AddState(idle);
        AddState(stand);
        // AddState(sit);

        // Set Initials State
        SetInitialState(idle);
    }
} // namespace Robot::Nomad::FSM