/*
 * NomadBLDCFSM.cpp
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

// Third Party Includes

// Project Include Files
#include <FSM/NomadBLDCFSM.h>
#include <FSM/StartupState.h>
#include <FSM/IdleState.h>
#include <Logger.h>

// Transition Events For States
NomadBLDCTransitionEvent::NomadBLDCTransitionEvent(const std::string &name, NomadBLDCData* data)
    : TransitionEvent(name), data_(data)
{
    data_ = data;
}

NomadBLDCFSM::NomadBLDCFSM() : FiniteStateMachine("Nomad BLDC Controller FSM")
{
    // Create Data Pointer
    data_ = new NomadBLDCData();

    _CreateFSM();
}
bool NomadBLDCFSM::Run(float dt)
{
    // Now run base FSM code
    return FiniteStateMachine::Run(dt);
}

const NomadBLDCData* NomadBLDCFSM::GetData() const
{
    return data_;
}

void NomadBLDCFSM::_CreateFSM()
{
    //std::cout << "[NomadControlFSM]: Creating FSM" << std::endl;
    Logger::Instance().Print("Creating FSM\r\n");

    // Initialization/Startup State
    // Zero Current Sensors
    // Setup and Do Things
    // Calibrate Mode
    // Change to Idle

    // ///////////////////////// Define Our States
    // // Off
    // std::shared_ptr<OffState> off = std::make_shared<OffState>();
    // off->SetControllerData(data_);
    // off->SetParentFSM(this);

    // Startup
    StartupState *startup = new StartupState();
    startup->SetControllerData(data_);
    startup->SetParentFSM(this);

    // Idle
    IdleState *idle = new IdleState();
    idle->SetControllerData(data_);
    idle->SetParentFSM(this);

    ///////////////////////// Define Our Transitions
    CommandModeEvent *transition_idle = new CommandModeEvent("IDLE TRANSITION", control_mode_type_t::IDLE_MODE, data_);

    // Setup Transitions
    startup->AddTransitionEvent(transition_idle, idle);

    // Add the states to the FSM
    AddState(startup);
    AddState(idle);

    // Set Initials State
    SetInitialState(startup);
}
