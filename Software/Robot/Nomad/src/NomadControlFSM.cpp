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
//#include <OperatorInterface/GamepadTeleopFSM/GamepadTeleopFSM.hpp>
//#include <OperatorInterface/GamepadTeleopFSM/States/GamepadState.hpp>
#include <Nomad/FSM/NomadControlFSM.hpp>
#include <Nomad/FSM/OffState.hpp>

//#include <TransitionEvent.h>
//#include <StandState.h>
//#include <IdleState.h>

namespace Robot
{
    namespace Nomad
    {
        namespace FSM
        {
            // Transition Events For States
            NomadControlTransitionEvent::NomadControlTransitionEvent(const std::string &name, std::shared_ptr<NomadControlData> data)
                : TransitionEvent(name), data_(data)
            {
                data_ = data;
            }

            NomadControlFSM::NomadControlFSM() : FiniteStateMachine("Nomad Primary Control FSM")
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

            void NomadControlFSM::_CreateFSM()
            {
                std::cout << "[NomadControlFSM]: Creating FSM" << std::endl;

                ///////////////////////// Define Our States
                // Off
                std::shared_ptr<OffState> off = std::make_shared<OffState>();
                

                // // Idle
                // std::shared_ptr<IdleState> idle = std::make_shared<IdleState>();
                // idle->SetGamepadInterface(gamepad_);

                // // Stand
                // std::shared_ptr<StandState> stand = std::make_shared<StandState>();
                // stand->SetGamepadInterface(gamepad_);

                // // Sit
                // std::shared_ptr<SitState> sit = std::make_shared<SitState>();
                // sit->SetGamepadInterface(gamepad_);

                // std::shared_ptr<ButtonEvent> start_event = std::make_shared<ButtonEvent>("START BUTTON", GamepadInterface::BUTTON_START, ButtonEvent::EVENT_PRESSED, gamepad_);
                // std::shared_ptr<DPadEvent> up_event = std::make_shared<DPadEvent>("UP BUTTON", GamepadInterface::DPadType::D_PAD_UP, gamepad_);
                // std::shared_ptr<DPadEvent> down_event = std::make_shared<DPadEvent>("DOWN BUTTON", GamepadInterface::DPadType::D_PAD_DOWN, gamepad_);

                // // Setup Transitions
                // off->AddTransitionEvent(start_event, idle);
                // idle->AddTransitionEvent(up_event, stand);
                // stand->AddTransitionEvent(down_event, sit);
                // sit->AddTransitionEvent(up_event, stand);

                // Add the stated to the FSM
                AddState(off);
                // AddState(idle);
                // AddState(stand);
                // AddState(sit);

                // Set Initials State
                SetInitialState(off);
            }
        } // namespace FSM
    }     // namespace Nomad
} // namespace Robot