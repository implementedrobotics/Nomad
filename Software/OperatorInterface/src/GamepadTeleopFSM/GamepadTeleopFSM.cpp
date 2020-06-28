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
#include <OperatorInterface/GamepadTeleopFSM/GamepadTeleopFSM.hpp>
#include <OperatorInterface/GamepadTeleopFSM/States/GamepadState.hpp>

//#include <TransitionEvent.h>
//#include <StandState.h>
//#include <IdleState.h>

namespace OperatorInterface
{
    namespace Teleop
    {
        // Transition Events For States
        GamepadTransitionEvent::GamepadTransitionEvent(const std::string &name, std::shared_ptr<GamepadInterface> gamepad)
            : TransitionEvent(name), gamepad_(gamepad), name_(name)
        {
            //gamepad_ = control_data;
        }

        
        GamepadTeleopFSM::GamepadTeleopFSM(std::shared_ptr<GamepadInterface> gamepad) : FiniteStateMachine("Nomad Primary Control FSM"), gamepad_(gamepad)
        {
            _CreateFSM();
        }
        bool GamepadTeleopFSM::Run(double dt)
        {
            // Poll Gamepad
            gamepad_->Poll();

            // Now run base FSM code
            return FiniteStateMachine::Run(dt);
        }
        void GamepadTeleopFSM::_CreateFSM()
        {
            std::cout << "[GamepadTeleopFSM]: Creating FSM" << std::endl;

            ///////////////////////// Define Our States
            // Off
            std::shared_ptr<OffState> off = std::make_shared<OffState>();
            off->SetGamepadInterface(gamepad_);

            // Idle
            std::shared_ptr<IdleState> idle = std::make_shared<IdleState>();
            idle->SetGamepadInterface(gamepad_);

            // Stand
            std::shared_ptr<StandState> stand = std::make_shared<StandState>();
            stand->SetGamepadInterface(gamepad_);

            std::shared_ptr<ButtonEvent> start_event = std::make_shared<ButtonEvent>("START BUTTON", GamepadInterface::BUTTON_START, ButtonEvent::EVENT_PRESSED, gamepad_);
            std::shared_ptr<DPadEvent> up_event = std::make_shared<DPadEvent>("UP BUTTON", GamepadInterface::DPadType::D_PAD_UP, gamepad_);

            // Setup Transitions
            off->AddTransitionEvent(start_event, idle);
            idle->AddTransitionEvent(up_event, stand);

            // Add the stated to the FSM
            AddState(off);
            AddState(idle);
            AddState(stand);

            // Set Initials State
            SetInitialState(off);
        }
    } // namespace Teleop
} // namespace OperatorInterface