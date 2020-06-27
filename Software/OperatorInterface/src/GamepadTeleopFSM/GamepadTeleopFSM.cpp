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

        void GamepadTeleopFSM::_CreateFSM()
        {
            std::cout << "[GamepadTeleopFSM]: Creating FSM" << std::endl;

            // ///////////////////////// Define Our States
            // // Idle
            // std::shared_ptr<IdleState> idle = std::make_shared<IdleState>();
            // idle->SetControllerData(control_DATA_);

            // // Stand
            // std::shared_ptr<StandState> stand = std::make_shared<StandState>();
            // stand->SetControllerData(control_DATA_);

            // std::shared_ptr<CommandRequestEvent> transitionStand = std::make_shared<CommandRequestEvent>("STAND TRANSITION", CONTROL_MODE::STAND, control_DATA_);
            // std::shared_ptr<CommandRequestEvent> transitionIdle = std::make_shared<CommandRequestEvent>("IDLE TRANSITION", CONTROL_MODE::IDLE, control_DATA_);

            // // Setup Transitions
            // idle->AddTransitionEvent(transitionStand, stand);
            // stand->AddTransitionEvent(transitionIdle, idle);

            // // Add the state to the FSM
            // AddState(idle);
            // AddState(stand);

            // // Set Initials State
            // SetInitialState(idle);

            // // Start the state machine
            // //Start();
        }
    } // namespace Teleop
} // namespace OperatorInterface