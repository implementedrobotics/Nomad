/*
 * GamepadTeleopFSM.h
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

#ifndef NOMAD_GAMEPADTELEOPFSM_H_
#define NOMAD_GAMEPADTELEOPFSM_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <Common/FiniteStateMachine.hpp>
#include <OperatorInterface/GamepadInterface.hpp>

namespace OperatorInterface
{
    namespace Teleop
    {
        // Finite State Machine Class
        class GamepadTeleopFSM : public Common::FiniteStateMachine
        {
        public:
            // Base Class Gamepad Teleop FSM
            GamepadTeleopFSM(std::shared_ptr<GamepadInterface> gamepad);

        protected:
            //Helper function to create state machine
            void _CreateFSM();

            std::shared_ptr<GamepadInterface> gamepad_;
        };

        class GamepadTransitionEvent : public Common::TransitionEvent
        {
        public:
            GamepadTransitionEvent(const std::string &name, std::shared_ptr<GamepadInterface> gamepad);

        protected:
            std::shared_ptr<GamepadInterface> gamepad_;
            std::string name_;
        };

        // Transitions
        class ButtonEvent : public GamepadTransitionEvent
        {
        public:
            
            enum EventType
            {
                EVENT_PRESSED = 0,
                EVENT_RELEASED = 1
            };

            // Base Class Transition Event
            // name = Transition Event name
            ButtonEvent(const std::string &name, 
            GamepadInterface::ButtonType button, 
            EventType event, 
            std::shared_ptr<GamepadInterface> gamepad) : GamepadTransitionEvent(name, gamepad), button_(button), event_(event)
            {
            }

            // Stop state machine and cleans up
            bool Triggered()
            {
                // if (control_DATA_->control_mode_ == req_mode_)
                // {
                //     std::cout << "Event ID: " << name_ << " is SET!" << std::endl;
                //     return true;
                // }
                return false;
            };

        protected:
            GamepadInterface::ButtonType button_;
            EventType event_;
        };


    } // namespace Teleop
} // namespace OperatorInterface
#endif // NOMAD_GAMEPADTELEOPFSM_H_