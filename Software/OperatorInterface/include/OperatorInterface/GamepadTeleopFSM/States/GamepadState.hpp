/*
 * GamepadState.hpp
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

#ifndef NOMAD_GAMEPADSTATE_H_
#define NOMAD_GAMEPADSTATE_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <Common/State.hpp>
#include <OperatorInterface/GamepadInterface.hpp>

namespace OperatorInterface
{
    namespace Teleop
    {
        // State Class
        class GamepadState : public Common::State
        {
        public:
            enum ControlMode
            {
                OFF = 0,
                IDLE = 1,
                STAND = 2,
                SIT = 3
            };

            // Base Class GamepadState
            GamepadState(const std::string &name, std::size_t id) : Common::State(name, id)
            {
            }

            void SetGamepadInterface(std::shared_ptr<GamepadInterface> gamepad)
            {
                gamepad_ = gamepad;
            }

            int GetMode()
            {
                return current_mode_;
            }

        protected:
            // Data pointer to controller data pointer
            std::shared_ptr<GamepadInterface> gamepad_;
            int current_mode_;
        };

        class OffState : public GamepadState
        {

        public:
            OffState();

            // Called upon a state change and we enter this state
            // current_time = current robot/controller time
            void Enter(double current_time);

            // // current_time = current robot/controller time
            // // Called upon a state change and we are exiting this state
            // void Exit(double current_time);

            // Logic to run each iteration of the state machine run
            // dt = time step for this iteration
            void Run(double dt);

        protected:
        };

        class IdleState : public GamepadState
        {

        public:
            IdleState();

            // Called upon a state change and we enter this state
            // current_time = current robot/controller time
            void Enter(double current_time);

            // // current_time = current robot/controller time
            // // Called upon a state change and we are exiting this state
            // void Exit(double current_time);

            // Logic to run each iteration of the state machine run
            // dt = time step for this iteration
            void Run(double dt);

        protected:
        };

        class StandState : public GamepadState
        {

        public:
            StandState();

            // Called upon a state change and we enter this state
            // current_time = current robot/controller time
            void Enter(double current_time);

            // // current_time = current robot/controller time
            // // Called upon a state change and we are exiting this state
            // void Exit(double current_time);

            // Logic to run each iteration of the state machine run
            // dt = time step for this iteration
            void Run(double dt);

        protected:
        };

        class SitState : public GamepadState
        {

        public:
            SitState();

            // Called upon a state change and we enter this state
            // current_time = current robot/controller time
            void Enter(double current_time);

            // // current_time = current robot/controller time
            // // Called upon a state change and we are exiting this state
            // void Exit(double current_time);

            // Logic to run each iteration of the state machine run
            // dt = time step for this iteration
            void Run(double dt);

        protected:
        };

    } // namespace Teleop
} // namespace OperatorInterface
#endif // NOMAD_GAMEPADSTATE_H_