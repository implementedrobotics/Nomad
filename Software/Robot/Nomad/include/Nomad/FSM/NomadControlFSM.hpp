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

#ifndef ROBOT_NOMAD_NOMADCONTROLFSM_H_
#define ROBOT_NOMAD_NOMADCONTROLFSM_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <Common/FiniteStateMachine.hpp>
#include <Nomad/FSM/NomadControlData.hpp>

namespace Robot
{
    namespace Nomad
    {
        namespace FSM
        {
            typedef enum
            {
                OFF = 0,
                IDLE = 1,
                PASSIVE = 2,
                STAND = 3,
                BALANCE = 4,
                LOCOMOTION = 5,
                JUMP = 6,
                ESTOP = 7
            } CONTROL_MODE;

            // Finite State Machine Class
            class NomadControlFSM : public Common::FiniteStateMachine
            {
            public:
                // Base Class Gamepad Teleop FSM
                NomadControlFSM();

                // Run an iteration of the state machine
                bool Run(double dt);

                // Return Mode from Active State
                const std::shared_ptr<NomadControlData> &GetData() const;

                //
            protected:
                //Helper function to create state machine
                void
                _CreateFSM();

                std::shared_ptr<NomadControlData> data_;
            };

            class NomadControlTransitionEvent : public Common::TransitionEvent
            {
            public:
                NomadControlTransitionEvent(const std::string &name, std::shared_ptr<NomadControlData> data);

            protected:
                std::shared_ptr<NomadControlData> data_;
                std::string name_;
            };

            // Transitions
            class CommandModeEvent : public NomadControlTransitionEvent
            {
            public:
                // Base Class Transition Event
                // name = Transition Event name
                CommandModeEvent(const std::string &name,
                                 CONTROL_MODE mode,
                                 std::shared_ptr<NomadControlData> data) : NomadControlTransitionEvent(name, data), req_mode_(mode)
                {
                }

                // Stop state machine and cleans up
                bool Triggered()
                {
                    if (data_->control_mode == req_mode_)
                    {
                        std::cout << "Event ID: " << name_ << " is SET!" << std::endl;
                        return true;
                    }
                    return false;
                };

            protected:
                CONTROL_MODE req_mode_;
            };
        } // namespace FSM
    }     // namespace Nomad
} // namespace Robot
#endif // ROBOT_NOMAD_NOMADCONTROLFSM_H_