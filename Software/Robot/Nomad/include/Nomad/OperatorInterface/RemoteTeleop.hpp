/*
 * RemoteTelop.hpp
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

#ifndef NOMAD_OPERATORINTERFACE_REMOTETELEOP_H_
#define NOMAD_OPERATORINTERFACE_REMOTETELEOP_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>
#include <memory>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Nomad/OperatorInterface/GamepadInterface.hpp>

//#include <OperatorInterface/GamepadInterface.hpp>

#include <Nomad/OperatorInterface/GamepadTeleopFSM/GamepadTeleopFSM.hpp>
//#include <OperatorInterface/GamepadTeleopFSM/GamepadTeleopFSM.hpp>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/int32_vec_t.hpp>

// TODO: Evaluate the need for the class... Could be handled all in the Trajectory Generator.  But if latency permits this is a good intermediate layer to handle translation of network/gamepad calls etc.
// Also this a good place to put test trajectory setpoints since we don't have a remote control UI yet.
namespace OperatorInterface
{
    namespace Teleop
    {
        class RemoteTeleop : public Realtime::RealTimeTaskNode
        {

        public:
            enum OutputPort
            {
                MODE = 0,    // Desired Mode
                SETPOINT = 1 // Desired Setpoint
            };

            enum Idx
            {
                X_DOT = 0,
                Y_DOT = 1,
                YAW_DOT = 2,
                Z_COM = 3
            };

            // Base Class Remote Teleop Task Node
            // name = Task Name
            // stack_size = Task Thread Stack Size
            // rt_priority = Task Thread Priority
            // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
            // rt_core_id = CPU Core to pin the task.  -1 for no affinity
            RemoteTeleop(const std::string &name = "Remote_Teleop",
                         const long rt_period = 10000,
                         const unsigned int rt_priority = Realtime::Priority::MEDIUM,
                         const int rt_core_id = -1,
                         const unsigned int stack_size = PTHREAD_STACK_MIN);

        protected:
            // Overriden Run Function
            virtual void Run();

            // Pre-Run Setup Routine.  Setup any one time initialization here.
            virtual void Setup();

            // (Output) State Estimate
            double_vec_t output_setpoint_;

            // (Output) State Estimate
            int32_vec_t output_mode_;

            // Gamepad Interface
            std::shared_ptr<GamepadInterface> gamepad_;

            // Finite State Machine to handle gamepad state
            std::unique_ptr<GamepadTeleopFSM> gamepad_FSM_;
        };
    } // namespace Teleop
} // namespace OperatorInterface

#endif // NOMAD_OPERATORINTERFACE_REMOTETELEOP_H_
