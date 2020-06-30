/*
 * ControlFSM.hpp
 *
 *  Created on: June 30, 2020
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

#ifndef CORE_CONTROLLERS_FSM_PRIMARYCONTROL_H_
#define CORE_CONTROLLERS_FSM_PRIMARYCONTROL_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Controllers/LegController.hpp>

namespace Controllers
{
    namespace FSM
    {
        class PrimaryControl : public Realtime::RealTimeTaskNode
        {

        public:
            enum OutputPort
            {
                LEG_COMMAND = 0,
                NUM_INPUTS = 1
            };

            enum InputPort
            {
                NUM_OUTPUTS = 0
            };

            // Base Class State Estimator Task Node
            // name = Task Name
            // stack_size = Task Thread Stack Size
            // rt_priority = Task Thread Priority
            // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
            // rt_core_id = CPU Core to pin the task.  -1 for no affinity
            PrimaryControl(const std::string &name = "Primary_Control_FSM",
                           const long rt_period = 10000,
                           const unsigned int rt_priority = Realtime::Priority::MEDIUM,
                           const int rt_core_id = -1,
                           const unsigned int stack_size = PTHREAD_STACK_MIN);

        protected:
            // Overriden Run Function
            virtual void Run();

            // Pre-Run Setup Routine.  Setup any one time initialization here.
            virtual void Setup();

            // Input

            // Output
            generic_msg_t leg_command_msg_;
            Controllers::Locomotion::leg_controller_cmd_t leg_controller_cmd_;
        };
    } // namespace FSM
} // namespace Controllers

#endif // CORE_CONTROLLERS_FSM_PRIMARYCONTROL_H_
