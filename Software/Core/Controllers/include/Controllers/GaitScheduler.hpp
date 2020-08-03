/*
 * GaitScheduler.hpp
 *
 *  Created on: July 14, 2019
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

#ifndef NOMAD_CORE_CONTROLLERS_GAITSCHEDULER_H_
#define NOMAD_CORE_CONTROLLERS_GAITSCHEDULER_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>

// Project Include Files
#include <Realtime/RealTimeTask.hpp>

namespace Controllers::Locomotion
{
    class GaitScheduler : public Realtime::RealTimeTaskNode
    {

    public:
        enum OutputPort
        {
            CONTACT_STATE = 0 // Leg Contact State
        };

        enum InputPort
        {
        };

        // Base Class Gait Scheduler Task Node
        // name = Task Name
        // stack_size = Task Thread Stack Size
        // rt_priority = Task Thread Priority
        // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
        // rt_core_id = CPU Core to pin the task.  -1 for no affinity
        GaitScheduler(const std::string &name = "GaitScheduler_Task",
                      const long rt_period = 20000,
                      const unsigned int rt_priority = Realtime::Priority::HIGH,
                      const int rt_core_id = -1,
                      const unsigned int stack_size = PTHREAD_STACK_MIN);

    protected:
        // Overriden Run Function
        virtual void Run();

        // Pre-Run Setup Routine.  Setup any one time initialization here.
        virtual void Setup();

    private:
        int gait_schedule_sequence_num_;
    };
} // namespace Controllers::Locomotion

#endif // NOMAD_CORE_CONTROLLERS_GAITSCHEDULER_H_
