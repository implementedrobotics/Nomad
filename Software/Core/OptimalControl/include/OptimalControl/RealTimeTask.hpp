/*
 * RealTimeTask.hpp
 *
 *  Created on: July 12, 2019
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

#include <limits.h>
#include <pthread.h>
#include <sched.h>

#include <iostream>
#include <string>

#ifndef NOMAD_CORE_OPTIMALCONTROL_REALTIMETASK_H_
#define NOMAD_CORE_OPTIMALCONTROL_REALTIMETASK_H_

namespace OptimalControl
{
namespace RealTimeControl
{

enum Priority
{
    LOWEST = 99,
    LOW = 80,
    MEDIUM = 50,
    HIGH = 20,
    HIGHEST = 1
};

class RealTimeTaskNode
{

public:
    // Base Class Real Time Task Node
    // name = Task Name
    // stack_size = Task Thread Stack Size
    // rt_priority = Task Thread Priority
    // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
    // rt_core_id = CPU Core to pin the task.  -1 for no affinity
    RealTimeTaskNode(const std::string &name = "RT_Task",
                     const unsigned int stack_size = PTHREAD_STACK_MIN,
                     const unsigned int rt_priority = Priority::MEDIUM,
                     const long rt_period = 10000,
                     const int rt_core_id = -1);

    // Task Delay Helper
    virtual void delay(int microseconds);

    // Task Start
    int start(void *task_param);

    // Task Stop
    void stop();

    // Set Task Name
    void setTaskName(const std::string &name) { task_name_ = name; }

    // Set Stack Size
    void setStackSize(const unsigned int stack_size) { stack_size_ = stack_size; }

    // Set Task Priority
    void setTaskPriority(const unsigned int priority) { rt_priority_ = priority; }

    // Set Task Period
    void setTaskPeriod(const long period) { rt_period_ = period; }

    // Set CPU Core Affinity
    void setCoreAffinity(const int core_id) { rt_core_id_ = core_id; }

    // Override Me for thread function
    virtual void run(void *data) = 0;

    // STATIC Member Task Run
    static void *runTask(void *task_instance);

private:
    // Task Name
    std::string task_name_;

    // Stack Size
    unsigned int stack_size_;

    // Task Priority (0 to 99)
    unsigned int rt_priority_;

    // Task Period
    long rt_period_;

    // Task CPU Affinity/CoreID
    int rt_core_id_;

    // Thread ID
    pthread_t thread_id;

    // Thread Status
    int thread_status_;

    // Task Parameter
    void *task_param_;
};
} // namespace RealTimeControl
} // namespace OptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_REALTIMETASK_H_
