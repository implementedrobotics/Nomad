/*
 * RealTimeTask.cpp
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

#include <OptimalControl/RealTimeTask.hpp>

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <iostream>
#include <string>

// TODO: Add proper sleep/delay functions
// TODO: Verify Task Frequency
// TODO: Run FIFO without root lookup
// TODO: Setup CPU Sets properly for affinity if set
// TODO: Create a subclassed task class
// TODO: Add in ZMQ Messaging+Context Passing and Test
namespace OptimalControl
{
namespace RealTimeControl
{
RealTimeTaskNode::RealTimeTaskNode(const std::string &name,
                                   const unsigned int stack_size,
                                   unsigned int rt_priority,
                                   const long rt_period,
                                   const int rt_core_id) : task_name_(name),
                                                           stack_size_(stack_size),
                                                           rt_priority_(rt_priority),
                                                           rt_period_(rt_period),
                                                           rt_core_id_(rt_core_id),
                                                           thread_status_(-1)
{
}

void *RealTimeTaskNode::runTask(void *task_instance)
{
    // TODO: Setup CPU sets, etc.
    RealTimeTaskNode *task = static_cast<RealTimeTaskNode *>(task_instance);
    std::cout << "RealTimeTaskNode: "
              << "Starting Task: " << task->task_name_ << std::endl;

    task->run(task->task_param_);

    std::cout << "RealTimeTaskNode: "
              << "Ending Task: " << task->task_name_ << std::endl;

    // Stop the task
    task->stop();
}

int RealTimeTaskNode::start(void *task_param)
{
    struct sched_param param;
    pthread_attr_t attr;

    // Initialize default thread attributes
    thread_status_ = pthread_attr_init(&attr);
    if (thread_status_)
    {
        std::cout << "RealTimeTaskNode: "
                  << "POSIX Thread failed to init attributes!" << std::endl;
        return thread_status_;
    }

    // Set Stack Size
    thread_status_ = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (thread_status_)
    {
        std::cout << "RealTimeTaskNode: "
                  << "POSIX Thread failed to set stack size!" << std::endl;
        return thread_status_;
    }

    // Set Scheduler Policy to RT(SCHED_FIFO)
    thread_status_ = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (thread_status_)
    {
        std::cout << "RealTimeTaskNode: "
                  << "POSIX Thread failed to set schedule policy!" << std::endl;
        return thread_status_;
    }
    // Set Thread Priority
    param.sched_priority = rt_priority_;
    thread_status_ = pthread_attr_setschedparam(&attr, &param);
    if (thread_status_)
    {
        std::cout << "RealTimeTaskNode: "
                  << "POSIX Thread failed to set thread priority!" << std::endl;
        return thread_status_;
    }

    // Use Scheduling Policy from Attributes.
    thread_status_ = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (thread_status_)
    {
        std::cout << "RealTimeTaskNode: "
                  << "POSIX Thread failed to set scheduling policy from attributes!" << std::endl;
        return thread_status_;
    }

    // Create our pthread.  Pass an instance of 'this' class as a parameter
    thread_status_ = pthread_create(&thread_id, &attr, &runTask, this);
    if (thread_status_)
    {
        std::cout << "RealTimeTaskNode: "
                  << "POSIX Thread failed to create thread!" << std::endl;
        return thread_status_;
    }

    // Set as Detached
    thread_status_ = pthread_detach(thread_id);
    if (thread_status_)
        std::cout << "RealTimeTaskNode: "
                  << "POSIX Thread failed to detach thread!" << std::endl;
}

void RealTimeTaskNode::stop() {
    // TODO: Check for thread running, etc.
    // TODO: Setup signal to inform run task to stop
}

void RealTimeTaskNode::delay(int microseconds)
{
    usleep(microseconds);
}
} // namespace RealTimeControl
} // namespace OptimalControl