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


#ifndef NOMAD_REALTIME_REALTIMETASK_H_
#define NOMAD_REALTIME_REALTIMETASK_H_

// C Includes
#include <limits.h>
#include <pthread.h>

// C++ Includes
#include <iostream>
#include <string>
#include <map>

// Third Party Includes
#include <zcm/zcm-cpp.hpp>

// Project Includes
#include <Realtime/Port.hpp>

namespace Realtime
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
    friend class RealTimeTaskManager;

public:
    static const int MAX_PORTS = 16;
    // Base Class Real Time Task Node
    // name = Task Name
    // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
    // rt_priority = Task Thread Priority -> Priority::MEDIUM,
    // rt_core_id = CPU Core to pin the task.  -1 for no affinity
    // stack_size = Task Thread Stack Size -> PTHREAD_STACK_MIN
    RealTimeTaskNode(const std::string &name,
                     const long rt_period,
                     const unsigned int rt_priority,
                     const int rt_core_id,
                     const unsigned int stack_size);

    ~RealTimeTaskNode();

    // Task Start
    int Start(void *task_param = NULL);

    // Task Stop
    void Stop();

    // Set Task Name
    void SetTaskName(const std::string &name) { task_name_ = name; }

    // Set Stack Size
    void SetStackSize(const unsigned int stack_size) { stack_size_ = stack_size; }

    // Set Task Priority
    void SetTaskPriority(const unsigned int priority) { rt_priority_ = priority; }

    // Set Task Frequency (Convenience to set a "rate" in HZ)
    void SetTaskFrequency(const unsigned int freqeuncy_hz);

    // Set Task Period (Microseconds)
    void SetTaskPeriod(const long period) { rt_period_ = period; }

    // Set CPU Core Affinity
    void SetCoreAffinity(const int core_id) { rt_core_id_ = core_id; }

    // Get Output Port
    Port* GetOutputPort(const int port_id) const;

    // Get Input Port
    Port* GetInputPort(const int port_id) const;

    // Set Topic
    // TODO: Add a "type" (TCP/UDP, THREAD ETC)
    void SetPortOutput(const int port_id, const std::string& path);

protected:
    // Override Me for thread function
    virtual void Run() = 0;

    // Setup function called prior to run loop.  Put any setup/initialization here, i.e. socket setup, pub sub etc.
    virtual void Setup() = 0;

    // Input Port Map
    std::vector<Port *> input_port_map_;

    // Output Port Map
    std::vector<Port *> output_port_map_;

    // Task Name
    std::string task_name_;

    // Stack Size
    unsigned int stack_size_;

    // Task Priority (0 to 99)
    unsigned int rt_priority_;

    // Task Period (microseconds)
    long rt_period_;

    // Task CPU Affinboolity/CoreID
    int rt_core_id_;

    // Thread ID
    pthread_t thread_id_;

    // Process ID
    pid_t process_id_;

    // Thread Status
    int thread_status_;

    // Task Parameter
    void *task_param_;



private:
    // STATIC Task Delay (More accurate but uses a busy wait)
    static long int TaskDelay(long int microseconds); // Usuful for tight timings or periods below 1000us
    // Static Task Sleep (Less accurate but less resource intensive) // Useful for sotter timings and periods > 1000us
    static long int TaskSleep(long int microseconds);



    // STATIC Member Task Run
    static void *RunTask(void *task_instance);


};

class RealTimeTaskManager
{

public:
    // Base Class Real Time Task Manager
    RealTimeTaskManager();

    // STATIC Singleton Instance
    static RealTimeTaskManager *Instance();

    int GetCPUCount() { return cpu_count_; }

    bool AddTask(RealTimeTaskNode *task);
    bool EndTask(RealTimeTaskNode *task);
    bool EndTask(const std::string &name);
    void PrintActiveTasks();

    // Context
    zcm::ZCM *GetZCMContext() const { return context_; }

protected:
    // Using ZCM for thread sync and message passing
    // ZCM Context
    // Thread inproc messaging must share a single context.  We put it in the singleton here to keep it unique;
    zcm::ZCM *context_;

private:
    // Singleton Instance
    static RealTimeTaskManager *manager_instance_;

    // Vector to hold running tasks.  Assumed to not be THAT many running task.  Hence the std::vector
    std::vector<RealTimeTaskNode *> task_map_;

    // Max numbers of CPUs
    int cpu_count_;
};
} // namespace Realtime

#endif // NOMAD_REALTIME_REALTIMETASK_H_
