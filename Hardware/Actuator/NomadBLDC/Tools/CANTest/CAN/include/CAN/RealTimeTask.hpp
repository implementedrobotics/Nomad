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
#include <memory>
#include <atomic>
#include <vector>

// Third Party Includes

// Project Includes
#include <CAN/Statistics.hpp>

namespace Realtime
{
    enum Priority
    {
        LOWEST = 1,
        LOW = 20,
        MEDIUM = 50,
        HIGH = 80,
        HIGHEST = 99
    };

    class RealTimeTaskNode
    {
        friend class RealTimeTaskManager;

    public:

        // Base Class Real Time Task Node
        // name = Task Name
        // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
        // rt_priority = Task Thread Priority -> Priority::MEDIUM,
        // rt_core_id = CPU Core to pin the task.  -1 for no affinity
        // stack_size = Task Thread Stack Size -> PTHREAD_STACK_MIN
        RealTimeTaskNode(const std::string &name,
                         const double rt_period,
                         const unsigned int rt_priority,
                         const int rt_core_id,
                         const unsigned int stack_size);

        ~RealTimeTaskNode();

        // Task Start
        int Start(void *task_param = NULL);

        // Task Stop
        void Stop();

        bool IsCancelled() const
        {
            return thread_cancel_event_;
        }

        // Set Task Name
        void SetTaskName(const std::string &name) { task_name_ = name; }

        // Set Stack Size
        void SetStackSize(const size_t stack_size) { stack_size_ = stack_size; }

        // Set Task Priority
        void SetTaskPriority(const unsigned int priority) { rt_priority_ = priority; }

        // Set Task Frequency (Convenience to set a "rate" in HZ)
        void SetTaskFrequency(const unsigned int freqeuncy_hz);

        // Set Task Period (Microseconds)
        void SetTaskPeriod(const double period) { 
            rt_period_ = period; 
        }

        // Set CPU Core Affinity
        void SetCoreAffinity(const int core_id) { rt_core_id_ = core_id; }

    protected:
        // Override Me for thread function
        virtual void Run() = 0;

        // Setup function called prior to run loop.  Put any setup/initialization here, i.e. socket setup, pub sub etc.
        virtual void Setup() = 0;
        
        // Exit Task.  Any task exit/cleanup happens here
        virtual void Exit() = 0;

        // Task Name
        std::string task_name_;

        // Stack Size
        size_t stack_size_;

        // Task Priority (0 to 99)
        unsigned int rt_priority_;

        // Task Period (seconds)
        double rt_period_;

        // Task Period Actual (seconds)
        double dt_actual_;

        // Timing Stats
        long unsigned int tick_count_;
        
        Statistics::RollingStats<double> stats;
        
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

        // Cancellation Signal
        std::atomic_bool thread_cancel_event_;

    protected:
        // STATIC Task Delay (More accurate but uses a busy wait)
        static long int TaskDelay(long int microseconds); // Usuful for tight timings or periods below 1000us

        // Static Task Sleep (Less accurate but less resource intensive) // Useful for sotter timings and periods > 1000us
        static long int TaskSleep(long int microseconds);
    
    private:
        // STATIC Member Task Run
        static void *RunTask(void *task_instance);
    };

    class RealTimeTaskManager
    {
        friend class RealTimeTaskNode;
    public:
        // Base Class Real Time Task Manager
        RealTimeTaskManager();

        // STATIC Singleton Instance
        static RealTimeTaskManager *Instance();

        // Number of CPU Cores available in the system
        int GetCPUCount() { return cpu_count_; }

        // Add Task to the Manager
        bool AddTask(RealTimeTaskNode *task);

        // End Task and Shutdown
        bool EndTask(RealTimeTaskNode *task);

        // End Task and Shutdown
        bool EndTask(const std::string &name);

        // Print the currently active task lists
        void PrintActiveTasks();

        // Sets up malloc and pagefault behavior for RT threads
        static bool EnableRTMemory(const unsigned int memory_size);

    private:

        // Get any current/new page faults.  Ideally this should be zero if RT thread is properly configured
        static void GetActivePageFaults(int &hard_faults, int &soft_faults, bool print_faults);

        // Touch full thread stack to make sure nothing page faults.
        static bool CheckThreadStackFaults(const unsigned int stack_size);

        // Touch full thread stack to make sure nothing page faults.
        static bool ReserveMemory(const unsigned int malloc_buffer_size);

        // Singleton Instance
        static RealTimeTaskManager *manager_instance_;

        // Vector to hold running tasks.  Assumed to not be THAT many running task.  Hence the std::vector
        std::vector<RealTimeTaskNode *> task_map_;

        // Max numbers of CPUs
        int cpu_count_;
    };
} // namespace Realtime

#endif // NOMAD_REALTIME_REALTIMETASK_H_