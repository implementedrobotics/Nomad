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

#include <CAN/RealTimeTask.hpp>

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <assert.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <map>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/timerfd.h>
#include <malloc.h>



// Timing comparison
#define tscmp(a, b, CMP) \
    (((a)->tv_sec == (b)->tv_sec) ? ((a)->tv_nsec CMP(b)->tv_nsec) : ((a)->tv_sec CMP(b)->tv_sec))
#define tsadd(a, b, result)                              \
    do                                                   \
    {                                                    \
        (result)->tv_sec = (a)->tv_sec + (b)->tv_sec;    \
        (result)->tv_nsec = (a)->tv_nsec + (b)->tv_nsec; \
        if ((result)->tv_nsec >= 1000000000)             \
        {                                                \
            ++(result)->tv_sec;                          \
            (result)->tv_nsec -= 1000000000;             \
        }                                                \
    } while (0)
#define tssub(a, b, result)                              \
    do                                                   \
    {                                                    \
        (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;    \
        (result)->tv_nsec = (a)->tv_nsec - (b)->tv_nsec; \
        if ((result)->tv_nsec < 0)                       \
        {                                                \
            --(result)->tv_sec;                          \
            (result)->tv_nsec += 1000000000;             \
        }                                                \
    } while (0)

namespace Realtime
{
    RealTimeTaskNode::RealTimeTaskNode(const std::string &name,
                                       const double rt_period,
                                       unsigned int rt_priority,
                                       const int rt_core_id,
                                       const unsigned int stack_size) : task_name_(name),
                                                                        rt_period_(rt_period),
                                                                        rt_priority_(rt_priority),
                                                                        rt_core_id_(rt_core_id),
                                                                        stack_size_(stack_size),
                                                                        thread_status_(-1),
                                                                        process_id_(-1),
                                                                        tick_count_(0),
                                                                        thread_cancel_event_(false)
    {
        // Add to task manager
        RealTimeTaskManager::Instance()->AddTask(this);
    }

    RealTimeTaskNode::~RealTimeTaskNode()
    {
        // Remove from task manager
        RealTimeTaskManager::Instance()->EndTask(this);
    }

    void *RealTimeTaskNode::RunTask(void *task_instance)
    {
        RealTimeTaskNode *task = static_cast<RealTimeTaskNode *>(task_instance);
        std::cout << "[RealTimeTaskNode]: "
                  << "Starting Task: " << task->task_name_ << std::endl;

        // Clear page faults from thread creation
        int hard_faults, soft_faults;
        RealTimeTaskManager::GetActivePageFaults(hard_faults, soft_faults, false);

        // Check Stack Page Faults
        if (!RealTimeTaskManager::CheckThreadStackFaults(task->stack_size_))
        {
            std::cout << "[RealTimeTaskManager::RunTask]: ERROR: Active Page Faults Found.  Realtime execution NOT guaranteed! Hard: " << std::endl;
        }

        task->process_id_ = getpid();

        pthread_attr_t attr;
        size_t stacksize;
        pthread_getattr_np(pthread_self(), &attr);
        pthread_attr_getstacksize(&attr, &stacksize);

        // Verify Stack Size
        if (stacksize != task->stack_size_)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "Stack Task: " << task->task_name_ << std::endl;
        }

        // TODO: Do we want to have support for adding to multiple CPUs here?
        // TODO: Task manager keeps up with which task are pinned to which CPUs.  Could make sure high priority task maybe get pinned to unused cores?
        cpu_set_t cpu_set;
        if (task->rt_core_id_ >= 0 && task->rt_core_id_ < RealTimeTaskManager::Instance()->GetCPUCount())
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "Setting Thread Affinity to CPU CORE: " << task->rt_core_id_ << std::endl;

            // Clear out CPU set type
            CPU_ZERO(&cpu_set);

            // Set CPU Core to CPU SET
            CPU_SET(task->rt_core_id_, &cpu_set);

            // Set CPU Core Affinity to desired cpu_set
            const int set_result = pthread_setaffinity_np(task->thread_id_, sizeof(cpu_set_t), &cpu_set);
            if (set_result != 0)
            {
                std::cout << "[RealTimeTaskNode]: "
                          << "Failed to set thread affinity: " << set_result << std::endl;
            }

            // Verify it was set successfully
            if (CPU_ISSET(task->rt_core_id_, &cpu_set))
            {
                std::cout << "[RealTimeTaskNode]: "
                          << "Successfully set thread " << task->thread_id_ << " affinity to CORE: " << task->rt_core_id_ << std::endl;
            }
            else
            {
                std::cout << "[RealTimeTaskNode]: "
                          << "Failed to set thread " << task->thread_id_ << " affinity to CORE: " << task->rt_core_id_ << std::endl;
            }
        }
        else if (task->rt_core_id_ >= RealTimeTaskManager::Instance()->GetCPUCount())
        {
            std::cout << "[RealTimeTaskNode]: " << task->task_name_
                      << "\tERROR.  Desired CPU Affinity exceeds number of available cores!" << std::endl
                      << "Please check system configuration." << std::endl;
        }

        // Output what the actually task thread affinity is
        const int set_result = pthread_getaffinity_np(task->thread_id_, sizeof(cpu_set_t), &cpu_set);
        if (set_result != 0)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "Failed to get thread affinity: " << set_result << std::endl;
        }

        std::cout << "[RealTimeTaskNode]: " << task->task_name_ << " running on CPU CORES: \t[";
        for (int j = 0; j < CPU_SETSIZE; j++)
        {
            if (CPU_ISSET(j, &cpu_set))
            {
                std::cout << j << "\t";
            }
        }
        std::cout << "]" << std::endl;

        // Setup thread cancellation.
        // TODO: Look into PTHREAD_CANCEL_DEFERRED
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

        // Call Setup
        task->Setup();

        // Setup timing stats
        Statistics::RollingStats<double> stats;

        // Create Timer for thread period sleep
        int fd = timerfd_create(CLOCK_MONOTONIC, 0);
        int secs = (int)task->rt_period_;
        int nanosecs = (int)(std::fmod(task->rt_period_, 1.0)*1e9);
        
        itimerspec timer;
        timer.it_interval.tv_sec = secs;
        timer.it_value.tv_sec = secs;
        timer.it_value.tv_nsec = nanosecs;
        timer.it_interval.tv_nsec = nanosecs;

        timerfd_settime(fd, 0, &timer, nullptr);

        uint64_t num_exp = 0;

        // TODO:  Check Control deadlines as well.  If run over we can throw exception here
        while (1)
        {
            //{
            //Systems::Time t;
            if (task->IsCancelled())
            {
                task->Exit();
                break;
            }
            auto start = std::chrono::high_resolution_clock::now();
            task->Run();
            auto elapsed = std::chrono::high_resolution_clock::now() - start;

            // Wait for timer to lapse
            if(read(fd, &num_exp, sizeof(uint64_t)) < 1)
            {
                // Error In Timer Read
            }
            
            auto total_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
            
            stats.Add((double)1.0 / ((total_elapsed) * 1e-6) );

            task->dt_actual_ = total_elapsed * 1e-6;

           // std::cout << "Name: " << task->task_name_ << " | " << stats.StandardDeviation() << " | " << stats.Mean() << " | " << task->rt_period_ << " | " << total_elapsed <<  std::endl;
           // std::cout << "Stats : " << std::setw(6) << stats.Min() << " | " << std::setw(6) << stats.Mean() << " | " << std::setw(6) << stats.Max() << " | " << std::setw(6) << stats.StandardDeviation() << std::endl;
            
            // Do timing statistics
            task->tick_count_++;

            //std::cout << "Period: " << task->rt_period_ << std::endl;
            //std::cout << "Target: " <<  task->rt_period_ - total_us << " Overrun: " << remainder << " Total: " << task->rt_period_ - total_us + remainder << std::endl;
             if((task->tick_count_ % 2500) == 0)
            {
               std::cout << "Total Task Time: " <<  total_elapsed << " Frequency: " <<  1.0 / ((total_elapsed) * 1e-6) << " HZ" << std::endl;
                //std::cout << "Stats : " << stats.Min() << " | " << stats.Mean() << " | " << stats.StandardDeviation() << " | " << stats.Max() << std::endl;
                std::cout << "Stats : " << std::setw(6) << stats.Min() << " | " << std::setw(6) << stats.Mean() << " | " << std::setw(6) << stats.Max() << " | " << std::setw(6) << stats.StandardDeviation() << std::endl;
           }
            //std::cout << "Missed: " << missed << std::endl;
            
            //}
        }
        std::cout << "[RealTimeTaskNode]: "
                  << "Ending Task: " << task->task_name_ << std::endl;

        // Stop the task
        pthread_exit(NULL);
    }

    int RealTimeTaskNode::Start(void *task_param)
    {
        struct sched_param param;
        pthread_attr_t attr;

        // Set Task Thread Paramters
        task_param_ = task_param;

        // Initialize default thread attributes
        thread_status_ = pthread_attr_init(&attr);
        if (thread_status_)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "POSIX Thread failed to init attributes!" << std::endl;
            return thread_status_;
        }

        // Set Stack Size
        thread_status_ = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + stack_size_);
        if (thread_status_)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "POSIX Thread failed to set stack size!" << std::endl;
            return thread_status_;
        }
        // TODO: Back off RT Priority until PREEMPT Kernel.
        // Set Scheduler Policy to RT(SCHED_FIFO)
        thread_status_ = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        if (thread_status_)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "POSIX Thread failed to set schedule policy!" << std::endl;
            return thread_status_;
        }
        // // Set Thread Priority
        param.sched_priority = rt_priority_;
        thread_status_ = pthread_attr_setschedparam(&attr, &param);
        if (thread_status_)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "POSIX Thread failed to set thread priority!" << std::endl;
            return thread_status_;
        }

        // Use Scheduling Policy from Attributes.
        thread_status_ = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        if (thread_status_)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "POSIX Thread failed to set scheduling policy from attributes!" << std::endl;
            return thread_status_;
        }

        // Create our pthread.  Pass an instance of 'this' class as a parameter
        thread_status_ = pthread_create(&thread_id_, &attr, &RunTask, this);
        if (thread_status_)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "POSIX Thread failed to create thread!" << std::endl;
            return thread_status_;
        }

        // Set as Detached
        thread_status_ = pthread_detach(thread_id_);
        if (thread_status_)
        {
            std::cout << "[RealTimeTaskNode]: "
                      << "POSIX Thread failed to detach thread!" << std::endl;
            return thread_status_;
        }

        // Give thread some time to init
        usleep(1e5);
        return thread_status_;
    }

    void RealTimeTaskNode::Stop()
    {
        // TODO: Wait here for full stop?
        thread_cancel_event_ = true;
    }
    long int RealTimeTaskNode::TaskDelay(long int microseconds)
    {
        struct timespec now;
        struct timespec then;
        struct timespec start;
        struct timespec sleep;
        struct timespec remainder;
        long int nanoseconds = microseconds * 1e3;
        if (nanoseconds > 999999999)
        {
            return microseconds;
        }
        clock_gettime(CLOCK_MONOTONIC_RAW, &start);
        now = start;
        sleep.tv_sec = 0;
        sleep.tv_nsec = nanoseconds;
        tsadd(&start, &sleep, &then);
        while (tscmp(&now, &then, <))
        {
            clock_gettime(CLOCK_MONOTONIC_RAW, &now);
        }
        tssub(&now, &start, &remainder);

        return (remainder.tv_nsec / 1000) - microseconds;
    }
    long int RealTimeTaskNode::TaskSleep(long int microseconds)
    {
        // TODO: Absolute wait or relative?
        // For now we use relative.
        struct timespec delay;
        struct timespec remainder;
        struct timespec ats;

        // Setup Delay Timespec
        delay.tv_sec = 0;
        delay.tv_nsec = microseconds * 1e3;
        if (delay.tv_nsec >= 1000000000) // Wrap Overruns
        {
            delay.tv_nsec -= 1000000000;
            delay.tv_sec++;
        }

        // Get the start time of the delay.  To be used to know over and underruns
        clock_gettime(CLOCK_MONOTONIC, &ats);

        // Add the delay offset to know when this SHOULD end.
        ats.tv_sec += delay.tv_sec;
        ats.tv_nsec += delay.tv_nsec;

        // Sleep for delay
        clock_nanosleep(CLOCK_MONOTONIC, 0, &delay, &remainder);

        // Get the time now after the delay.
        clock_gettime(CLOCK_MONOTONIC, &remainder);


        // Compute difference and calculcate over/underrun
        remainder.tv_sec -= ats.tv_sec;
        remainder.tv_nsec -= ats.tv_nsec;
        if (remainder.tv_nsec < 0) // Wrap nanosecond overruns
        {
            remainder.tv_nsec += 1000000000;
            remainder.tv_sec++;
        }
        return remainder.tv_nsec / 1000;
    }

    void RealTimeTaskNode::SetTaskFrequency(const unsigned int frequency_hz)
    {
        // Frequency must be greater than 0
        assert(frequency_hz > 0);

        // Period in microseconds
        SetTaskPeriod(long((1.0 / frequency_hz) * 1e6));
    }

    // Task Manager Source

    // Global static pointer used to ensure a single instance of the class.
    RealTimeTaskManager *RealTimeTaskManager::manager_instance_ = NULL;

    RealTimeTaskManager::RealTimeTaskManager()
    {
        // Get CPU Count
        cpu_count_ = sysconf(_SC_NPROCESSORS_ONLN);

        std::cout << "[RealTimeTaskManager]: Task manager RUNNING.  Total Number of CPUS available: " << cpu_count_ << std::endl;
    }

    RealTimeTaskManager *RealTimeTaskManager::Instance()
    {
        if (manager_instance_ == NULL)
        {
            manager_instance_ = new RealTimeTaskManager();
        }
        return manager_instance_;
    }

    bool RealTimeTaskManager::AddTask(RealTimeTaskNode *task)
    {
        assert(task != NULL);

        for (int i = 0; i < task_map_.size(); i++)
        {
            if (task_map_[i] == task)
            {
                std::cout << "[RealTimeTaskManager]: Task " << task->task_name_ << " already exists." << std::endl;
                return false;
            }
        }

        task_map_.push_back(task);

        std::cout << "[RealTimeTaskManager]: Task " << task->task_name_ << " successfully added." << std::endl;
        return true;
    }

    bool RealTimeTaskManager::EndTask(RealTimeTaskNode *task)
    {
        assert(task != NULL);

        for (int i = 0; i < task_map_.size(); i++)
        {
            if (task_map_[i] == task)
            {
                task->Stop();
                task_map_.erase(task_map_.begin() + i);
                std::cout << "[RealTimeTaskManager]: Task " << task->task_name_ << " successfully removed" << std::endl;
                return true;
            }
        }

        std::cout << "[RealTimeTaskManager]: No Task " << task->task_name_ << " currently running" << std::endl;
        return false;
    }

    void RealTimeTaskManager::PrintActiveTasks()
    {
        for (auto task : task_map_)
        {
            std::cout << "[RealTimeTaskManager]: Task: " << task->task_name_ << "\tPriority: " << task->rt_priority_ << "\tCPU Affinity: " << task->rt_core_id_ << std::endl;
        }
    }
    bool RealTimeTaskManager::EnableRTMemory(const unsigned int memory_size)
    {
        int hard_faults = 0, soft_faults = 0;
        //GetActivePageFaults(hard_faults, soft_faults, false);

        // Setup malloc behavior
        if (mlockall(MCL_CURRENT | MCL_FUTURE))
        {
            std::cout << "[RealTimeTaskManager::EnableRTMemory]: mlockall FAILED.  This could be due to insufficient priveleges(sudo)" << std::endl;
            return false;
        }

        // Disable malloc trimming ( Keep memory.  Do not return to Kernel)
        mallopt(M_TRIM_THRESHOLD, -1);

        // Disable memory map usage ( No virtual memory )
        mallopt(M_MMAP_MAX, 0);

        //GetActivePageFaults(hard_faults, soft_faults, false);

        if (!ReserveMemory(memory_size))
        {
            std::cout << "[RealTimeTaskManager::ReserveMemory]: Unable to reserve memory buffer of size : " << memory_size << " bytes" << std::endl;
            return false;
        }

        GetActivePageFaults(hard_faults, soft_faults, false);

        if (!ReserveMemory(memory_size))
        {
            std::cout << "[RealTimeTaskManager::ReserveMemory]: Unable to reserve memory buffer of size : " << memory_size << " bytes" << std::endl;
            return false;
        }

        // Should be 0, 0 here
        GetActivePageFaults(hard_faults, soft_faults, false);
        if (hard_faults > 0 || soft_faults > 0)
        {
            std::cout << "[RealTimeTaskManager::ReserveMemory]: ERROR: Active Page Faults Found.  Realtime execution NOT guaranteed! Hard: " << hard_faults << " Soft: " << soft_faults << std::endl;
            return false;
        }

        return true;
    }

    bool RealTimeTaskManager::ReserveMemory(const unsigned int buffer_size)
    {
        uint8_t *buffer = (uint8_t *)malloc(buffer_size);
        if (buffer == nullptr)
        {
            return false;
        }

        // Fully touch all pages in this allocation to ensure everything is mapped into RAM
        for (int i = 0; i < buffer_size; i += sysconf(_SC_PAGESIZE))
        {
            buffer[i] = 0;
        }

        // Release our buffer. malloc should be configured such that the memory is NOT released back to the
        // Kernel.  Therefore any new allocations will not go to swap and will come from this memory pool locked to our process
        free(buffer);
        return true;
    }

    void RealTimeTaskManager::GetActivePageFaults(int &hard_faults, int &soft_faults, bool print_faults = true)
    {
        static int prev_hard_faults = 0, prev_soft_faults = 0;

        // Usage struct
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);

        // Update NEW fault count values
        hard_faults = usage.ru_majflt - prev_hard_faults;
        soft_faults = usage.ru_minflt - prev_soft_faults;

        if (print_faults)
            std::cout << "[RealTimeTaskManager::EnableRTMemory]: Active Page Faults: MAJOR: " << hard_faults << " "
                      << "MINOR: " << soft_faults << std::endl;

        // Update Fault Counts
        prev_hard_faults = usage.ru_majflt;
        prev_soft_faults = usage.ru_minflt;
    }

    bool RealTimeTaskManager::CheckThreadStackFaults(const unsigned int stack_size)
    {
        volatile char buffer[stack_size];
        int hard_faults = 0, soft_faults = 0;

        // Fully touch all pages in this stack allocation to ensure everything is mapped into RAM
        for (int i = 0; i < stack_size; i += sysconf(_SC_PAGESIZE))
        {
            buffer[i] = 0;
        }

        // Should generate ZERO page faults.
        GetActivePageFaults(hard_faults, soft_faults, false);

        // Check it
        if (hard_faults > 0 || soft_faults > 0)
        {
            return false;
        }

        return true;
    }
} // namespace Realtime