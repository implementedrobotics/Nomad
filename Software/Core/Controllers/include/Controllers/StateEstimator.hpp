/*
 * StateEstimator.hpp
 *
 *  Created on: July 13, 2019
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

// C System Files

// C++ System Files
#include <iostream>
#include <string>

#include <Controllers/RealTimeTask.hpp>

#ifndef NOMAD_CORE_CONTROLLERS_STATEESTIMATOR_H_
#define NOMAD_CORE_CONTROLLERS_STATEESTIMATOR_H_

namespace Controllers
{
namespace Estimators
{
class StateEstimator : public RealTimeControl::RealTimeTaskNode
{

public:
    // Base Class State Estimator Task Node
    // name = Task Name
    // stack_size = Task Thread Stack Size
    // rt_priority = Task Thread Priority
    // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
    // rt_core_id = CPU Core to pin the task.  -1 for no affinity
    StateEstimator(const std::string &name = "State_Estimator_Task",
                   const long rt_period = 10000,
                   const unsigned int rt_priority = RealTimeControl::Priority::MEDIUM,
                   const int rt_core_id = -1,
                   const unsigned int stack_size = PTHREAD_STACK_MIN);

    void SetInputTransport(const std::string &transport) { input_transport_ = transport; }
    void SetOutputTransport(const std::string &transport) { output_transport_ = transport; }

    const std::string &GetInputTransport() const { return input_transport_; }
    const std::string &GetOutputTransport() const { return output_transport_; }

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    // Using ZMQ for thread sync and message passing
    // ZMQ Socket
    zmq::socket_t *socket_;

    // ZMQ Transport
    // TODO: Need a enum for types, i.e. TCP, UDP, IPC, INPROC
    // TODO: Also a port int
    // TODO: Also a Socket Type
    // TODO: Also Queue/Message Options, i.e. HWM and CONFLATE

    // Transport for input to State Estimator
    std::string input_transport_;

    // Transport for output of State Estimator
    std::string output_transport_;

private:
    int state_estimate_num_;
};
} // namespace Estimators
} // namespace Controllers

#endif // NOMAD_CORE_CONTROLLERS_STATEESTIMATOR_H_
