/*
 * StateEstimator.cpp
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

// Primary Include
#include <Controllers/StateEstimator.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>

// Third-Party Includes

// Project Includes
#include <Controllers/RealTimeTask.hpp>

namespace Controllers
{

namespace Estimators
{
//using namespace RealTimeControl;

StateEstimator::StateEstimator(const std::string &name,
                               const long rt_period,
                               unsigned int rt_priority,
                               const int rt_core_id,
                               const unsigned int stack_size) : 
                               RealTimeControl::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size),
                               state_estimate_num_(0)
{
    
    // Create Ports
    zmq::context_t *ctx = RealTimeControl::RealTimeTaskManager::Instance()->GetZMQContext();

    // State Estimate Output Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    RealTimeControl::Port *port = new RealTimeControl::Port("STATE_HAT", ctx, "state", rt_period);
    output_port_map_[OutputPort::STATE_HAT] = port;
    
}

void StateEstimator::Run()
{
    // Estimate State
    CoMState output_state;

    output_state.x[0] = 1.0;
    output_state.x[1] = 2.0;
    output_state.x[2] = 31.0;
    output_state.x[3] = 40.0;
    output_state.x[4] = 555.0;
    output_state.x[5] = 66.0;
    output_state.x[6] = 7.0;
    output_state.x[7] = 8.0;
    output_state.x[8] = 9.0;
    output_state.x[9] = 10.0;
    output_state.x[10] = 11.0;
    output_state.x[11] = 12.0;
    output_state.x[12] = 13.0;

    std::cout << "State Size: " << sizeof(output_state) << std::endl;

    // Publish State
    // TODO: Zero Copy Publish
    //std::stringstream s;
    //s << "Hello " << state_estimate_num_;
    //auto msg = s.str();
    //const int length = sizeof(output_state);
    //zmq::message_t message(length);
    //memcpy(message.data(), &output_state, length);

    //GetOutputPort(0)->Send(message);
    GetOutputPort(0)->Send(&output_state, sizeof(output_state));
    std::cout << "[StateEstimator]: Publishing: " << output_state.x << std::endl;

    state_estimate_num_++;
}

void StateEstimator::Setup()
{
    GetOutputPort(0)->Bind();
    std::cout << "[StateEstimator]: " << "State Estimator Publisher Running!" << std::endl;
}

} // namespace Estimators
} // namespace Controllers
