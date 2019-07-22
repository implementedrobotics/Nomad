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
#include <Realtime/RealTimeTask.hpp>
#include <Controllers/Messages.hpp>



namespace Controllers
{

namespace Estimators
{

    // TODO: Static Variable in "Physics" Class somewhere
double kGravity = 9.81;
//using namespace RealTimeControl;

StateEstimator::StateEstimator(const std::string &name,
                               const long rt_period,
                               unsigned int rt_priority,
                               const int rt_core_id,
                               const unsigned int stack_size) : 
                               Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size),
                               sequence_num_(0),
                               num_states_(13)
{
    
    // Create Ports
    zmq::context_t *ctx = Realtime::RealTimeTaskManager::Instance()->GetZMQContext();

    // State Estimate Output Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    Realtime::Port *port = new Realtime::Port("STATE_HAT", ctx, "state", rt_period);
    output_port_map_[OutputPort::STATE_HAT] = port;
    
}

void StateEstimator::Run()
{
    // Estimate State
    //Messages::Controllers::Estimators::CoMState<13> output_state;
    Messages::Controllers::Estimators::CoMState output_state;

    // Get Timestamp
    // TODO: "GetUptime" Static function in a time class
    uint64_t time_now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    output_state.timestamp = time_now;
    output_state.sequence_number = sequence_num_;
    output_state.x[0] = 0.0; // X Position
    output_state.x[1] = 0.0; // Y Position
    output_state.x[2] = 0.0; // Z Position
    output_state.x[3] = 0.0; // X Velocity
    output_state.x[4] = 0.0; // Y Velocity
    output_state.x[5] = 0.0; // Z Velocity
    output_state.x[6] = 0.0; // Roll Orientation
    output_state.x[7] = 0.0; // Pitch Orientation
    output_state.x[8] = 2.0; // Yaw Orientation
    output_state.x[9] = 0.0; // Roll Rate
    output_state.x[10] = 0.0; // Pitch Rate
    output_state.x[11] = 0.0; // Yaw Rate
    output_state.x[12] = kGravity; // Gravity

    //std::cout << "State Size: " << sizeof(output_state) << std::endl;

    // Publish State
    GetOutputPort(0)->Send(&output_state, sizeof(output_state));
    //std::cout << "[StateEstimator]: Publishing: " << output_state.timestamp << " Send: " << send_status << std::endl;
    sequence_num_++;
}

void StateEstimator::Setup()
{
    GetOutputPort(0)->Bind();
    std::cout << "[StateEstimator]: " << "State Estimator Publisher Running!" << std::endl;
}

} // namespace Estimators
} // namespace Controllers
