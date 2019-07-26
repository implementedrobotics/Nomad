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
#include <zcm/zcm-cpp.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Controllers/Messages.hpp>
#include <Controllers/Messages/msg_t.hpp>


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
    // Create Message
    output_state_.data = new double[num_states_];
    output_state_.size = sizeof(double) * num_states_;

    // Create Ports
    // State Estimate Output Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    Realtime::PortImpl<msg_t> *port = new Realtime::PortImpl<msg_t> ("STATE_HAT", rt_period);
    output_port_map_[OutputPort::STATE_HAT] = port;
    
}

void StateEstimator::Run()
{
    // Estimate State
    // Get Timestamp
    // TODO: "GetUptime" Static function in a time class
    //uint64_t time_now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    //output_state.timestamp = time_now;
    //output_state.sequence_number = sequence_num_;
    // output_state_.data[0] = 1.0; // X Position
    // output_state_.data[1] = 2.0; // Y Position
    // output_state_.data[2] = 3.0; // Z Position
    // output_state_.data[3] = 4.0; // X Velocity
    // output_state_.data[4] = 5.0; // Y Velocity
    // output_state_.data[5] = 6.0; // Z Velocity
    // output_state_.data[6] = 7.0; // Roll Orientation
    // output_state_.data[7] = 0.0; // Pitch Orientation
    // output_state_.data[8] = 2.0; // Yaw Orientation
    // output_state_.data[9] = 0.0; // Roll Rate
    // output_state_.data[10] = 0.0; // Pitch Rate
    // output_state_.data[11] = 0.0; // Yaw Rate
    // output_state_.data[12] = kGravity; // Gravity

    //std::cout << "State Estimator Send: " << std::endl;
    // Publish State
    msg_t my_data {};
    my_data.str = (char*)"HELLO, WORLD!\n";

    bool send_status = GetOutputPort(0)->Send<msg_t>(my_data);
    //std::cout << "[StateEstimator]: Publishing: " << " " << " Send: " << send_status << std::endl;
    sequence_num_++;
}

void StateEstimator::Setup()
{
    GetOutputPort(0)->Bind();
    std::cout << "[StateEstimator]: " << "State Estimator Publisher Running!" << std::endl;
}

} // namespace Estimators
} // namespace Controllers
