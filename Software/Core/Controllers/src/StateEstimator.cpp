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
#include <memory>

// Third-Party Includes
#include <zcm/zcm-cpp.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Realtime/Messages/double_vec_t.hpp>


namespace Controllers
{
namespace Estimators
{

// TODO: Static Variable in "Physics" Class somewhere
double kGravity = 9.81;

StateEstimator::StateEstimator(const std::string &name,
                               const long rt_period,
                               unsigned int rt_priority,
                               const int rt_core_id,
                               const unsigned int stack_size) : 
                               Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size),
                               num_states_(13)
{  
    // Create Messages
    output_state_.length = num_states_;
    output_state_.data.resize(num_states_);

    // Create Ports
    // State Estimate Output Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    std::shared_ptr<Realtime::Port> port = std::make_shared<Realtime::Port>("STATE_HAT", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::DOUBLE, num_states_, rt_period);

    port->SetSignalLabel(Idx::X, "X");
    port->SetSignalLabel(Idx::Y, "Y");
    port->SetSignalLabel(Idx::Z, "Z");

    port->SetSignalLabel(Idx::X_DOT, "X_DOT");
    port->SetSignalLabel(Idx::Y_DOT, "Y_DOT");
    port->SetSignalLabel(Idx::Z_DOT, "Z_DOT");

    port->SetSignalLabel(Idx::PHI, "Roll");
    port->SetSignalLabel(Idx::THETA, "Pitch");
    port->SetSignalLabel(Idx::PSI, "Yaw");

    port->SetSignalLabel(Idx::W_X, "Angular Roll Rate");
    port->SetSignalLabel(Idx::W_Y, "Angular Pitch Rate");
    port->SetSignalLabel(Idx::W_Z, "Angular Yaw Rate");

    port->SetSignalLabel(Idx::GRAVITY, "Gravity");

    output_port_map_[OutputPort::STATE_HAT] = port;    
}

void StateEstimator::Run()
{
    // Estimate State

    // Update State
    output_state_.data[Idx::X] = 1.0; // X Position
    output_state_.data[Idx::Y] = 2.0; // Y Position
    output_state_.data[Idx::Z] = 3.0; // Z Position
    output_state_.data[Idx::X_DOT] = 4.0; // X Velocity
    output_state_.data[Idx::Y_DOT] = 5.0; // Y Velocity
    output_state_.data[Idx::Z_DOT] = 6.0; // Z Velocity
    output_state_.data[Idx::PHI] = 7.0; // Roll Orientation
    output_state_.data[Idx::THETA] = 0.0; // Pitch Orientation
    output_state_.data[Idx::PSI] = 2.0; // Yaw Orientation
    output_state_.data[Idx::W_X] = 0.0; // Roll Rate
    output_state_.data[Idx::W_Y] = 0.0; // Pitch Rate
    output_state_.data[Idx::W_Z] = 0.0; // Yaw Rate
    output_state_.data[Idx::GRAVITY] = kGravity; // Gravity

    //std::cout << "State Estimator Send: " << std::endl;
    
    // Publish State
    bool send_status = GetOutputPort(OutputPort::STATE_HAT)->Send(output_state_);
    
    //std::cout << "[StateEstimator]: Publishing: " << " " << " Send: " << send_status << std::endl;
}

void StateEstimator::Setup()
{
    GetOutputPort(OutputPort::STATE_HAT)->Bind();
    //std::cout << "[StateEstimator]: " << "State Estimator Publisher Running!" << std::endl;
}

} // namespace Estimators
} // namespace Controllers
