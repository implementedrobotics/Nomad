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
#include <Communications/Messages/double_vec_t.hpp>
#include <Common/Time.hpp>


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


    // State Estimate Input Port
    input_port_map_[InputPort::IMU] = std::make_shared<Realtime::Port>("IMU", Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, num_states_, rt_period_);

    // State Estimate Output Port
    output_port_map_[OutputPort::STATE_HAT] = port;    
}

void StateEstimator::Run()
{
    // Estimate State
    bool imu_recv = GetInputPort(InputPort::IMU)->Receive(x_hat_in_); // Receive Setpoint
    if (!imu_recv)
    {
        std::cout << "[StateEstimator]: Receive Buffer Empty!" << std::endl;
        return;
    }


    Eigen::VectorXd x_hat_ = Eigen::Map<Eigen::VectorXd>(x_hat_in_.data.data(), num_states_);
    //std::cout << "[StateEstimator]: Received: " << x_hat_in_.sequence_num <<  std::endl;

    // Update State
    output_state_.data[Idx::X] = x_hat_[0]; // X Position
    output_state_.data[Idx::Y] = x_hat_[1]; // Y Position
    output_state_.data[Idx::Z] = x_hat_[2]; // Z Position
    output_state_.data[Idx::X_DOT] = x_hat_[3]; // X Velocity
    output_state_.data[Idx::Y_DOT] = x_hat_[4]; // Y Velocity
    output_state_.data[Idx::Z_DOT] = x_hat_[5]; // Z Velocity
    output_state_.data[Idx::PHI] = x_hat_[6]; // Roll Orientation
    output_state_.data[Idx::THETA] = x_hat_[7]; // Pitch Orientation
    output_state_.data[Idx::PSI] = x_hat_[8]; // Yaw Orientation
    output_state_.data[Idx::W_X] = x_hat_[9]; // Roll Rate
    output_state_.data[Idx::W_Y] = x_hat_[10]; // Pitch Rate
    output_state_.data[Idx::W_Z] = x_hat_[11]; // Yaw Rate
    output_state_.data[Idx::GRAVITY] = x_hat_[12]; // Gravity

    //std::cout << "State Estimator Send: " << std::endl;
    
    // Publish State
    bool send_status = GetOutputPort(OutputPort::STATE_HAT)->Send(output_state_);
    
    //std::cout << "[StateEstimator]: Publishing: " << output_state_.data[Idx::X] << " Send: " << send_status << std::endl;
}

void StateEstimator::Setup()
{

    // Connect Input Ports
    bool connect = GetInputPort(InputPort::IMU)->Connect();            // State Estimate

    GetOutputPort(OutputPort::STATE_HAT)->Bind();
    std::cout << "[StateEstimator]: " << "State Estimator Publisher Running!: " << connect << std::endl;
}

} // namespace Estimators
} // namespace Controllers
