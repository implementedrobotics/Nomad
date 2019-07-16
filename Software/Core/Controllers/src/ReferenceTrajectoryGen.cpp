/*
 * ReferenceTrajectoryGenerator.cpp
 *
 *  Created on: July 16, 2019
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
#include <Controllers/ReferenceTrajectoryGen.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>

// Third-Party Includes
#include <Eigen/Dense>

// Project Includes
#include <Controllers/RealTimeTask.hpp>
#include <Controllers/StateEstimator.hpp>

namespace Controllers
{

namespace Locomotion
{
//using namespace RealTimeControl;

ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(const std::string &name, const unsigned int N, const double T) : 
                               RealTimeControl::RealTimeTaskNode(name, 20000, RealTimeControl::Priority::MEDIUM, -1, PTHREAD_STACK_MIN),
                               reference_sequence_num_(0),
                               num_states_(13),
                               T_(T),
                               N_(N)
{
    
    // Sample Time
    T_s_ = T_ / N_;

    // Reference State Trajectory
    X_ref_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states_, N_);

    // Create Ports
    zmq::context_t *ctx = RealTimeControl::RealTimeTaskManager::Instance()->GetZMQContext();

    // Reference Output Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    RealTimeControl::Port *port = new RealTimeControl::Port("REFERENCE", ctx, "reference", rt_period_);
    output_port_map_[OutputPort::REFERENCE] = port;

    port = new RealTimeControl::Port("STATE_HAT", ctx, "state", rt_period_);
    input_port_map_[InputPort::STATE_HAT] = port;

    port = new RealTimeControl::Port("SETPOINT", ctx, "setpoint", rt_period_);
    input_port_map_[InputPort::SETPOINT] = port;
    
}

void ReferenceTrajectoryGenerator::Run()
{

    Controllers::Estimators::CoMState x_hat;

    TrajectorySetpoint setpoint;
    setpoint.x_dot = 1.0;
    setpoint.y_dot = 2.0;
    setpoint.yaw_dot = 0.0;
    setpoint.z_com = 0.5;

    std::cout << sizeof(setpoint) << std::endl;

    // Get Inputs
    GetInputPort(0)->Receive((void *)&x_hat, sizeof(x_hat)); // Receive State Estimate
    std::cout << "X: " << x_hat.x[3] << std::endl;
    std::cout << "[ReferenceTrajectoryGenerator]: Received State: " << x_hat.x[3] << " : " << reference_sequence_num_ << std::endl;

    //GetInputPort(1)->Receive((void *)&setpoint, sizeof(setpoint)); // Receive Setpoint

    // Compute Trajectory

    // Publish Trajectory
    // TODO: Zero Copy Publish
    std::stringstream s;
    s << "Hello " << reference_sequence_num_;
    auto msg = s.str();
    zmq::message_t message(msg.length());
    memcpy(message.data(), msg.c_str(), msg.length());

    GetOutputPort(0)->Send(message);
    std::cout << "[ReferenceTrajectoryGenerator]: Publishing: " << msg << std::endl;

    reference_sequence_num_++;
}

void ReferenceTrajectoryGenerator::Setup()
{

    // State Estimate
    GetInputPort(0)->Connect();

    // Setpoint Input
    //GetInputPort(1)->Connect();

    // Reference Output
    GetOutputPort(0)->Bind();
    std::cout << "[ReferenceTrajectoryGenerator]: " << "Reference Trajectory Publisher Running!" << std::endl;
}

} // namespace Locomotion
} // namespace Controllers
