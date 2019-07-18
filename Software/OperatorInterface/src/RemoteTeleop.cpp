/*
 * RemoteTeleop.cpp
 *
 *  Created on: July 17, 2019
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
#include <OperatorInterface/RemoteTeleop.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>

// Third-Party Includes

// Project Includes
#include <Controllers/RealTimeTask.hpp>
#include <Controllers/Messages.hpp>

namespace OperatorInterface
{

namespace Teleop
{
RemoteTeleop::RemoteTeleop(const std::string &name,
                               const long rt_period,
                               unsigned int rt_priority,
                               const int rt_core_id,
                               const unsigned int stack_size) : 
                               Controllers::RealTimeControl::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size),
                               sequence_num_(0)
{
    
    // Create Ports
    zmq::context_t *ctx = Controllers::RealTimeControl::RealTimeTaskManager::Instance()->GetZMQContext();

    // Setpoint OUTPUT Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    Controllers::RealTimeControl::Port *port = new Controllers::RealTimeControl::Port("SETPOINT", ctx, "setpoint", rt_period);
    output_port_map_[OutputPort::SETPOINT] = port;

    // TODO: Multiple Inputs with relative control priorities. i.e. Autonomous Mapping vs. Remote Controller vs. "Safety" Controller
    // Remote INPUT Port
    port = new Controllers::RealTimeControl::Port("REMOTE", ctx, "remote", rt_period);
    output_port_map_[OutputPort::SETPOINT] = port;
   
}

void RemoteTeleop::Run()
{
    // Input (Remote)
    //Messages::Controllers::Locomotion::TrajectorySetpoint setpoint;

    // Output (Reference Setpoint)
    Messages::Controllers::Locomotion::TrajectorySetpoint setpoint;

    // Get Timestamp
    // TODO: "GetUptime" Static function in a time class
    uint64_t time_now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    setpoint.timestamp = time_now;
    setpoint.sequence_number = sequence_num_;
    setpoint.x_dot = 1.0;
    setpoint.y_dot = sequence_num_;
    setpoint.yaw_dot = 1.0;
    setpoint.z_com = 0.5;

    // Publish Setpoint
    GetOutputPort(0)->Send(&setpoint, sizeof(setpoint));
    // std::cout << "[RemoteTeleop]: Publishing: " << setpoint.sequence_number << std::endl;

    sequence_num_++;
}

void RemoteTeleop::Setup()
{
    GetOutputPort(0)->Bind();
    std::cout << "[RemoteTeleop]: " << "Remote Teleop Task Running!" << std::endl;
}

} // namespace Teleop
} // namespace OperatorInterface
