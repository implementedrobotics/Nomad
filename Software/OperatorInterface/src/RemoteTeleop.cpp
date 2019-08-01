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
#include <Realtime/RealTimeTask.hpp>


namespace OperatorInterface
{

namespace Teleop
{
RemoteTeleop::RemoteTeleop(const std::string &name,
                               const long rt_period,
                               unsigned int rt_priority,
                               const int rt_core_id,
                               const unsigned int stack_size) : 
                               Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size)
{
    
    // Create Messages
    output_setpoint_.length = 4;
    output_setpoint_.data.resize(output_setpoint_.length);

    // Create Ports
    // Setpoint OUTPUT Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    Realtime::Port *port = new Realtime::Port ("SETPOINT", Realtime::Port::Direction::OUTPUT, Realtime::Port::DataType::DOUBLE, rt_period);
    output_port_map_[OutputPort::SETPOINT] = port;    


    // TODO: Multiple Inputs with relative control priorities. i.e. Autonomous Mapping vs. Remote Controller vs. "Safety" Controller
    // Remote INPUT Port
    //port = new Realtime::Port("REMOTE", ctx, "remote", rt_period);
    //input_port_map_[OutputPort::SETPOINT] = port;
   
}

void RemoteTeleop::Run()
{
    // Input (Remote)
    //Messages::Controllers::Locomotion::TrajectorySetpoint setpoint;

    output_setpoint_.data[0] = 1.0;  // x_dot
    output_setpoint_.data[1] = 0.0;    // y_dot
    output_setpoint_.data[2] = 0.0;  // yaw_dot
    output_setpoint_.data[3] = 0.5;    // z_comt

    // Publish Setpoint
    bool send_status = GetOutputPort(0)->Send(output_setpoint_);
    std::cout << "[RemoteTeleop]: Publishing: " << " status: " << send_status << std::endl;

}

void RemoteTeleop::Setup()
{
    GetOutputPort(0)->Bind();
    //std::cout << "[RemoteTeleop]: " << "Remote Teleop Task Running!" << std::endl;
}

} // namespace Teleop
} // namespace OperatorInterface
