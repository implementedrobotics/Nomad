/*
 * ConvexMPC.cpp
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
#include <Controllers/ConvexMPC.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>

// Third-Party Includes
// Project Includes
#include <Controllers/RealTimeTask.hpp>

namespace Controllers
{

namespace Locomotion
{
//using namespace RealTimeControl;

ConvexMPC::ConvexMPC(const std::string &name,
                     const long rt_period,
                     unsigned int rt_priority,
                     const int rt_core_id,
                     const unsigned int stack_size) : RealTimeControl::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size),
                                                      control_sequence_num_(0)
{
    // Create Ports
    zmq::context_t *ctx = RealTimeControl::RealTimeTaskManager::Instance()->GetZMQContext();

    // State Estimate Input Port
    // TODO: Independent port speeds.  For now all ports will be same speed as task node
    RealTimeControl::Port *port = new RealTimeControl::Port("STATE_HAT", ctx, "state", rt_period);
    input_port_map_[InputPort::STATE_HAT] = port;

    port = new RealTimeControl::Port("STATE_HAT", ctx, "state", rt_period);
    output_port_map_[OutputPort::FORCES] = port;

}

void ConvexMPC::Run()
{
    control_sequence_num_++;
    zmq::message_t rx_msg;
    GetInputPort(0)->Receive(rx_msg);
    //socket_->recv(&rx_msg);
    
    std::string rx_str;
    rx_str.assign(static_cast<char *>(rx_msg.data()), rx_msg.size());
    std::cout << "[ConvexMPC]: Received: " << rx_str << " : " << control_sequence_num_ << std::endl;
}

void ConvexMPC::Setup()
{
    GetInputPort(0)->Connect();
    std::cout << "[ConvexMPC]: "
              << "ConvexMPC State Subscriber Running!" << std::endl;
}

} // namespace Locomotion
} // namespace Controllers
