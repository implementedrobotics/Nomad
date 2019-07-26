// /*
//  * GaitScheduler.cpp
//  *
//  *  Created on: July 14, 2019
//  *      Author: Quincy Jones
//  *
//  * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
//  * Permission is hereby granted, free of charge, to any person obtaining a
//  * copy of this software and associated documentation files (the "Software"),
//  * to deal in the Software without restriction, including without limitation
//  * the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  * and/or sell copies of the Software, and to permit persons to whom the Software
//  * is furnished to do so, subject to the following conditions:
//  * The above copyright notice and this permission notice shall be included in all
//  * copies or substantial portions of the Software.
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//  * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//  */

// // Primary Include
// #include <Controllers/GaitScheduler.hpp>

// // C System Includes

// // C++ System Includes
// #include <iostream>
// #include <string>
// #include <sstream>

// // Third-Party Includes
// // Project Includes
// #include <Realtime/RealTimeTask.hpp>

// namespace Controllers
// {

// namespace Locomotion
// {
// //using namespace RealTimeControl;

// GaitScheduler::GaitScheduler(const std::string &name,
//                      const long rt_period,
//                      unsigned int rt_priority,
//                      const int rt_core_id,
//                      const unsigned int stack_size) : Realtime::RealTimeTaskNode(name, rt_period, rt_priority, rt_core_id, stack_size),
//                                                       gait_schedule_sequence_num_(0)
// {
//     // Create Ports
//     zmq::context_t *ctx = Realtime::RealTimeTaskManager::Instance()->GetZMQContext();

//     // Gait Scheduler Output Ports
//     // TODO: Independent port speeds.  For now all ports will be same speed as task node
//     Realtime::Port *port = new Realtime::Port("CONTACT_STATE", ctx, "gait/contact", rt_period);
//     output_port_map_[OutputPort::CONTACT_STATE] = port;
// }

// void GaitScheduler::Run()
// {
//     // Publish State

//     // TODO: Struct Pubish.
//     // Send Gait Contact Phase
//     // Send Gait Swing/Contact Phase Times
//     // TODO: Zero Copy Publish
//     std::stringstream s;
//     s << "Contact [0,1,0,1] " << gait_schedule_sequence_num_;
//     auto msg = s.str();
//     zmq::message_t message(msg.length());
//     memcpy(message.data(), msg.c_str(), msg.length());

//     GetOutputPort(0)->Send(message);

//     std::cout << "[GaitScheduler]: Publishing: " << msg << std::endl;

//     gait_schedule_sequence_num_++;
// }

// void GaitScheduler::Setup()
// {
//     GetOutputPort(0)->Bind();
//     std::cout << "[GaitScheduler]: " << "Gait Scheduler Publisher Running!" << std::endl;
// }

// } // namespace Locomotion
// } // namespace Controllers
