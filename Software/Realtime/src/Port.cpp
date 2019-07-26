/*
 * Port.cpp
 *
 *  Created on: July 25, 2019
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

#include <Realtime/Port.hpp>

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <assert.h>

#include <iostream>
#include <string>
#include <chrono>
#include <map>

namespace Realtime
{

// TODO: I do not love this...
bool Port::Map(Port *input, Port *output)
{
    input->transport_url_ = output->transport_url_;
    input->channel_ = output->channel_;
    input->transport_type_ = output->transport_type_;
}

bool Port::Bind() 
{
    // Reset and Clear Reference
    context_.reset();

    // Setup Contexts
    if(transport_type_ == TransportType::INPROC)
    {
        context_ = PortManager::Instance()->GetInprocContext();
    }
    else if(transport_type_ == TransportType::IPC)
    {
        context_ = std::make_shared<zcm::ZCM>("ipc");
    }
    else if(transport_type_ == TransportType::UDP)
    {
        context_ = std::make_shared<zcm::ZCM>(transport_url_);
    }
    else if(transport_type_ == TransportType::SERIAL)
    {
        context_ = std::make_shared<zcm::ZCM>(transport_url_);
    }
    else
    {
        std::cout << "[PORT:CONNECT]: ERROR: Invalid Transport Type!" << std::endl;
    }

    return true;
}

bool Port::Send(void *buffer, const unsigned int length)
{
    // //zmq::message_t message(length + HEADER_SIZE);

    // // Update Sequence Count
    // packet_.sequence_number = sequence_num_;

    // // Get Timestamp
    // // TODO: "GetUptime" Static function in a time class
    // packet_.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

    // // Copy to packet
    // memcpy((void *)packet_.data, buffer, length);

    // bool status = socket_->send((void*)&packet_, length + HEADER_SIZE, 0);
    // sequence_num_++;

    // // TODO: Zero Copy
    // //memcpy(message.data(), (void *)&packet_, length + HEAflagsDER_SIZE);

    // std::cout << "Send: " << this << " "  << sequence_num_ << std::endl;
    //return status;

    //return Send(message, flags);

    return true;
}

bool Port::Receive(void *buffer, const unsigned int length)
{

    // //bool status = socket_->send((void*)&packet_, length + HEADER_SIZE, flags);
    // int bytes = socket_->recv((void*)&packet_, length + HEADER_SIZE, 0);
    // std::cout << "Received Bytes: " << bytes << std::endl;
    // //zmq::message_t rx_msg;
    // //bool ret_status = Receive(rx_msg, flags); // Receive Buffer
    // if(bytes) 
    // {
    //     // Copy to packet
    //     //memcpy((void *)&packet_, rx_msg.data(), rx_msg.size());
    
    //     // TODO: Check Timestamps and sequence for errors and latency
    //     // TODO: Should be able to just copy pointer as it will stay valid until another send.
    //     // Copy to output
    //     memcpy(buffer, (void *)packet_.data, length);

    //     std::cout << "Receive: " << packet_.sequence_number << std::endl;
    // }

    // return bytes > 0;
    return true;
}


// Port Manager Source

// Global static pointer used to ensure a single instance of the class.
PortManager *PortManager::manager_instance_ = NULL;

PortManager::PortManager()
{
    // ZCM Context
    inproc_context_ = std::make_shared<zcm::ZCM>("inproc");
    //inproc_context_ = new zcm::ZCM("inproc");
}

PortManager *PortManager::Instance()
{
    if (manager_instance_ == NULL)
    {
        manager_instance_ = new PortManager();
    }
    return manager_instance_;
}
} // namespace Realtime