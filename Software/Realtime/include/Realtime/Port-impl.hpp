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

#include <unistd.h>
#include <assert.h>

#include <iostream>
#include <chrono>

namespace Realtime
{

template <class T>
PortImpl<T>::PortImpl(const std::string &name, int period)
{
    name_ = name;
    update_period_ = period;
    queue_size_ = 20;
    transport_type_ = TransportType::INPROC;
    transport_url_ = "inproc";  // TODO: Noblock?
}

template <class T>
PortImpl<T>::~PortImpl()
{
    //TODO: Clear any buffers, etc.
}

template <class T>
void PortImpl<T>::HandleMessage(const zcm::ReceiveBuffer *rbuf,
                                const std::string &chan,
                                const T *msg)
{
    printf("Received message on channel \"%s\":\n", chan.c_str());
    printf("  Message   = %ld\n", msg->sequence_num);

    std::unique_lock<std::mutex> lck (mutex_);
    if(msg_buffer_.size() >= queue_size_)
    {
        // Kill Old Message
        msg_buffer_.pop_front();
    }
    msg_buffer_.push_back(*msg);

    //std::cout << msg_buffer_.size() << std::endl;
}

// Send data on port
template <class T>
bool Port::Send(T &tx_msg)
{
    //std::cout << "Channel: " << channel_ << std::endl;

    // Publish
    int rc = context_->publish(channel_, &tx_msg);

    // True if OK
    return rc == ZCM_EOK;
}

// Receive data on port
template <class T>
bool Port::Receive(T &rx_msg)
{
    
    //std::cout << static_cast<PortImpl<T> *>(this)->GetMessageQueue().size() << std::endl;
    if(static_cast<PortImpl<T> *>(this)->GetMessageQueue().size() == 0)
        return false;

    std::unique_lock<std::mutex> lck (mutex_);
    rx_msg = static_cast<PortImpl<T> *>(this)->GetMessageQueue().back();
    return true;
}

// Connect Port
template <class T>
bool Port::Connect()
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

    // Now Subscribe
    // TODO: Save subs somewhere for unsubscribe
    auto subs = context_->subscribe(channel_, &PortImpl<T>::HandleMessage, static_cast<PortImpl<T> *>(this));
    return true;
}

} // namespace Realtime