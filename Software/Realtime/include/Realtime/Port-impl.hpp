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
PortImpl<T>::PortImpl(const std::string &name, zcm::ZCM *ctx, const std::string &transport, int period)
{
    name_ = name;
    context_ = ctx;
    transport_ = transport;
    update_period_ = period;
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
    printf("  Message   = %s\n", msg->str.c_str());

    // TODO: Check against a maximum queue size, blah blah
    // TODO: Mutex For Read
    msg_buffer_.push_back(*msg);

    msg_buffer_.front().str;
}

// Send data on port
template <class T>
bool Port::Send(T &tx_msg)
{
    std::cout << "Channel: " << transport_ << std::endl;
    // Publish
    int rc = context_->publish(transport_, &tx_msg);

    // sequence_num_++;

    //return status;
    return rc == ZCM_EOK;
}

// Receive data on port
template <class T>
bool Port::Receive(T &rx_msg)
{
    //return socket_->recv(&rx_msg, flags);
    ((PortImpl<T> *)this)->msg_buffer_.back();
    return true;
}

// Connect Port
template <class T>
bool Port::Connect()
{

    // TODO: More correct CAST here?
    auto subs = context_->subscribe(".*", &PortImpl<T>::HandleMessage, (PortImpl<T> *)this);
    return true;
}

} // namespace Realtime