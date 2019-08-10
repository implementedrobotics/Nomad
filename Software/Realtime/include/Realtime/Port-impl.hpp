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
PortHandler<T>::PortHandler(int queue_size) : queue_size_(queue_size)
{
}

template <class T>
PortHandler<T>::~PortHandler()
{
    //TODO: Clear any buffers, etc.
}

template <class T>
void PortHandler<T>::HandleMessage(const zcm::ReceiveBuffer *rbuf,
                                   const std::string &chan,
                                   const T *msg)
{
    //printf("Received message on channel \"%s\":\n", chan.c_str());
    //printf("  Message   = %ld\n", msg->sequence_num);

    std::unique_lock<std::mutex> lck(mutex_);
    if (msg_buffer_.size() >= queue_size_)
    {
        // Kill Old Message
        msg_buffer_.pop_front();
    }
    msg_buffer_.push_back(*msg);

    //std::cout << msg_buffer_.size() << std::endl;
}
template <class T>
const inline bool PortHandler<T>::Read(T &rx_msg)
{
    std::unique_lock<std::mutex> lck(mutex_);
    if (msg_buffer_.empty())
        return false;
        
    rx_msg = msg_buffer_.back();
    msg_buffer_.pop_back();
    return true;
}

// Send data on port
template <class T>
bool Port::Send(T &tx_msg)
{
    //std::cout << "Sending Channel: " << channel_ << std::endl;

    // Append Sequence Number and Timestamp
    // Get Timestamp
    // TODO: "GetUptime" Static function in a time class
    //uint64_t time_now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    uint64_t time_now = Systems::Time::GetTime();

    // Move this back to the PORT portion
    tx_msg.timestamp = time_now;
    tx_msg.sequence_num = sequence_num_++;

    // Publish
    int rc = context_->publish(channel_, &tx_msg);

    // True if OK
    return rc == ZCM_EOK;
}

// Receive data on port
template <class T>
bool Port::Receive(T &rx_msg)
{
    return static_cast<PortHandler<T> *>(handler_)->Read(rx_msg);
}

} // namespace Realtime