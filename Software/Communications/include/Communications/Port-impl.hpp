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

#include <Communications/Port.hpp>

#include <unistd.h>
#include <assert.h>

#include <memory>
#include <iostream>
#include <chrono>


namespace Communications
{

    template <typename T>
    std::shared_ptr<Port> Port::CreateInput(const std::string &name, int period)
    {
        std::shared_ptr<Communications::Port> port = std::make_shared<Communications::Port>(name, Direction::INPUT, period);
        port->_CreateHandler<T>();
        return port;
    }

    template <typename T>
    void Port::_CreateHandler()
    {
        handler_ = (void *)(new PortHandler<T>(queue_size_));
    }

    template <class T>
    PortHandler<T>::PortHandler(int queue_size) : queue_size_(queue_size), num_unread_(0)
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
        //printf("Received message on channel \"%s\" and %s:\n", chan.c_str(), channel_.c_str());
        //printf("  Message   = %ld\n", msg->sequence_num);

        // TODO: CV Notify...
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
    void PortHandler<T>::HandleMessage(T &rx_msg)
    {
        if (msg_buffer_.size() >= queue_size_)
        {
            // Kill Old Message
            msg_buffer_.pop_front();
        }
        msg_buffer_.push_back(rx_msg);
    }

    template <class T>
    const inline bool PortHandler<T>::Read(T &rx_msg, std::chrono::duration<double> timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if(cond_.wait_for(lock, timeout, [&] {return num_unread_ > 0; }))
        {
            num_unread_ = 0; // Reset Unread count
            rx_msg = msg_buffer_.back();
            msg_buffer_.pop_back();
            //std::cout << "GOT DATA: " << "TRUE" << std::endl;
            return true;
        }
        else
        {
            return false;
        }

        // std::unique_lock<std::mutex> lck(mutex_);
        // if (msg_buffer_.empty())
        //     return false;

        // rx_msg = msg_buffer_.back();
        // msg_buffer_.pop_back();
        // return true;
    }

    // Connect Port
    template <class T>
    bool Port::Connect()
    {
        // Reset and Clear Reference
        context_.reset();

        // Setup Contexts
        if (transport_type_ == TransportType::INPROC)
        {
            context_ = PortManager::Instance()->GetInprocContext();
        }
        else if (transport_type_ == TransportType::IPC)
        {
            context_ = std::make_shared<zcm::ZCM>("ipc");
        }
        else if (transport_type_ == TransportType::UDP)
        {
            context_ = std::make_shared<zcm::ZCM>(transport_url_);
        }
        else if (transport_type_ == TransportType::SERIAL)
        {
            context_ = std::make_shared<zcm::ZCM>(transport_url_);
        }
        else if (transport_type_ == TransportType::NATIVE)
        {
            // Nothing to do here yet
        }
        else
        {
            std::cout << "[PORT:CONNECT]: ERROR: Invalid Transport Type!" << std::endl;
        }

        // Now Subscribe
        // TODO: Save subs somewhere for unsubscribe
        // TODO: Switch Types
        // if (data_type_ == DataType::DOUBLE)
        // {
        //     auto subs = context_->subscribe(channel_, &PortHandler<double_vec_t>::HandleMessage, static_cast<PortHandler<double_vec_t> *>(handler_));
        // }
        // else if (data_type_ == DataType::INT32)
        // {
        //     auto subs = context_->subscribe(channel_, &PortHandler<int32_vec_t>::HandleMessage, static_cast<PortHandler<int32_vec_t> *>(handler_));
        // }
        // else if (data_type_ == DataType::BYTE)
        // {
        //     auto subs = context_->subscribe(channel_, &PortHandler<generic_msg_t>::HandleMessage, static_cast<PortHandler<generic_msg_t> *>(handler_));
        // }
        // else
        // {
        //     std::cout << "[PORT:CONNECT]: ERROR: Unsupported Data Type! : " << data_type_ << std::endl;
        //     return false;
        // }
        //static_cast<PortHandler<T> *>(handler_)->channel_ = channel_;

        if (transport_type_ != TransportType::NATIVE)
        {
            auto subs = context_->subscribe(channel_, &PortHandler<T>::HandleMessage, static_cast<PortHandler<T> *>(handler_));
            if (transport_type_ != TransportType::INPROC)
            {
                context_->start();
            }
        }

        started_ = true;
        return true;
    }

    // Send data on port
    template <class T>
    bool Port::Send(T &tx_msg)
    {
        //std::cout << "Sending Channel: " << channel_ << std::endl;
        if (channel_.empty())
        {
            std::cout << "[PORT]: ERROR: Output channel is NOT defined!" << std::endl;
            return false;
        }
        // Append Sequence Number and Timestamp
        // Get Timestamp
        // TODO: "GetUptime" Static function in a time class
        //uint64_t time_now = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        uint64_t time_now = Systems::Time::GetTimeStamp();

        // Move this back to the PORT portion
        tx_msg.timestamp = time_now;
        tx_msg.sequence_num = sequence_num_++;

        // Publish
        if(transport_type_ != TransportType::NATIVE)
        {
            int rc = context_->publish(channel_, &tx_msg);
            
            // True if OK
            return rc == ZCM_EOK;
        }
        else
        {
            //std::cout << "THREAD SENDING!" << std::endl;
            for(auto port : listeners_)
            {
                PortHandler<T> *handler = static_cast<PortHandler<T> *>(port->handler_);
                handler->mutex_.lock();
                handler->HandleMessage(tx_msg);
                handler->num_unread_++;
                handler->mutex_.unlock();
                handler->cond_.notify_one();
                //std::cout << "LISTEN" << std::endl;
            }
        }
        


    }

    // Receive data on port
    template <class T>
    bool Port::Receive(T &rx_msg, std::chrono::duration<double> timeout)
    {
        // Check Started, If not Connect it
        if (!started_)
        {
            Connect<T>();
        }
        return static_cast<PortHandler<T> *>(handler_)->Read(rx_msg, timeout);
    }

} // namespace Communications