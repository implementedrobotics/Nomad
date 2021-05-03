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

using namespace std::chrono_literals;
namespace Communications
{

    template <typename PortType>
    Port<PortType>::Port(const std::string &name, Direction direction, DataType data_type, int dimension, int period) : PortInterface(name, direction, data_type, dimension, period)
    {
        // queue_size_ = 1;
        // transport_type_ = TransportType::NATIVE;
        // transport_url_ = "native"; // TODO: Noblock?

        // // If Input Port Create Handlers
        // if (direction == Direction::INPUT)
        // {
        //     if (data_type == DataType::DOUBLE)
        //     {
        //         PortHandler<double_vec_t> *handler = new PortHandler<double_vec_t>(queue_size_);
        //         handler_ = (void *)handler;
        //     }
        //     else if (data_type == DataType::INT32)
        //     {
        //         PortHandler<int32_vec_t> *handler = new PortHandler<int32_vec_t>(queue_size_);
        //         handler_ = (void *)handler;
        //     }
        //     else if (data_type == DataType::BYTE)
        //     {
        //         PortHandler<generic_msg_t> *handler = new PortHandler<generic_msg_t>(queue_size_);
        //         handler_ = (void *)handler;
        //     }
        // }
        // else // Setup Outputs
        // {
        // }
    }

    template <typename PortType>
    Port<PortType>::Port(const std::string &name, Direction direction, int period) : PortInterface(name, direction, period)
    {
        // queue_size_ = 1;
        // transport_type_ = TransportType::NATIVE;
        // transport_url_ = "native"; // TODO: Noblock?

      //  if(direction == Direction::OUTPUT)
        //    listeners_.reserve(MAX_LISTENERS); // Reserve Space for Output Listeners
    }


    // TODO: Clear Handler Memory Etc,
    template <typename PortType>
    Port<PortType>::~Port()
    {
    }

    template <typename PortType>
    std::shared_ptr<PortInterface> Port<PortType>::CreateInput(const std::string &name, int period)
    {
        std::shared_ptr<Communications::Port<PortType>> port = std::make_shared<Communications::Port<PortType>>(name, Direction::INPUT, period);
        port->_CreateHandler();
        return port;
    }

    template <typename PortType>
    std::shared_ptr<PortInterface> Port<PortType>::CreateOutput(const std::string &name, int period)
    {
        std::shared_ptr<Communications::Port<PortType>> port = std::make_shared<Communications::Port<PortType>>(name, Direction::OUTPUT, period);
        return port;
    }


    // Send data on port
    template <typename PortType>
    bool PortInterface::Send(PortType &tx_msg)
    {
        //std::cout << "Sending Channel: " << channel_ << std::endl;
        //::Systems::Time t;
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
            for (auto port : listeners_)
            {
                PortHandler<PortType> *handler = static_cast<PortHandler<PortType> *>(port->handler_);
                {
                    handler->mutex_.lock();
                    handler->HandleMessage(tx_msg);
                    handler->num_unread_++;
                    handler->mutex_.unlock();
                    handler->cond_.notify_all();
                }
            }
            return true;
        }
    }

    // Receive data on port
    template <class PortType>
    bool PortInterface::Receive(PortType &rx_msg, std::chrono::microseconds timeout)
    {
        // Check Started, If not Connect it
        if (!started_)
        {
            Connect();
        }
        if(transport_type_ != NATIVE)
        {
            //context_->flush();
        }
        auto start_time = std::chrono::high_resolution_clock::now(); 

        PortHandler<PortType> *handler = static_cast<PortHandler<PortType> *>(handler_);
        bool read_status = false;
        while(!(read_status = handler->Read(rx_msg, timeout)))
        {
            auto time_now = std::chrono::high_resolution_clock::now(); 
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_now - start_time); 
            if(duration >= timeout)
                return false;

            // TODO: Verify how to make a less busy wait here...
            usleep(25);
        }
        return read_status;
        //return static_cast<PortHandler<T> *>(handler_)->Read(rx_msg, timeout);
    }


    // TODO: I do not love this...
    // template <typename PortType>
    // bool Port<PortType>::Map(std::shared_ptr<Port> input, std::shared_ptr<Port> output)
    // {
    //     input->transport_url_ = output->transport_url_;
    //     input->channel_ = output->channel_;
    //     input->transport_type_ = output->transport_type_;
    //     input->dimension_ = output->dimension_;
    //     input->data_type_ = output->data_type_;
    //     input->signal_labels_ = output->signal_labels_;

    //     // Add to listeners
    //     output->listeners_.push_back(input);
    //     return true;

    //    // std::cout << "Map: " << input->transport_url_ << " " << input->channel_ << std::endl;
    // }

    template <typename PortType>
    bool Port<PortType>::Bind()
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
            std::cout << "Binding Output Native." << std::endl;
        }
        else
        {
            std::cout << "[PORT:BIND]: ERROR: Invalid Transport Type!" << std::endl;
            return false;
        }
        return true;
    }




    template <typename PortType>
    void Port<PortType>::_CreateHandler()
    {
        handler_ = (void *)(new PortHandler<PortType>(queue_size_));
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
        // TODO: Need explicit buffer path as it is not as optimal and honestly not as necessary
        // TODO: CV Notify...
        std::unique_lock<std::mutex> lck(mutex_);
        // if (msg_buffer_.size() >= queue_size_)
        // {
        //     // Kill Old Message
        //     msg_buffer_.pop_front();
        // }
        // msg_buffer_.push_back(*msg);

        data_ = *msg;

        //Increase Num Unread
        num_unread_++;

        //std::cout << msg_buffer_.size() << std::endl;
    }
    template <class T>
    void PortHandler<T>::HandleMessage(T &rx_msg)
    {     
        // if (msg_buffer_.size() >= queue_size_)
        // {
        //     // Kill Old Message
        //     msg_buffer_.pop_front();
        // }
        
        // msg_buffer_.push_back(rx_msg);
        data_ = rx_msg;
    }

    template <class T>
    const inline bool PortHandler<T>::Read(T &rx_msg, std::chrono::duration<double> timeout)
    {
       // std::unique_lock<std::mutex> lock(mutex_);

        mutex_.lock();
        if(num_unread_ > 0)
        {
            num_unread_ = 0; // Reset Unread count
            rx_msg = data_;//msg_buffer_.back();
            //msg_buffer_.pop_back();
            mutex_.unlock();
            return true;
        }
        else
        {
            mutex_.unlock();
            return false;
        }
        
        //lock.unlock();
        // if(cond_.wait_for(lock, timeout, [&] {return num_unread_ > 0; }))
        // {
        //     num_unread_ = 0; // Reset Unread count
        //     rx_msg = msg_buffer_.back();
        //     msg_buffer_.pop_back();
        //     return true;
        // }
        // else
        // {
        //     return false;
        // }

        // std::unique_lock<std::mutex> lck(mutex_);
        // if (msg_buffer_.empty())
        //     return false;

        // rx_msg = msg_buffer_.back();
        // msg_buffer_.pop_back();
        // return true;
    }

    // Connect Port
    template <typename PortType>
    bool Port<PortType>::Connect()
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

        //static_cast<PortHandler<T> *>(handler_)->channel_ = channel_;

        if (transport_type_ != TransportType::NATIVE)
        {
            auto subs = context_->subscribe(channel_, &PortHandler<PortType>::HandleMessage, static_cast<PortHandler<PortType> *>(handler_));
            if (transport_type_ != TransportType::INPROC)
            {
                context_->start();
            }
        }

        started_ = true;
        return true;
    }

} // namespace Communications