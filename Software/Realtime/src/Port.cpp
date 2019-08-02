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

#include <Realtime/Messages/double_vec_t.hpp>

namespace Realtime
{

Port::Port(const std::string &name, Direction direction, DataType data_type, int dimension, int period) : direction_(direction), data_type_(data_type), name_(name), update_period_(period), sequence_num_(0), dimension_(dimension)
{
    queue_size_ = 20;
    transport_type_ = TransportType::INPROC;
    transport_url_ = "inproc"; // TODO: Noblock?

    // If Input Port Create Handlers
    if (direction == Direction::INPUT)
    {
        if (data_type == DataType::DOUBLE)
        {
            PortHandler<double_vec_t> *handler = new PortHandler<double_vec_t>(queue_size_);
            handler_ = (void *)handler;
        }
    }
    else // Setup Outputs
    {
    }

    // Reserve Dimension Names
    if (dimension > 0)
        signal_labels_.reserve(dimension);
}

// TODO: Clear Handler Memory Etc,
Port::~Port()
{
}

void Port::SetSignalLabel(const int signal_idx, const std::string &label)
{
    signal_labels_[signal_idx] = label;
}
// TODO: I do not love this...
bool Port::Map(Port *input, Port *output)
{
    input->transport_url_ = output->transport_url_;
    input->channel_ = output->channel_;
    input->transport_type_ = output->transport_type_;
    input->dimension_ = output->dimension_;
    input->data_type_ = output->data_type_;
    input->signal_labels_ = output->signal_labels_;
}

bool Port::Bind()
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
    else
    {
        std::cout << "[PORT:CONNECT]: ERROR: Invalid Transport Type!" << std::endl;
    }

    return true;
}

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
    else
    {
        std::cout << "[PORT:CONNECT]: ERROR: Invalid Transport Type!" << std::endl;
    }

    // Now Subscribe
    // TODO: Save subs somewhere for unsubscribe
    // TODO: Switch Types
    if (data_type_ == DataType::DOUBLE)
    {
        auto subs = context_->subscribe(channel_, &PortHandler<double_vec_t>::HandleMessage, static_cast<PortHandler<double_vec_t> *>(handler_));
    }
    else
    {
        std::cout << "[PORT:CONNECT]: ERROR: Unsupported Data Type! : " << data_type_ << std::endl;
    }

    return true;
}

///////////////////////
// Port Manager Source
///////////////////////
// Global static pointer used to ensure a single instance of the class.
PortManager *PortManager::manager_instance_ = NULL;

PortManager::PortManager()
{
    // ZCM Context
    inproc_context_ = std::make_shared<zcm::ZCM>("inproc");
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