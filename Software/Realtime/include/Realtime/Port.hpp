/*
 * Port.hpp
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

#ifndef NOMAD_REALTIME_PORT_H_
#define NOMAD_REALTIME_PORT_H_

// C Includes

// C++ Includes
#include <deque>
#include <memory>
#include <mutex>

// Third Party Includes
#include <zcm/zcm-cpp.hpp>

namespace Realtime
{
class Port
{

public:

    // Transport Type Enum
    enum TransportType {
        INPROC=0,
        IPC,
        UDP,
        SERIAL
    };

    // Transport
    // For INPROC/IPC transport URL should depend on block/noblock?  Not technically necessary to set.
    void SetTransport(const TransportType transport, const std::string &transport_url, const std::string &channel) { 
        transport_type_ = transport;
        transport_url_ = transport_url;
        channel_ = channel; }

    // Map Portsconst
    static bool Map(Port *input, Port *output);

    // Connect Port
    template <class T>
    bool Connect();

    // Bind Port
    bool Bind();

    // Send data on port raw
    bool Send(void *buffer, const unsigned int length);
    
    // Send message type data on port
    template <class T>
    bool Send(T &msg);
    
    // Receive data on port raw
    bool Receive(void *buffer, const unsigned int length);

    // Receive message type data on port
    template <class T>
    bool Receive(T &msg);

protected:

    // Port Name
    std::string name_;

    // Update Period
    unsigned int update_period_;

    // Size of the queue bufferr
    int queue_size_;

    // Transport Type
    TransportType transport_type_;

    // Transport URL
    std::string transport_url_;

    // Channel
    std::string channel_;

    // Using ZeroCM for thread sync and message passing
    // Context
    std::shared_ptr<zcm::ZCM> context_;

    // Thread mutex
    std::mutex mutex_;

};

// TODO: Better name for this class?
template <class T>
class PortImpl : public Port
{
friend class Port;
public:
    // Base class for RealTimeTaskNode Port
    // name = Port Name
    // ctx = ZCM Context
    // transport = ZCM Message Transport Location String
    // period = Update period (Does not matter for Input Ports)
    PortImpl(const std::string &name, int period);
    ~PortImpl();

    // Message Handling Callback
    void HandleMessage(const zcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const T *msg);

protected:

    // Get Message Buffer
    inline std::deque<T>& GetMessageQueue() { return msg_buffer_; }

    // Message Buffer
    std::deque<T> msg_buffer_;

private:
};


class PortManager
{

public:
    // Base Class Real Time Task Manager
    PortManager();

    // STATIC Singleton Instance
    static PortManager *Instance();

    // Singleton ZCM Context for INPROC messaging
   std::shared_ptr<zcm::ZCM> GetInprocContext() const { return inproc_context_; }

protected:
    // Using ZMQ for thread sync and message passing
    // ZMQ Context
    // Thread inproc messaging must share a single context.  We put it in the singleton here to keep it unique;
    //zcm::ZCM *inproc_context_;

    std::shared_ptr<zcm::ZCM> inproc_context_;

private:
    // Singleton Instance
    static PortManager *manager_instance_;
};


} // namespace Realtime

#include <Realtime/Port-impl.hpp>
#endif // NOMAD_REALTIME_PORT_H_
