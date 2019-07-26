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

// Third Party Includes
#include <zcm/zcm-cpp.hpp>

namespace Realtime
{
class Port
{

public:
    // Base class for RealTimeTaskNode Port
    // name = Port Name
    // ctx = ZCM Context
    // transport = ZCM Message Transport Location String
    // period = Update period (Does not matter for Input Ports)
    //Port(const std::string &name, zcm::ZCM *zcm_context, const std::string &transport, int period);
    //~Port();

    // Transport
    void SetTransport(const std::string &transport) { transport_ = transport; }

    // Map Ports
    static bool Map(Port *input, Port *output);

    // Connect Port
    template <class T>
    bool Connect();

    // Bind Port
    bool Bind();

    // Send data on port raw
    bool Send(void *buffer, const unsigned int length);

    // Receive data on port raw
    bool Receive(void *buffer, const unsigned int length);

    // Send message type data on port
    template <class T>
    bool Send(T &msg);

    // Receive message type data on port
    template <class T>
    bool Receive(T &msg);

protected:
    // Port Name
    std::string name_;

    // Update Period
    unsigned int update_period_;

    // TODO: Need a enum for types, i.e. TCP, UDP, IPC, INPROC

    // Transport(Channel)
    std::string transport_;

    // Using ZeroCM for thread sync and message passing
    // Context
    zcm::ZCM *context_;
};

template <class T>
class PortImpl : public Port
{

public:
    // Base class for RealTimeTaskNode Port
    // name = Port Name
    // ctx = ZCM Context
    // transport = ZCM Message Transport Location String
    // period = Update period (Does not matter for Input Ports)
    PortImpl(const std::string &name, zcm::ZCM *zcm_context, const std::string &transport, int period);
    ~PortImpl();

    void HandleMessage(const zcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const T *msg);

    std::deque<T> msg_buffer_;

private:
};
} // namespace Realtime

#include <Realtime/Port-impl.hpp>
#endif // NOMAD_REALTIME_PORT_H_
