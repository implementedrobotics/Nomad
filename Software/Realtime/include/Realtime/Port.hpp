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
#include <limits.h>
#include <pthread.h>

// C++ Includes
#include <iostream>
#include <string>
#include <map>

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
    void SetTransport(const std::string &transport) {transport_ = transport;}

    // Map Ports
    static bool Map(Port *input, Port *output);

    // Connect Port
    bool Connect();

    // Bind Port
    bool Bind();

    // Send data on port raw
    bool Send(void *buffer, const unsigned int length);

    // Receive data on port raw
    bool Receive(void *buffer, const unsigned int length);  


protected:
    // Port Name
    std::string name_;

    // Update Period
    unsigned int update_period_;

    // Using ZeroCM for thread sync and message passing
    
    // TODO: Need a enum for types, i.e. TCP, UDP, IPC, INPROC

    // Transport
    std::string transport_;

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

    // Send message type data on port
    bool Send(T &msg);

    // Receive message type data on port
    bool Receive(T &msg);

private:

    // Message
    T message_;

};
} // namespace Realtime

#include <Realtime/Port-impl.hpp>
#endif // NOMAD_REALTIME_PORT_H_
