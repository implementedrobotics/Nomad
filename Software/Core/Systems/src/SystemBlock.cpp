/*
 * SystemBlock.cpp
 *
 *  Created on: August 15, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
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

// Primary Include
#include <Systems/SystemBlock.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>

// Third-Party Includes

// Project Includes
#include <Systems/BlockDiagram.hpp>

namespace Core::Systems
{

    SystemBlock::SystemBlock(const std::string &name, const double T_s) : name_(name), T_s_(T_s), T_(0), T_prev_(0), parent_(nullptr)
    {
        for (auto &port : output_port_map_)
        {
            port = nullptr;
            //port->Bind();
        }
    }
    void SystemBlock::Run(double d_t)
    {

        // Update current time
        T_ += d_t;

        // Check Time to Run
        if((T_s_ > 0) && ((T_prev_ + T_s_) > T_))
        {   
            return;     
        }

        // Update function for stateful outputs
        UpdateStateOutputs();

        // Update function for stateless outputs
        UpdateStatelessOutputs();

        // Update fucntion for next state from inputs
        UpdateState();

        // Update last run time
        T_prev_ = T_;
    }

    void SystemBlock::Setup()
    {
        // Bind any active output ports
        for( auto port : output_port_map_)
        {
            if(port != nullptr)
            {
                port->Bind();
            }
        }
    }

    // Get Output Port
    std::shared_ptr<Communications::Port> SystemBlock::GetOutputPort(const int port_id) const
    {
        assert(port_id >= 0 && port_id < MAX_PORTS);
        return output_port_map_[port_id];
    }

    // Get Input Port
    std::shared_ptr<Communications::Port> SystemBlock::GetInputPort(const int port_id) const
    {
        assert(port_id >= 0 && port_id < MAX_PORTS);
        return input_port_map_[port_id];
    }

    void SystemBlock::SetPortOutput(const int port_id, const Communications::Port::TransportType transport, const std::string &transport_url, const std::string &channel)
    {
        assert(port_id >= 0 && port_id < MAX_PORTS);
        output_port_map_[port_id]->SetTransport(transport, transport_url, channel);
    }

} // namespace Controllers::Systems
