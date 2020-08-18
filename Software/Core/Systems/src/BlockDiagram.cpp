/*
 * BlockDiagram.cpp
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
#include <Systems/BlockDiagram.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>

// Third-Party Includes

// Project Includes
#include <Common/Time.hpp>
#include <Realtime/RealTimeTask.hpp>

namespace Core::Systems
{

    //using namespace RealTimeControl;
    BlockDiagram::BlockDiagram(const std::string &name, const double T_s) : Realtime::RealTimeTaskNode(name, T_s, Realtime::Priority::MEDIUM, -1, PTHREAD_STACK_MIN),
                                                                            T_s_(T_s)
    {
    }
    void BlockDiagram::Run()
    {
       // std::cout << "Run Diagram: " << task_name_ << std::endl;
       // ::Systems::Time t;
        for (auto system : systems_)
        {
            system->Run(T_s_);
        }
    }
    void BlockDiagram::Setup()
    {
        // Bind any active output ports
        for (auto &port : output_port_map_)
        {
            if (port != nullptr)
            {
                port->Bind();
            }
        }
    }

    // Add system to block
    void BlockDiagram::AddSystem(std::shared_ptr<SystemBlock> system)
    {
        // Call Setup
        system->Setup();
        system->parent_ = this;

        // Add to Vector List Registry
        systems_.push_back(std::move(system));
    }

    // Get Output Port
    std::shared_ptr<Communications::Port> BlockDiagram::GetOutputPort(const int port_id) const
    {
        assert(port_id >= 0 && port_id < MAX_PORTS);
        return output_port_map_[port_id];
    }

    // Get Input Port
    std::shared_ptr<Communications::Port> BlockDiagram::GetInputPort(const int port_id) const
    {
        assert(port_id >= 0 && port_id < MAX_PORTS);
        return input_port_map_[port_id];
    }

    void BlockDiagram::SetPortOutput(const int port_id, const Communications::Port::TransportType transport, const std::string &transport_url, const std::string &channel)
    {
        assert(port_id >= 0 && port_id < MAX_PORTS);
        output_port_map_[port_id]->SetTransport(transport, transport_url, channel);
    }

    void BlockDiagram::Connect(std::shared_ptr<Communications::Port> output, std::shared_ptr<Communications::Port> input)
    {
        Communications::Port::Map(input, output);
    }

} // namespace Controllers::Systems
