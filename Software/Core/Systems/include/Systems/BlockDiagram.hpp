/*
 * BlockDiagram.hpp
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

#ifndef NOMAD_CORE_CONTROLLERS_BLOCKDIAGRAM_H_
#define NOMAD_CORE_CONTROLLERS_BLOCKDIAGRAM_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>
#include <memory>

// Project Include Files
#include <Realtime/RealTimeTask.hpp>
#include <Systems/SystemBlock.hpp>

namespace Core::Systems
{
    class BlockDiagram : public Realtime::RealTimeTaskNode
    {

    public:
        // Block Diagram Class For Systems Task Node
        // name = Task Name
        // T_s = Sample Time (-1 for inherit)
        BlockDiagram(const std::string &name, const double T_s = -1);

        // Add system to diagram
        void AddSystem(std::shared_ptr<SystemBlock> system);

        // Get Output Port
        std::shared_ptr<Communications::Port> GetOutputPort(const int port_id) const;

        // Get Input Port
        std::shared_ptr<Communications::Port> GetInputPort(const int port_id) const;

        // Set Transport Configuration for Port
        void SetPortOutput(const int port_id, const Communications::Port::TransportType transport, const std::string &transport_url, const std::string &channel);

        // Connect Function
        void Connect(std::shared_ptr<Communications::Port> output, std::shared_ptr<Communications::Port> input);

    protected:

        // Overriden Run Function
        virtual void Run();

        // Pre-Run Setup Routine.  Setup any one time initialization here.
        virtual void Setup();

        // Sampling Time (s)
        double T_s_;

        // Input Port Map
        std::shared_ptr<Communications::Port> input_port_map_[MAX_PORTS];

        // Output Port Map
        std::shared_ptr<Communications::Port> output_port_map_[MAX_PORTS];

        // Systems that belong to this block diagram
        std::vector<std::shared_ptr<SystemBlock>> systems_;

    };
} // namespace Systems

#endif // NOMAD_CORE_CONTROLLERS_BLOCKDIAGRAM_H_
