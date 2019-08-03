/*
 * ReferenceTrajectoryGenerator.cpp
 *
 *  Created on: July 16, 2019
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

// Primary Include
#include <Plotting/PlotterTaskNode.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>

// Third-Party Includes
#include <Eigen/Dense>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Plotting/matplotlibcpp.h>


// Plot Namespace:
namespace plt = matplotlibcpp;

namespace Plotting
{

PlotterTaskNode::PlotterTaskNode(const std::string &name) : Realtime::RealTimeTaskNode(name, 20000, Realtime::Priority::MEDIUM, -1, PTHREAD_STACK_MIN)
{

    input_port_map_.reserve(InputPort::MAX_PORTS);
    for (int i = 0; i < InputPort::MAX_PORTS; i++)
    {
        input_port_map_[i] = nullptr;
    }
}

void PlotterTaskNode::Run()
{
    // Get Inputs
    for (int i = 0; i < InputPort::MAX_PORTS; i++)
    {
        Realtime::Port *input = GetInputPort(i);
        if(input == nullptr)
        {
            continue;
        }

        if (input->Receive(port_message_))
        {
            Eigen::VectorXd msg_vec = Eigen::Map<Eigen::VectorXd>(port_message_.data.data(), port_message_.length);
            std::cout << "PlotNode: " << i <<  msg_vec << std::endl;
            plot_data_[i].push_back(msg_vec);
            time_data_[i].push_back(port_message_.timestamp);
        }
        else
        {
            std::cout << "[PlotterTaskNode]: Receive Buffer Empty: " << i << std::endl;
            continue;
        }
    }
}

void PlotterTaskNode::Setup()
{
    // Connect Mapped Ports:
    for (int i = 0; i < InputPort::MAX_PORTS; i++)
    {
        Realtime::Port *input = GetInputPort(i);
        if(input == nullptr)
            continue;

        input->Connect();
    }
    
}

void PlotterTaskNode::ConnectInput(InputPort port_id, Realtime::Port *output_port)
{
    // TODO: Make sure port is not already taken, etc.  If so error.
    Realtime::Port *in_port = new Realtime::Port(output_port->GetName(), Realtime::Port::Direction::INPUT, Realtime::Port::DataType::DOUBLE, -1, rt_period_);
    Realtime::Port::Map(in_port, output_port);

    input_port_map_[port_id] = in_port;

    // Reserve
    plot_data_[port_id].reserve(in_port->GetDimension());

}

void PlotterTaskNode::AddPlotVariable(InputPort port_id, int signal_idx)
{
    // TODO: Names, Styles, Blah, Blah
    plot_vars_[port_id].push_back(signal_idx);

}
void PlotterTaskNode::RenderPlot()
{
    std::vector<double> graph;
    // TODO: Loop Plot vars etc.
    for(int i = 0; i < plot_data_[0].size(); i++)
    {
        std::vector<Eigen::VectorXd> vec_list = plot_data_[0];
        for(int j = 0; j < vec_list.size(); j++)
        {
            graph.push_back(vec_list[j][0]);
        }
    }
    plt::style("seaborn");
    plt::plot(graph);
    plt::show();

}
} // namespace Plotting
