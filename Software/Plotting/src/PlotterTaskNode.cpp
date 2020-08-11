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
#include <fstream>
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
        std::shared_ptr<Communications::Port> input = GetInputPort(i);
        if (input == nullptr)
        {
            continue;
        }

        if (input->Receive(port_message_))
        {
            Eigen::VectorXd msg_vec = Eigen::Map<Eigen::VectorXd>(port_message_.data.data(), port_message_.length);
            plot_data_[i].emplace_back(msg_vec);
            time_data_[i].emplace_back(port_message_.timestamp / 1e6);
        }
        else
        {
            //std::cout << "[PlotterTaskNode]: Receive Buffer Empty: " << i << std::endl;
            continue;
        }
    }
}

void PlotterTaskNode::Setup()
{
    // Connect Mapped Ports:
    for (int i = 0; i < InputPort::MAX_PORTS; i++)
    {
        std::shared_ptr<Communications::Port> input = GetInputPort(i);
        if (input == nullptr)
            continue;

        //input->Connect();
    }
}

void PlotterTaskNode::ConnectInput(InputPort port_id, std::shared_ptr<Communications::Port> output_port)
{
    // TODO: Make sure port is not already taken, etc.  If so error.
    std::shared_ptr<Communications::Port> in_port = std::make_shared<Communications::Port>(output_port->GetName(), Communications::Port::Direction::INPUT, Communications::Port::DataType::DOUBLE, -1, rt_period_);
    Communications::Port::Map(in_port, output_port); // Map It
    input_port_map_[port_id] = in_port;        // Cache It

    // Reserve
    plot_data_[port_id].reserve(10000);
}

void PlotterTaskNode::AddPlotVariable(InputPort port_id, int signal_idx)
{
    // TODO: Names, Styles, Blah, Blah
    plot_vars_[port_id].push_back(signal_idx);
}
void PlotterTaskNode::RenderPlot()
{
    std::vector<std::vector<double>> graph;

    // Loop all ports:
    for (int i = 0; i < InputPort::MAX_PORTS; i++)
    {
        for (int k = 0; k < plot_vars_[i].size(); k++)
        {
            std::vector<double> data;
            for (int j = 0; j < plot_data_[i].size(); j++)
            {
                int plot_var = plot_vars_[i][k];
                Eigen::VectorXd vec = plot_data_[i][j];
                data.emplace_back(vec[plot_var]);
            }
            graph.emplace_back(data);
        }
    }

    plt::style("seaborn");
    for (int i = 0; i < graph.size(); i++)
    {
        plt::plot(time_data_[0], graph[i]);
    }
    plt::show();
}

void PlotterTaskNode::DumpCSV(const std::string &filename)
{
    std::ofstream outputFile;
    outputFile.open(filename, std::ofstream::out | std::ofstream::trunc);

    // TODO: Revisit This
    for (int i = 0; i < plot_data_[0].size(); i++)
    {
        Eigen::VectorXd vec = plot_data_[0][i];
        outputFile << time_data_[0][i] << "," << vec[0] << std::endl;
    }

    outputFile.close();
}
} // namespace Plotting
