/*
 * PlotterTaskNode.hpp
 *
 *  Created on: July 17, 2019
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

#ifndef NOMAD_PLOTTING_PLOTTERTASKNODE_H_
#define NOMAD_PLOTTING_PLOTTERTASKNODE_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>
#include <vector>

// Third Party Includes
#include <Eigen/Dense>

// Project Includes
#include <Realtime/Messages/double_vec_t.hpp>
#include <Realtime/RealTimeTask.hpp>


// TODO: Some of this functionality shoud be in a "Logger" node.  The logger would then connect to the plotter.
namespace Plotting
{
class PlotterTaskNode : public Realtime::RealTimeTaskNode
{

public:

    enum InputPort
    {
        PORT_1 = 0, // Input Port 1
        PORT_2 = 1, // Input Port 2
        PORT_3 = 2, // Input Port 3
        PORT_4 = 3, // Input Port 4
        PORT_5 = 4, // Input Port 5
        PORT_6 = 5, // Input Port 6
        PORT_7 = 6, // Input Port 7
        PORT_8 = 7, // Input Port 8
        MAX_PORTS
    };

    // Base Class Convex Model Predictive Controller Locomotion Task Node
    // name = Task Name
    // N = Trajectory Steps
    // T = Trajectory Time Window
    PlotterTaskNode(const std::string &name);

    // TODO: Set plot params on the port
    // i.e. Plot style, Plot Names, Port Axis Names, etc.
    void RenderPlot(); // Plot the acquired data

    // Connect Input to Port Output
    void ConnectInput(InputPort port_id, std::shared_ptr<Realtime::Port> port);

    // Add a signal variable to plot
    // TODO: Which subplot is this going on, etc.
    void AddPlotVariable(InputPort port_id, int signal_idx);

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    // Plot Variable List
    std::vector<int> plot_vars_[MAX_PORTS];

    // Plot Data List (Doubles Only For Now)
    std::vector<Eigen::VectorXd> plot_data_[MAX_PORTS];

    // Hold time points
    std::vector<double> time_data_[MAX_PORTS];

    // Messages
    double_vec_t port_message_;

    // Buffer Size
    uint64_t sample_window_;

    // Add Subplots/Scopes

    // TODO: Type:
    // Realtime/Window/Trigger Event
    // TODO: Additional Types

    // Cached
    // TODO: Axis Properties Class (Color, style, Name, Blah)
    
};
} // namespace Plotting

#endif // NOMAD_PLOTTING_PLOTTERTASKNODE_H_
