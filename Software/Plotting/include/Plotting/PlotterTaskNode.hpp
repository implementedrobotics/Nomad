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
#include <Controllers/Messages.hpp>
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

    // Base Class Plotter Task Node
    // name = Task Name
    // stack_size = Task Thread Stack Size
    // rt_priority = Task Thread Priority
    // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
    // rt_core_id = CPU Core to pin the task.  -1 for no affinity
    PlotterTaskNode(const std::string &name = "Plotter_Task_Node",
                   const long rt_period = 10000,
                   const unsigned int rt_priority = Realtime::Priority::MEDIUM,
                   const int rt_core_id = -1,
                   const unsigned int stack_size = PTHREAD_STACK_MIN);

    // TODO: Set plot params on the port
    // i.e. Plot style, Plot Names, Port Axis Names, etc.
    virtual void RenderPlot(); // Plot the acquired data

    void SetPortDimension(const unsigned int port_id, const unsigned int port_dim);


protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    std::vector<Eigen::VectorXd> plot_data_[MAX_PORTS];

    // Input Port DImensions
    std::vector<int> port_dims_;

    // TODO: Axis Properties Class (Color, style, Name, Blah)
    

private:
    int sequence_num_;
};
} // namespace Plotting

#endif // NOMAD_PLOTTING_PLOTTERTASKNODE_H_
