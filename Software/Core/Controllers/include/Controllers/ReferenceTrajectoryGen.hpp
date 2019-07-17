/*
 * ReferenceTrajectoryGenerator.hpp
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

#ifndef NOMAD_CORE_CONTROLLERS_REFERENCETRAJECTORYGEN_H_
#define NOMAD_CORE_CONTROLLERS_REFERENCETRAJECTORYGEN_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>

// Project Include Files
#include <Controllers/RealTimeTask.hpp>

namespace Controllers
{
namespace Locomotion
{
class ReferenceTrajectoryGenerator : public RealTimeControl::RealTimeTaskNode
{

public:
    enum OutputPort
    {
        REFERENCE = 0 // Trajectory Reference
    };

    enum InputPort
    {
        STATE_HAT = 0, // State Estimate
        SETPOINT = 1   // Input Setpoint (Operator)
    };

    // Base Class Reference Trajectory Generator Task Node
    // name = Task Name
    // stack_size = Task Thread Stack Size
    // rt_priority = Task Thread Priority
    // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
    // rt_core_id = CPU Core to pin the task.  -1 for no affinity
    ReferenceTrajectoryGenerator(const std::string &name, const unsigned int N, const double T);

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    // Trajectory State
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> X_ref_;

    int num_states_; // Number of System States

    int N_; // Number of Sample Points

    double T_s_; // Sample Time
    double T_;   // Horizon Length

private:
    int reference_sequence_num_;
};
} // namespace Locomotion
} // namespace Controllers

#endif // NOMAD_CORE_CONTROLLERS_REFERENCETRAJECTORYGEN_H_
