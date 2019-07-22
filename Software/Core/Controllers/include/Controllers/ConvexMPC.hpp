/*
 * ConvexMPC.hpp
 *
 *  Created on: July 13, 2019
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

#ifndef NOMAD_CORE_CONTROLLERS_CONVEXMPC_H_
#define NOMAD_CORE_CONTROLLERS_CONVEXMPC_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>

// Project Include Files
#include <Realtime/RealTimeTask.hpp>
#include <Controllers/Messages.hpp>
#include <OptimalControl/OptimalControlProblem.hpp>
#include <OptimalControl/LinearCondensedOCP.hpp>
#include <Systems/RigidBody.hpp>

namespace Controllers
{
namespace Locomotion
{
class ConvexMPC : public Realtime::RealTimeTaskNode
{

public:

    enum OutputPort
    {
        FORCES = 0 // State Estimate
    };

    enum InputPort
    {
        STATE_HAT = 0,    // State Estimate
        REFERENCE_TRAJECTORY = 1 // Reference Trajectory
    };

    // Base Class Convex Model Predictive Controller Locomotion Task Node
    // name = Task Name
    // N = Trajectory Steps
    // T = Trajectory Time Window
    ConvexMPC(const std::string &name, const unsigned int N, const double T);


protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    // Optimal Control Problem
    OptimalControl::LinearOptimalControl::LinearCondensedOCP *ocp_;

    // TODO:
    // Dynamic System Block
    RigidBlock1D block_;

    // State/Input Weights

    // Number of System States
    unsigned int num_states_;

    // Number of System Inputs
    unsigned int num_inputs_;

    // Prediction Steps
    unsigned int N_;

    // Prediction Length (s)
    double T_;

    // Sampling Time (s)
    double T_s_;

    // Input (State Estimate)
    Messages::Controllers::Estimators::CoMState x_hat_in_;

    // Input (Reference Trajectory)
    Messages::Controllers::Locomotion::ReferenceTrajectory reference_in_;

    // Output (Optimal Forces)
    Messages::Generic::Vector<1> force_output_;

private:
    int sequence_num_;
};
} // namespace Locomotion
} // namespace Controllers

#endif // NOMAD_CORE_CONTROLLERS_CONVEXMPC_H_
