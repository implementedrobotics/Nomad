/*
 * LegController.hpp
 *
 *  Created on: July 1, 2019
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

#ifndef NOMAD_CORE_CONTROLLERS_LEGCONTROLLER_H_
#define NOMAD_CORE_CONTROLLERS_LEGCONTROLLER_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>

// Project Include Files
#include <Controllers/RealTimeTask.hpp>
#include <Controllers/Messages.hpp>

namespace Controllers
{
namespace Locomotion
{

// TODO: This could just be a base class.  For now it is our specific trajectory generator for the convex mpc.  i.e. Number of states is fixed, and uses all assumptions (like 0 z-velocity, etc)
// Long story short this is not universal...
class LegController : public RealTimeControl::RealTimeTaskNode
{

public:
    enum OutputPort
    {
        CONTROL = 0,  // Controller Output (Hardware)
        SIMULATION = 1    // Simulation Output (Sim)
    };

    enum InputPort
    {
        GROUND_REACTION = 0, // Ground Reaction Forces
        SWING_LEG = 1   // Swing Leg Impedance
    };

    // Base Class Reference Trajectory Generator Task Node
    // name = Task Name
    // N = Trajectory Steps
    // T = Trajectory Time Window
    LegController(const std::string &name, const unsigned int N, const double T);

protected:
    // Overriden Run Function
    virtual void Run();

    // Pre-Run Setup Routine.  Setup any one time initialization here.
    virtual void Setup();

    // Input (State Estimate)
    //Messages::Controllers::Estimators::CoMState x_hat_in_;

private:
    int sequence_num_;
};
} // namespace Locomotion
} // namespace Controllers

#endif // NOMAD_CORE_CONTROLLERS_LEGCONTROLLER_H_
