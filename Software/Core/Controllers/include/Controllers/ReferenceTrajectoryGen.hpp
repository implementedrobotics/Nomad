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
#include <Realtime/RealTimeTask.hpp>
#include <Communications/Messages/double_vec_t.hpp>

namespace Controllers::Locomotion
{

    // TODO: This could just be a base class.  For now it is our specific trajectory generator for the convex mpc.  i.e. Number of states is fixed, and uses all assumptions (like 0 z-velocity, etc)
    // Long story short this is not universal...
    class ReferenceTrajectoryGenerator : public Realtime::RealTimeTaskNode
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
        // N = Trajectory Steps
        // T = Trajectory Time Window
        ReferenceTrajectoryGenerator(const std::string &name, const unsigned int N, const double T);

    protected:
        // Overriden Run Function
        virtual void Run();

        // Pre-Run Setup Routine.  Setup any one time initialization here.
        virtual void Setup();

        // Trajectory State
        Eigen::MatrixXd X_ref_;

        // Number of System States
        int num_states_;

        // Number of Sample Points
        int N_;

        // Sample Time
        double T_s_;

        // Horizon Length
        double T_;

        // Input (State Estimate)
        double_vec_t x_hat_in_;

        // Input (Setpoint)
        double_vec_t setpoint_in_;

        // Output (Reference Trajectory)
        double_vec_t reference_out_;
    };
} // namespace Controllers::Locomotion

#endif // NOMAD_CORE_CONTROLLERS_REFERENCETRAJECTORYGEN_H_
