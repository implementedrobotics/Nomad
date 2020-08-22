/*
 * RigidBodyGRFSolverQP.cpp
 *
 *  Created on: August 21, 2020
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
#include <Nomad/Controllers/RigidBodyGRFSolverQP.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <memory>

// Third-Party Includes

// Project Includes

#include <Common/Time.hpp>

namespace Robot::Nomad::Controllers
{
    RigidBodyGRFSolverQP::RigidBodyGRFSolverQP(const int num_contacts) : Core::OptimalControl::ConvexLinearSystemSolverQP(6, num_contacts * 3, num_contacts * 5)
    {
        // force_min_ = SOME BIG NEGATIVE
        // force_max_ = SOME BIG POSITIVE
    }

    void RigidBodyGRFSolverQP::SetForceEnvelope(Eigen::Vector3d force_min, Eigen::Vector3d force_max)
    {
        force_min_ = force_min;
        force_max_ = force_max_;
    }



    void RigidBodyGRFSolverQP::SetContactState(int idx, const ContactState &contact)
    {

    }

    void RigidBodyGRFSolverQP::SetDesiredState(Eigen::VectorXd x)
    {

    }

    void RigidBodyGRFSolverQP::Solve()
    {
        // Update Solver Parameters
        Core::OptimalControl::ConvexLinearSystemSolverQP::Solve();
    }

    void RigidBodyGRFSolverQP::UpdateConstraints()
    {

    }

} // namespace Robot::Nomad::Controllers
