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
#include <assert.h>

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <memory>

// Third-Party Includes

// Project Includes
#include <Common/Math/MathUtils.hpp>

#include <Common/Time.hpp>

namespace Robot::Nomad::Controllers
{
    RigidBodyGRFSolverQP::RigidBodyGRFSolverQP(const int num_contacts) : 
    Core::OptimalControl::ConvexLinearSystemSolverQP(6, num_contacts * 3, num_contacts * 5), 
    num_contacts_(num_contacts),
    normal_force_min_(0.0),
    normal_force_max_(100.0),
    mass_(10.0),
    gravity_(0,0,9.81)
    {
        // Reserve Contact List
        contacts_.reserve(num_contacts);
        // Update MOI Values
        SetCentroidalMOI(0.01);

        std::cout << "Here now: " << &solver_time_ << std::endl;
    }

    void RigidBodyGRFSolverQP::SetNormalForceEnvelope(double force_min, double force_max)
    {
        normal_force_min_ = force_min;
        normal_force_max_ = force_max;
    }

    void RigidBodyGRFSolverQP::SetContactState(int idx, const ContactState &contact)
    {
        assert(idx >= 0 && idx < num_contacts_);

        contacts_[idx] = contact;
    }

    void RigidBodyGRFSolverQP::SetCurrentState(Eigen::VectorXd x)
    {
        x_ = x;
    }

    void RigidBodyGRFSolverQP::SetDesiredState(Eigen::VectorXd x)
    {
        x_desired_ = x;
    }

    void RigidBodyGRFSolverQP::Solve()
    {
        // Get Current State
        Eigen::Vector3d theta_base = x_.segment(0, 3);
        Eigen::Vector3d x_com = x_.segment(3, 3);
        Eigen::Vector3d omega_base = x_.segment(6, 3);
        Eigen::Vector3d x_com_dot = x_.segment(9, 3);

        // Get Desired State
        Eigen::Vector3d theta_base_desired = x_desired_.segment(0, 3);
        Eigen::Vector3d x_com_desired = x_desired_.segment(3, 3);
        Eigen::Vector3d omega_base_desired = x_desired_.segment(6, 3);
        Eigen::Vector3d x_com_dot_desired = x_desired_.segment(9, 3);

        std::cout << "COM POS: " << std::endl << x_com << std::endl;

        // RPY -> Quaternion -> Orientation Error
        Eigen::Quaterniond orientation = Eigen::AngleAxisd(theta_base(0), Eigen::Vector3d::UnitX()) *
                                         Eigen::AngleAxisd(theta_base(1), Eigen::Vector3d::UnitY()) *
                                         Eigen::AngleAxisd(theta_base(2), Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond orientation_desired = Eigen::AngleAxisd(theta_base_desired(0), Eigen::Vector3d::UnitX()) *
                                                 Eigen::AngleAxisd(theta_base_desired(1), Eigen::Vector3d::UnitY()) *
                                                 Eigen::AngleAxisd(theta_base_desired(2), Eigen::Vector3d::UnitZ());

        // Compute Orientation Error
        Eigen::Quaterniond orientation_error = orientation_desired * orientation.conjugate();

        std::cout << "Orientation Error: " << std::endl << orientation_error.vec() << std::endl;

        // PD Control Law
        Eigen::Vector3d x_com_dd_desired = K_p_com_ * (x_com_desired - x_com) + K_d_com_ * (x_com_dot_desired - x_com_dot);
        Eigen::Vector3d omega_base_dot_desired = K_p_base_ * (orientation_error.vec() * Common::Math::sgn(orientation_error.w())) + K_d_base_ * (omega_base_desired - omega_base);

        std::cout << "X_COM DD: " << std::endl << x_com_dd_desired << std::endl;

        // Update Solver Parameters

        std::cout << "A: " << A_.rows() << " " << A_.cols() << std::endl;
        // Update A Matrix
        for(int i = 0; i < num_contacts_; i++)
        {
            ContactState contact = contacts_[i];

            A_.block<3, 3>(0, i * 3) = Eigen::Matrix3d::Identity();
            A_.block<3, 3>(3, i * 3) = Common::Math::SkewSymmetricCrossProduct(x_com-contact.pos_world);
        }

        // Update B Matrix
        b_.head(3) = mass_ * (x_com_dd_desired + gravity_);
        b_.tail(3) = I_g_ * omega_base_dot_desired;

        UpdateConstraints();
        Core::OptimalControl::ConvexLinearSystemSolverQP::Solve();
    }

    void RigidBodyGRFSolverQP::UpdateConstraints()
    {
        // TODO: Add Constraints
    }

} // namespace Robot::Nomad::Controllers
