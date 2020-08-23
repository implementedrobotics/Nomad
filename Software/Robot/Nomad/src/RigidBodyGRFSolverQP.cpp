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
#include <OptimalControl/ControlsLibrary.hpp>
#include <Common/Math/MathUtils.hpp>
#include <Common/Time.hpp>

using namespace ControlsLibrary;

namespace Robot::Nomad::Controllers
{
    RigidBodyGRFSolverQP::RigidBodyGRFSolverQP(const int num_contacts) : 
    Core::OptimalControl::ConvexLinearSystemSolverQP(6, num_contacts * 3, num_contacts * 5), 
    num_contacts_(num_contacts),
    normal_force_min_(-500.0),
    normal_force_max_(500.0),
    mass_(10.0),
    gravity_(0,0,9.81)
    {
        // Reserve Contact List
        contacts_.reserve(num_contacts);

        // Update MOI Values
        SetCentroidalMOI(0.01);
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
        x_com_dd_desired[2] = 2000.0;
        // Update Solver Parameters

        // Update A Matrix
        for(int i = 0; i < num_contacts_; i++)
        {
            ContactState contact = contacts_[i];

            A_.block<3, 3>(0, i * 3) = Eigen::Matrix3d::Identity();
            A_.block<3, 3>(3, i * 3) = Common::Math::SkewSymmetricCrossProduct(x_com-x_com);//x_com-contact.pos_world);
        }

        // Update B Matrix
        b_.head(3) = mass_ * (x_com_dd_desired + gravity_);
        b_.tail(3) = I_g_ * omega_base_dot_desired;

        std::cout << "A: " << std::endl << A_ << std::endl;
        std::cout << "B: " << std::endl << b_ << std::endl;

        UpdateConstraints();
        Core::OptimalControl::ConvexLinearSystemSolverQP::Solve();
    }

    void RigidBodyGRFSolverQP::UpdateConstraints()
    {       
        // TODO: Change "5" to variable 
        // Eq. (7) and Eq. (8) High-slope Terrain Locomotion for Torque-Controlled Quadruped Robots

        // TODO: Move this to subclass variables
        EigenHelpers::BlockMatrixXd C = EigenHelpers::BlockMatrixXd(num_contacts_, num_contacts_, 5, 3, 0);
        
        Eigen::MatrixXd C_i = Eigen::MatrixXd(5,3);

        Eigen::VectorXd d_lower_i = Eigen::VectorXd(5);
        Eigen::VectorXd d_upper_i = Eigen::VectorXd(5);

        // Lower Bounds From Eq. (8)
        d_lower_i(0) = -qpOASES::INFTY/10000000000;
        d_lower_i(1) = -qpOASES::INFTY/10000000000;
        d_lower_i(2) = 0;
        d_lower_i(3) = 0;
        d_lower_i(4) = normal_force_min_;

        // Upper Bounds From Eq. (8)
        d_upper_i(0) = 0;
        d_upper_i(1) = 0;
        d_upper_i(2) = qpOASES::INFTY/100000000000;
        d_upper_i(3) = qpOASES::INFTY/100000000000;
        d_upper_i(4) = normal_force_max_;

        //d_lower_i << -qpOASES::INFTY << -qpOASES::INFTY << 0.0 << 0.0 << normal_force_min_;
        //d_upper_i << 0.0 << 0.0 << qpOASES::INFTY << qpOASES::INFTY << normal_force_max_;

        // Loop contacts and add our force constraints
        for(int i = 0; i < num_contacts_; i++)
        {
            ContactState contact = contacts_[i];

            // TODO: Convert Surface Normal Orientation -> Plane Normal and Tangent Vector
            Eigen::Vector3d n_i = Eigen::Vector3d::UnitZ();
            Eigen::Vector3d t_1_i = Eigen::Vector3d::UnitX();
            Eigen::Vector3d t_2_i = Eigen::Vector3d::UnitY();

            // Friction Cone Constraints
            // Eq. (22), Eq. (23) and Eq. (24) Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control
            // |F_x| < |mu*F_z| or -mu*F_z <= F_x <= mu*F_z
            // |F_y| < |mu*F_z| or -mu*F_z <= F_y <= mu*F_z
            // 0 < F_min < F_z < F_max 
            C_i.row(0) = (-contact.mu*n_i + t_1_i).transpose(); // F_x >= -mu*F_z
            C_i.row(1) = (-contact.mu*n_i + t_2_i).transpose(); // F_y >= -mu*F_z
            C_i.row(2) = (contact.mu*n_i + t_2_i).transpose();   // F_y <=  mu*F_z
            C_i.row(3) = (contact.mu*n_i + t_1_i).transpose();   // F_x <=  mu*F_z
            C_i.row(4) = (n_i).transpose();   // F_min <= F_z <= F_max

            // Update Big Constraint Matrix
            C(i,i) = C_i;

            // Update Bounds.  Zero any forces with legs not in contact.  Can't make force without something to push against.
            lbA_.segment(i*5,5) = d_lower_i * contact.contact;
            ubA_.segment(i*5,5) = d_upper_i * contact.contact;
        }
        A_qp_ = C;
        std::cout << "Aqp: " << std::endl;
        std::cout << A_qp_ << std::endl;

        std::cout << "lb: " << std::endl;
        std::cout << lbA_ << std::endl;

        std::cout << "ub: " << std::endl;
        std::cout << ubA_ << std::endl;

    }

} // namespace Robot::Nomad::Controllers
