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

static constexpr int kNumConstraintsPerContact = 5;
static constexpr int kNumBodyDOF = 6;
static constexpr int kNumContactDOF = 3;

namespace Robot::Nomad::Controllers
{

    RigidBodyGRFSolverQP::RigidBodyGRFSolverQP(const int num_contacts) : Core::OptimalControl::ConvexLinearSystemSolverQP(kNumBodyDOF,
                                                                                                                          num_contacts * kNumContactDOF,
                                                                                                                          num_contacts * kNumConstraintsPerContact),
                                                                         num_contacts_(num_contacts),
                                                                         normal_force_min_(10),
                                                                         normal_force_max_(150.0),
                                                                         mass_(1.0),
                                                                         gravity_(0, 0, 9.81)
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

        Eigen::Vector3d theta_base_error = Common::Math::ComputeOrientationError(theta_base, theta_base_desired);


        Eigen::VectorXd w = Eigen::VectorXd(kNumBodyDOF);
        w << 1,1,1,200,200,200;
        SetControlWeights(w);
        K_p_com_ = Eigen::Vector3d(50,50,150).asDiagonal();
        K_d_com_ = Eigen::Vector3d(10,10,10).asDiagonal();

        K_p_base_ = Eigen::Vector3d(200,200,200).asDiagonal();
        K_d_base_ = Eigen::Vector3d(10,10,10).asDiagonal();

        // PD Control Law
        Eigen::Vector3d x_com_dd_desired = K_p_com_ * (x_com_desired - x_com) + K_d_com_ * (x_com_dot_desired - x_com_dot);
        Eigen::Vector3d omega_base_dot_desired = K_p_base_ * (theta_base_error) + K_d_base_ * (omega_base_desired - omega_base);

        // Update Solver Parameters
        // Update A Matrix
        for(int i = 0; i < num_contacts_; i++)
        {
            ContactState contact = contacts_[i];
            A_.block<3, 3>(0, i * 3) = Eigen::Matrix3d::Identity();
            A_.block<3, 3>(3, i * 3) = Common::Math::SkewSymmetricCrossProduct(contact.pos_world - x_com);
        }

        // Update B Matrix
        Eigen::Matrix3d R_z;
        R_z = Eigen::AngleAxisd(-theta_base(2),Eigen::Vector3d::UnitZ());
        std::cout << R_z << std::endl;
        Eigen::Matrix3d I_g = R_z * I_b_ * R_z.transpose();
        b_.head(3) = mass_ * (x_com_dd_desired + gravity_);
        b_.tail(3) = I_g * omega_base_dot_desired;

        UpdateConstraints();
        Core::OptimalControl::ConvexLinearSystemSolverQP::Solve();
    }

    void RigidBodyGRFSolverQP::UpdateConstraints()
    {       
        // TODO: Change "5" to variable 
        // Eq. (7) and Eq. (8) High-slope Terrain Locomotion for Torque-Controlled Quadruped Robots

        // TODO: Move this to subclass variables
        Common::Math::EigenHelpers::BlockMatrixXd C = Common::Math::EigenHelpers::BlockMatrixXd(num_contacts_, num_contacts_, kNumConstraintsPerContact, kNumContactDOF, 0);
        
        Eigen::MatrixXd C_i = Eigen::MatrixXd(kNumConstraintsPerContact,kNumContactDOF);

        Eigen::VectorXd d_lower_i = Eigen::VectorXd(kNumConstraintsPerContact);
        Eigen::VectorXd d_upper_i = Eigen::VectorXd(kNumConstraintsPerContact);

        // Lower Bounds From Eq. (8)
        d_lower_i(0) = -qpOASES::INFTY;
        d_lower_i(1) = -qpOASES::INFTY;
        d_lower_i(2) = 0;
        d_lower_i(3) = 0;
        d_lower_i(4) = normal_force_min_;

        // Upper Bounds From Eq. (8)
        d_upper_i(0) = 0;
        d_upper_i(1) = 0;
        d_upper_i(2) = qpOASES::INFTY;
        d_upper_i(3) = qpOASES::INFTY;
        d_upper_i(4) = normal_force_max_;

        //d_lower_i << -qpOASES::INFTY << -qpOASES::INFTY << 0.0 << 0.0 << normal_force_min_;
        //d_upper_i << 0.0 << 0.0 << qpOASES::INFTY << qpOASES::INFTY << normal_force_max_;

        // Loop contacts and add our force constraints
        for(int i = 0; i < num_contacts_; i++)
        {
            ContactState contact = contacts_[i];

            // TODO: Convert Surface Normal Orientation -> Plane Normal and Tangent Vector
            Eigen::Vector3d t_1_i = Eigen::Vector3d::UnitX();
            Eigen::Vector3d t_2_i = Eigen::Vector3d::UnitY();
            Eigen::Vector3d n_i = Eigen::Vector3d::UnitZ();


            // Friction Cone Constraints
            // Eq. (22), Eq. (23) and Eq. (24) Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control
            // |F_x| < |mu*F_z| or -mu*F_z <= F_x <= mu*F_z
            // |F_y| < |mu*F_z| or -mu*F_z <= F_y <= mu*F_z
            // 0 < F_min < F_z < F_max 
            C_i.row(0) = (-contact.mu * n_i.transpose() + t_1_i.transpose());  // F_x >= -mu*F_z
            C_i.row(1) = (-contact.mu * n_i.transpose() + t_2_i.transpose());  // F_y >= -mu*F_z
            C_i.row(2) = (contact.mu  * n_i.transpose() + t_2_i.transpose());   // F_y <=  mu*F_z
            C_i.row(3) = (contact.mu  * n_i.transpose() + t_1_i.transpose());   // F_x <=  mu*F_z
            C_i.row(4) = (n_i).transpose();   // F_min <= F_z <= F_max

            // Update Big Constraint Matrix
            C(i,i) = C_i;

            // Update Bounds.  Zero any forces with legs not in contact.  Can't make force without something to push against.
            lbA_.segment(i*kNumConstraintsPerContact, kNumConstraintsPerContact) = d_lower_i * contact.contact;
            ubA_.segment(i*kNumConstraintsPerContact, kNumConstraintsPerContact) = d_upper_i * contact.contact;
        }

        A_qp_ = C.MatrixXd();
        // std::cout << "Aqp Test: " << std::endl;
        // std::cout << A_qp_.data()[3] << std::endl;

        // std::cout << "Aqp: " << std::endl;
        // std::cout << A_qp_ << std::endl;

        // std::cout << "lb: " << std::endl;
        // std::cout << lbA_ << std::endl;

        // std::cout << "ub: " << std::endl;
        // std::cout << ubA_ << std::endl;
    }

} // namespace Robot::Nomad::Controllers
