/*
 * RigidBodyGRFSolver.hpp
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

#ifndef ROBOT_NOMAD_CONTROLLERS_RIGIDBODYGRFSOLVER_H_
#define ROBOT_NOMAD_CONTROLLERS_RIGIDBODYGRFSOLVER_H_

// C System Files

// C++ System Files
#include <iostream>
#include <string>
#include <memory>

// Project Include Files
#include <OptimalControl/ConvexMatrixEquationSolverQP.hpp>

namespace Robot::Nomad::Controllers
{
    struct ContactState
    {
        int contact;                            // 1 = Contact, 0 = No Contact
        double mu;                              // Friction at Contact
        Eigen::Vector3d pos;                    // Position of Contact in WCS (Undefined when not in contact)
        Eigen::MatrixXd J;                      // Contact Jacobian (Undefined when not in contact)
        Eigen::Quaterniond surface_orientation; // Contact Surface Orientation.  (Undefined when not in contact)
                                                // For Normal and Tangetial Friction Cone Computations (Undefined when not in contact)
    };

    class RigidBodyGRFSolverQP : public Core::OptimalControl::ConvexLinearSystemSolverQP
    {

    public:
        // Base Class RigidBodyGRFSolver
        // Simplfified Centroidal Dynamics Solver for Floating Base/Legged Robots
        // num_contacts = Number of ground contacts in system
        RigidBodyGRFSolverQP(const int num_contacts);

        // Set allowable force range for optimization
        void SetForceEnvelope(Eigen::Vector3d force_min, Eigen::Vector3d force_max);

        // Set Weights
        void SetAlpha(double alpha) { alpha_ = alpha; }            // Force Minimization Influence
        void SetBeta(double beta) { beta_ = beta; };                   // Solution Filtering Influence
        void SetControlWeights(Eigen::VectorXd weights) { S_ = weights;}           // Control weights between base position and orientation
        void SetMinimizationWeights(Eigen::VectorXd weights) { W_1_ = weights;}   // Force minimization Weight
        void SetSolutionFilteringWeights(Eigen::VectorXd weights) { W_2_ = weights;} // Solution Filtering Weight

        // idx = Index of contact
        // contact = New contact state
        void SetContactState(int idx, const ContactState &contact);

        // Set Desired State for Force Computation
        // x = [Θ^T, p^T, ω^T, p_dot^T]^T | Θ = orientation, p = position, ω = angular velocity, p_dot = velocity
        void SetDesiredState(Eigen::VectorXd x);

        // Update Problem parameters and solve
        void Solve();

    protected:
        // Update QP Problem information prior to solve
        void UpdateConstraints();

        Eigen::Vector3d force_min_;
        Eigen::Vector3d force_max_;
        Eigen::Vector3d com_pos_;             // WCS Position of Floating Base CoM
        Eigen::Quaterniond base_orientation_; // Orientation of Floating Base

        // Translational P/D Gains
        Eigen::MatrixXd k_P_pos_;
        Eigen::MatrixXd k_D_pos_;

        // Orientation P/D Gains
        Eigen::MatrixXd k_P_orientation_;
        Eigen::MatrixXd k_D_orientation_;

        Eigen::MatrixXd inetia; // System Inertia
        double mass_;           // System mass

        std::vector<ContactState> contacts_; // List of contacts in system
        int num_contacts_;                   // Number of ground contacts in system
    };
} // namespace Robot::Nomad::Controllers

#endif // ROBOT_NOMAD_CONTROLLERS_RIGIDBODYGRFSOLVER_H_
