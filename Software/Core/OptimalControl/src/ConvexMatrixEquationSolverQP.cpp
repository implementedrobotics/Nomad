/*
 * ConvecMatrixEquationSolverQP.cpp
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


// C System Files

// C++ System Files
#include <chrono>

// Third Party Includes

// Project Includes
#include <OptimalControl/ConvexMatrixEquationSolverQP.hpp>


namespace Core::OptimalControl
{

    ConvexLinearSystemSolverQP::ConvexLinearSystemSolverQP(
        const unsigned int num_eq,
        const unsigned int num_vars,
        const unsigned int num_constraints) : num_equations_(num_eq),
                                       num_variables_(num_vars),
                                       num_constraints_(num_constraints),
                                       max_iterations_(5000),
                                       solved_(false),
                                       is_hot_(false),
                                       alpha_(1e-3),
                                       beta_(1e-3)
    {
        // Resize Matrices
        A_.resize(num_eq, num_vars);
        x_star_ = Eigen::VectorXd::Zero(num_vars);
        x_star_prev_ = Eigen::VectorXd::Zero(num_vars);
        b_.resize(num_eq);

        // Default to equal weights/Identity
        S_ = Eigen::MatrixXd::Identity(num_eq, num_eq);
        W_1_ = Eigen::MatrixXd::Identity(num_vars, num_vars);
        W_2_ = Eigen::MatrixXd::Identity(num_vars, num_vars);

        // Resize Matrices
        lbA_.resize(num_constraints_);
        ubA_.resize(num_constraints_);

        // qpOases is Row Major Formatted. Eigen defaults to Column Major
        H_qp_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_vars, num_vars);
        A_qp_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_constraints_, num_vars);
        g_qp_.resize(num_vars);
    }

    void ConvexLinearSystemSolverQP::Solve()
    {
        // Get starting timepoint
        auto start = std::chrono::high_resolution_clock::now();

        // Cache Previous Solution
        x_star_prev_ = x_star_;

        // Setup Problem
        H_qp_ =  2 * (A_.transpose() * S_ * A_ + alpha_ * (W_1_ + beta_ * W_2_));
        g_qp_ = -2 * (A_.transpose() * S_ * b_) - 2 * (beta_ * (x_star_prev_ * W_2_));

        // Get ending timepoint
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        solver_iterations_ = max_iterations_;
        if (is_hot_)
              qp_.hotstart(g_qp_.data(), lbA_.data(), ubA_.data(), solver_iterations_);
        else
        {
            //qp_.init(H_.data(), g_.data(), NULL, NULL, NULL, NULL, NULL, max_iterations_); (Constraint Version)
            qp_.init(H_qp_.data(), g_qp_.data(), lbA_.data(), ubA_.data(), solver_iterations_);
            is_hot_ = true;
        }

        // Get Solution
        // TODO: Check solved here
        qp_.getPrimalSolution(x_star_.data());

        std::cout << "Solver Time: " << duration.count() << " microseconds" << std::endl;
    }

    void ConvexLinearSystemSolverQP::PrintDebug()
    {
        // TODO: Print solver times, solution, iterations, etc
    }

} // namespace Core::OptimalControl
