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
                                       max_iterations_(200),
                                       solved_(false),
                                       is_hot_(false),
                                       alpha_(1e-3),
                                       beta_(1e-3),
                                       qp_(num_vars, num_constraints)
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

        // Disable debug by default
        EnableQPDebug(false);
    }

    void ConvexLinearSystemSolverQP::Solve()
    {
        // Get starting timepoint
        auto start = std::chrono::high_resolution_clock::now();

        // Cache Previous Solution
        x_star_prev_ = x_star_;

        // Setup Problem
        H_qp_ =  2 * (A_.transpose() * S_ * A_ + alpha_ * W_1_ + beta_ * W_2_);
        g_qp_ = -2 * (A_.transpose() * S_ * b_) - 2 * (beta_ * (W_2_ * x_star_prev_));

        // Alternate Formulation 
        // H_qp_ =  2 * (A_.transpose() * S_ * A_ + (alpha_ * W_1_));
        // g_qp_ = -2 * (A_.transpose() * S_ * b_) - 2 * (x_star_prev_ * alpha_);

        solver_iterations_ = max_iterations_;

        qpOASES::int_t qp_ret;
        if (is_hot_)
        {
            qp_ret = qp_.hotstart(H_qp_.data(), g_qp_.data(), A_qp_.data(), NULL, NULL, lbA_.data(), ubA_.data(), solver_iterations_);
        }
        else
        {
            // std::cout << "H_qp: " << std::endl;
            // std::cout << H_qp_ << std::endl;

            // std::cout << "g_qp: " << std::endl;
            // std::cout << g_qp_ << std::endl;

            // std::cout << "lbA: " << std::endl;
            // std::cout << lbA_ << std::endl;

            // TODO: For some reason can't set from contructor variable when using cpu_time flag
            qp_ret = qp_.init(H_qp_.data(), g_qp_.data(), A_qp_.data(), NULL, NULL, lbA_.data(), ubA_.data(), solver_iterations_);
            is_hot_ = true;
        }

        // Get ending timepoint
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        // Get Solution
        // TODO: Check solved here
        if(qp_ret == qpOASES::SUCCESSFUL_RETURN)
        {
            // Successful now get the solution
            qp_.getPrimalSolution(x_star_.data());
            std::cout << "Successful Solve: " << solver_iterations_ << " iterations" << "| Time: " << duration.count() << " microseconds" << std::endl;
            std::cout << "Value: " << std::endl;
            std::cout << x_star_ << std::endl;
        }
        else
        {
            std::cout << "Failed to solve QP! " << qp_ret << std::endl;

            // TODO: Switch error codes possible here

        }
    }

    void ConvexLinearSystemSolverQP::EnableQPDebug(bool enable)
    {
        qpOASES::Options qp_opts;
        if(enable)
        {
            qp_opts.printLevel = qpOASES::PL_MEDIUM;
            qp_.setOptions(qp_opts);
            qp_.setPrintLevel(qpOASES::PL_MEDIUM);
        }
        else
        {
            qp_opts.printLevel = qpOASES::PL_NONE;
            qp_.setOptions(qp_opts);
            qp_.setPrintLevel(qpOASES::PL_NONE);
        }
    }
    void ConvexLinearSystemSolverQP::PrintDebug()
    {
        // TODO: Print solver times, solution, iterations, etc
    }

} // namespace Core::OptimalControl
