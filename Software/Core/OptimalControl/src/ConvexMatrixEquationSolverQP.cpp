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
        const unsigned int num_vars) : num_equations_(num_eq),
                                       num_variables_(num_vars),
                                       max_iterations_(5000),
                                       solved_(false)
    {
        A_.resize(num_eq, num_vars);
        x_star_.resize(num_eq);
        x_star_prev_.resize(num_eq);
        b_.resize(num_eq);

        // Default to equal weights/Identity
        S_ = Eigen::MatrixXd::Identity(num_eq, num_eq);
        W_1_ = Eigen::MatrixXd::Identity(num_vars, num_vars);
        W_2_ = Eigen::MatrixXd::Identity(num_vars, num_vars);

        lb_.resize(num_constraints_);
        ub_.resize(num_constraints_);

        H_qp_.resize(num_vars, num_vars);
        A_qp_.resize(num_constraints_, num_vars);
        C_.resize(num_constraints_, num_vars);

        g_qp_.resize(num_vars);

    }

    void ConvexLinearSystemSolverQP::Solve()
    {
        // Get starting timepoint
        auto start = std::chrono::high_resolution_clock::now();

        // Cache Previous Solution
        x_star_prev_ = x_star_;

        // Setup Problem
        H_qp_ =  2 * (A_.transpose() * S_ * A_ + alpha * (W_1_ + beta * W_2_));
        g_qp_ = -2 * (A_.transpose() * S_ * b_) - 2 * (beta * (x_star_prev_ * W_2_));

        // Get ending timepoint
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        solver_iterations_ = max_iterations_;
        // if (is_hot)
        //      qp_.hotstart(g_qp_.data(), lb_.data(), ub_.data(), solver_iterations_);
        //  else
        {
            //qp_.init(H_.data(), g_.data(), NULL, NULL, NULL, NULL, NULL, max_iterations_); (Constraint Version)
            qp_.init(H_qp_.data(), g_qp_.data(), lb_.data(), ub_.data(), solver_iterations_);
            //      is_hot = true;
        }

        // Get Solution
        // TODO: Check solved here
        qp_.getPrimalSolution(x_star_.data());

        std::cout << "Solver Time: " << duration.count() << " microseconds" << std::endl;
    }

    void ConvexLinearSystemSolverQP::PrintDebug()
    {

    }

} // namespace Core::OptimalControl
