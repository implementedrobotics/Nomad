
/*
 * LinearCondensedOCP.cpp
 *
 *  Created on: July 7, 2019
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

#include <OptimalControl/LinearCondensedOCP.hpp>
#include <chrono>

using namespace ControlsLibrary;

namespace OptimalControl
{
namespace LinearOptimalControl
{
LinearCondensedOCP::LinearCondensedOCP(const unsigned int N,
                                       const double T,
                                       const unsigned int num_states,
                                       const unsigned int num_inputs,
                                       const bool time_varying,
                                       const unsigned int max_iterations) : LinearOptimalControlProblem(N, T, num_states, num_inputs, max_iterations)
{

    A_N_ = EigenHelpers::BlockMatrixXd(N, 1, num_states, num_states);
    B_N_ = EigenHelpers::BlockMatrixXd(N, N - 1, num_states, num_inputs);

    num_vars_ = num_inputs * (N - 1);
    num_cons_ = num_inputs * (N - 1);

    H_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_vars_, num_vars_);
    g_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_vars_, 1);
    C_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_cons_, num_vars_);

    // Default to Unbounded
    lb_ = Eigen::VectorXd::Constant(num_vars_, -50);
    ub_ = Eigen::VectorXd::Constant(num_vars_, 50);

    // Default to Unbounded
    lbC_ = Eigen::VectorXd::Constant(num_cons_, -qpOASES::INFTY);
    ubC_ = Eigen::VectorXd::Constant(num_cons_, qpOASES::INFTY);

    // Setup QP
    is_hot = false; // Default to Cold Start
    qp_ = qpOASES::QProblemB(num_vars_);

    // TODO: Push this up?  Or have a specific qpOASES type?
    qpOASES::Options qp_options;

    // TODO: Not all OCPs are MPC.  Make this a flag.  Otherwise you can do a more "accurate" solver setup
    qp_options.setToMPC(); // Default to Fast MPC Options
    qp_options.printLevel = qpOASES::PL_LOW;
    //qp_options.terminationTolerance = 1e-4;
    //qp_options.boundTolerance = 1e-4;
    qp_.setOptions(qp_options);
}

void LinearCondensedOCP::Condense()
{
    A_N_.SetBlock(0, 0, Eigen::MatrixXd::Identity(num_states_, num_states_));

    for (int i = 0; i < N_ - 1; i++)
    {
        A_N_.SetBlock(i + 1, 0, A_N_(i, 0) * A_[i]);
        B_N_.FillDiagonal(A_N_(i, 0) * B_[i], (-i - 1));
    }

    //std::cout << "Condensed A: " << std::endl;
    //std::cout << A_N_ << std::endl;

    //std::cout << "Condensed B: " << std::endl;
    //std::cout << B_N_ << std::endl;
}

void LinearCondensedOCP::Solve()
{
    /* std::cout << "Solve:" << std::endl;
    std::cout << B_N_.MatrixXd().transpose().rows() << std::endl;
    std::cout << B_N_.MatrixXd().transpose().cols() << std::endl;
    std::cout << Q_.rows() << std::endl;
    std::cout << Q_.cols() << std::endl;
    std::cout << B_N_.MatrixXd().rows() << std::endl;
    std::cout << R_.rows() << std::endl;
    std::cout << R_.cols() << std::endl;*/

    // Get starting timepoint
    auto start = std::chrono::high_resolution_clock::now();

    // Reshape and Flatten to 1D
    Eigen::Map<const Eigen::VectorXd> X_ref(X_ref_.data(), X_ref_.size());

    // Make some temp variables.  Not sure why this is necessary but having some segfaults without.
    Eigen::MatrixXd A_N = A_N_.MatrixXd();
    Eigen::MatrixXd B_N = B_N_.MatrixXd();
    Eigen::MatrixXd B_N_T = B_N.transpose();

    Eigen::MatrixXd m = B_N_T * Q_ * B_N;
    Eigen::MatrixXd n = (A_N * x_0_);
    H_ = 2 * (m + R_);
    g_ = 2 * B_N_T * Q_ * ((n)-X_ref);
    
    solver_iterations_ = max_iterations_;
    if (is_hot)
        qp_.hotstart(g_.data(), lb_.data(), ub_.data(), solver_iterations_);
    else
    {
        //qp_.init(H_.data(), g_.data(), NULL, NULL, NULL, NULL, NULL, max_iterations_); (Constraint Version)
        qp_.init(H_.data(), g_.data(), lb_.data(), ub_.data(), solver_iterations_);
        is_hot = true;
    }

    // Get Solution
    qp_.getPrimalSolution(U_.data());

    // Get ending timepoint
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "Solver Time: " << duration.count() << " microseconds" << std::endl;

}

} // namespace LinearOptimalControl
} // namespace OptimalControl
