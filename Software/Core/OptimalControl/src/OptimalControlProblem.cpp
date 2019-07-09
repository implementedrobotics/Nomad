
/*
 * OptimalControlProblem.cpp
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

#include <OptimalControl/OptimalControlProblem.hpp>

namespace OptimalControl
{

OptimalControlProblem::OptimalControlProblem(const unsigned int N,
                                             const double T,
                                             const unsigned int num_states,
                                             const unsigned int num_inputs,
                                             const unsigned int max_iterations) : T_(T), N_(N), num_states_(num_states), num_inputs_(num_inputs), max_iterations_(max_iterations)
{
    // Initial Condition
    x_0_ = Eigen::VectorXd(num_states);
    
    // Target Reference
    X_ref_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, N);

    // State/Input Trajectories
    X_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, N);
    U_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs, N - 1);

    // State/Input Weights
    Q_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_states);
    R_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs, num_inputs);

    // Compute Sampling Time
    T_s_ = T_ / N_;
}
} // namespace OptimalControl

namespace OptimalControl
{
namespace LinearOptimalControl
{
LinearOptimalControlProblem::LinearOptimalControlProblem(const unsigned int N,
                                                         const double T,
                                                         const unsigned int num_states,
                                                         const unsigned int num_inputs,
                                                         const unsigned int max_iterations) : OptimalControlProblem(N, T, num_states, num_inputs, max_iterations)
{
    A_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_states);
    B_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_inputs);
}
void LinearOptimalControlProblem::Solve()
{
}
} // namespace LinearOptimalControl
} // namespace OptimalControl