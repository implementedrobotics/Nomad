
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

namespace OptimalControl
{
namespace LinearOptimalControl
{
LinearCondensedOCP::LinearCondensedOCP(const int N,
                                       const double T,
                                       const int num_states,
                                       const int num_inputs,
                                       const bool time_varying,
                                       const int max_iterations) : LinearOptimalControlProblem(N, T, num_states, num_inputs, max_iterations)
{

    //U_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs * (N - 1), 1);

    A_N_ = EigenHelpers::BlockMatrixXd(N, 1, num_states, num_states);
    B_N_ = EigenHelpers::BlockMatrixXd(N, N - 1, num_states, num_inputs);

    H_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs * (N - 1), num_inputs * (N - 1));
    g_ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs * (N - 1), 1);

    qp_ = qpOASES::QProblem(num_inputs * N, 0);

    // TODO: Push this up?  Or have a specific qpOASES type?
    qpOASES::Options myOptions;
    //myOptions.enableRamping = BT_FALSE;
    //myOptions.maxPrimalJump = 1;
    //myOptions.setToMPC();
    qp_.setPrintLevel(qpOASES::PL_HIGH);
    qp_.setOptions(myOptions);
}

void LinearCondensedOCP::Solve()
{
    qp_.init(H_.data(), g_.data(), NULL, NULL, NULL, NULL, NULL, max_iterations_);
    //Eigen::MatrixXd x_out(num_inputs_ * N_, 1);

    qp_.getPrimalSolution(U_.data());
}

} // namespace LinearOptimalControl
} // namespace OptimalControl
