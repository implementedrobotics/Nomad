
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
    lb_ = Eigen::VectorXd(num_vars_);
    ub_ = Eigen::VectorXd(num_vars_);
    lbC_ = Eigen::VectorXd(num_cons_);
    ubC_ = Eigen::VectorXd(num_cons_);

    qp_ = qpOASES::QProblemB(num_vars_);

    // TODO: Push this up?  Or have a specific qpOASES type?
    qpOASES::Options myOptions;
    //myOptions.enableRamping = BT_FALSE;
    //myOptions.maxPrimalJump = 1;
    //myOptions.setToMPC();
    qp_.setPrintLevel(qpOASES::PL_HIGH);
    qp_.setOptions(myOptions);
}

void LinearCondensedOCP::Condense()
{
    A_N_.SetBlock(0,0, Eigen::MatrixXd::Identity(num_states_, num_states_));

    for (int i = 0; i < N_ - 1; i++)
    {
        A_N_.SetBlock(i + 1, 0, A_N_(i, 0) * A_[i]);
        B_N_.FillDiagonal(A_N_(i,0)*B_[i], (-i-1));
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
    //bm_test.FillDiagonal(block_val, 0);


    H_ = 2 * (B_N_.MatrixXd().transpose()*Q_*B_N_.MatrixXd() + R_);
    g_ = 2 * B_N_.MatrixXd().transpose()*Q_*(A_N_.MatrixXd() * x_0_);// - X_ref_);

    //qp_.init(H_.data(), g_.data(), NULL, NULL, NULL, NULL, NULL, max_iterations_);
    qp_.init(H_.data(), g_.data(), NULL, NULL, max_iterations_);

    qp_.getPrimalSolution(U_.data());

    // Get ending timepoint 
    auto stop = std::chrono::high_resolution_clock::now(); 
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 

    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl; 

    qp_.hotstart(g_.data(), NULL, NULL, max_iterations_);

    qp_.getPrimalSolution(U_.data());


    // Get ending timepoint 
    auto stop2 = std::chrono::high_resolution_clock::now(); 
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop2 - stop); 

    std::cout << "Time taken by function Hot: " << duration.count() << " microseconds" << std::endl; 


}

} // namespace LinearOptimalControl
} // namespace OptimalControl
