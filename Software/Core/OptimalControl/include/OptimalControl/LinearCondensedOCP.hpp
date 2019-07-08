
/*
 * LinearCondensedOCP.hpp
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

#include <qpOASES.hpp>
#include <OptimalControl/ControlsLibrary.hpp>
#include <OptimalControl/OptimalControlProblem.hpp>
#include <vector>

#ifndef NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_
#define NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_


using namespace ControlsLibrary;

namespace OptimalControl
{
namespace LinearOptimalControl
{
class LinearCondensedOCP : public LinearOptimalControlProblem
{
public:
    // Base Class Linear Optimal Control Problem
    // N = Prediction Steps
    // T = Horizon Length
    // num_states = Number of States of OCP
    // num_inputs = Number of Inputs of OCP
    LinearCondensedOCP(const int N, const double T, const int num_states, const int num_inputs, const bool time_varying = false, const int max_iterations = 1000);

    // Set Model Matrices (Time Invariant)
    void SetModelMatrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
    {
        // TODO: Check for Time Varying Here
        // TODO: Check Dimensions and make sure they match num_states/inputs
        A_[0] = A;
        B_[0] = B;
    }

    // Set Model Matrices (Time Varying)
    void SetModelMatrices(const std::vector<Eigen::MatrixXd> &A, const std::vector<Eigen::MatrixXd> &B)
    {
        A_ = A;
        B_ = B;
    }

    // Solve
    virtual void Solve();

protected:
    std::vector<Eigen::MatrixXd> A_; // System State Transition Matrix (Vector List for Time Varying)
    std::vector<Eigen::MatrixXd> B_; // Input Matrix (Vector List for Time Varying)

    EigenHelpers::BlockMatrixXd A_N_; // Condensed System State Transition Matrix for QP
    EigenHelpers::BlockMatrixXd B_N_; // Condensed Input Matrix for QP

    Eigen::MatrixXd H_; // Hessian
    Eigen::MatrixXd g_; 

    qpOASES::QProblem qp_;
    bool time_varying_;
};
} // namespace OptimalControl
} // namespace LinearOptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_