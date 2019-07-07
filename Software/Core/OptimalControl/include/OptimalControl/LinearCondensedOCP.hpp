
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

#include <OptimalControl/OptimalControlProblem.hpp>
#include <vector>

#ifndef NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_
#define NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_

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
    LinearCondensedOCP(const int &N, const double &T, const int &num_states, const int &num_inputs, const bool time_varying = false);

    // Set Model Matrices (Time Invariant)
    void SetModelMatrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
    {
        // TODO: Check for Time Varying Here
        // TODO: Check Dimensions and make sure they match num_states/inputs
        _A[0] = A;
        _B[0] = B;
    }

    // Set Model Matrices (Time Varying)
    void SetModelMatrices(const std::vector<Eigen::MatrixXd> &A, const std::vector<Eigen::MatrixXd> &B)
    {
        _A = A;
        _B = B;
    }

    // Solve
    virtual void Solve();

protected:
    std::vector<Eigen::MatrixXd> _A; // System State Transition Matrix (Vector List for Time Varying)
    std::vector<Eigen::MatrixXd> _B; // Input Matrix (Vector List for Time Varying)

    Eigen::MatrixXd _A_N; // Condensed System State Transition Matrix for QP
    Eigen::MatrixXd _B_N; // Condensed Input Matrix for QP

    Eigen::MatrixXd _H; // Hessian
    Eigen::MatrixXd _g; 

    bool _time_varying;
};
} // namespace OptimalControl
} // namespace LinearOptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_