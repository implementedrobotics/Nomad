
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
    // time_varying = Are system matrices time varying?
    // max_iterations = Maximum number of iterations for solve
    LinearCondensedOCP(const unsigned int N, 
    const double T, 
    const unsigned int num_states, 
    const unsigned int num_inputs, 
    const bool time_varying = false, 
    const unsigned int max_iterations = 1000);

    // Set Model Matrices (Time Invariant)
    void SetModelMatrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, bool condense = true)
    {
        // TODO: Check for Time Varying Here
        // TODO: Check Dimensions and make sure they match num_states/inputs
        //A_[0] = A;
        //B_[0] = B;

        // TODO: For now everything is considered "time varying".  Although it is just a constant matrix set over the prediction horizon
        // for(int i = 0; i < N_; i++)
        // {
        //     A_[i] = A;
        //     B_[i] = B;
        // }
        A_ = std::vector<Eigen::MatrixXd>(N_, A);
        B_ = std::vector<Eigen::MatrixXd>(N_, B);

        // Condense Formulation
        if (condense)
            Condense();
    }

    // Set Model Matrices (Time Varying)
    void SetModelMatrices(const std::vector<Eigen::MatrixXd> &A, const std::vector<Eigen::MatrixXd> &B, bool condense = true)
    {
        A_ = A;
        B_ = B;

        // Condense Formulation
        if (condense)
            Condense();
    }

    // Set Weight Matrices
    virtual void SetWeights(const Eigen::VectorXd &Q, const Eigen::VectorXd &R)
    {
        // TODO: Verify Vector Size Matches correct state and inputs
        Q_ = Q.replicate(N_, 1).matrix().asDiagonal().toDenseMatrix();

        R_ = R.replicate(N_ - 1, 1).matrix().asDiagonal().toDenseMatrix();
    }

    // Solve
    virtual void Solve();

protected:
    void Condense();

protected:
    std::vector<Eigen::MatrixXd> A_; // System State Transition Matrix (Vector List for Time Varying)
    std::vector<Eigen::MatrixXd> B_; // Input Matrix (Vector List for Time Varying)

    ControlsLibrary::EigenHelpers::BlockMatrixXd A_N_; // Condensed System State Transition Matrix for QP
    ControlsLibrary::EigenHelpers::BlockMatrixXd B_N_; // Condensed Control Input Matrix for QP

    Eigen::MatrixXd H_;   // Hessian Matrix
    Eigen::MatrixXd g_;   // Gradient Vector
    Eigen::MatrixXd C_;   // Constaint Matrix(A in qpOASES)
    Eigen::VectorXd lb_;  // Lower bound on U
    Eigen::VectorXd ub_;  // Upper bound on U
    Eigen::VectorXd lbC_; // Lower constraint bound
    Eigen::VectorXd ubC_; // Upper Constraint bound

    // TODO: Need qpOASES::SQProblem for MPC and Varying QP Matrices(H,g,C)
    //qpOASES::QProblem qp_;
    qpOASES::QProblemB qp_; // This will probably go away.  Just use QProblem with 0 constraints. Will be slighly less efficient
    qpOASES::SQProblem sqp_;
    int num_vars_; // Number of Variables in QP Problem
    int num_cons_; // Number of Constraints in QP Problem

    bool is_hot; // Is the QP Problem warmed up?
    bool time_varying_; // Is system a LTV?
    bool is_sequential_;  // This problem type is sequential. i.e. run in a loop such as MPC to product control sequences over time.
};
} // namespace LinearOptimalControl
} // namespace OptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_LINEARCONDENSEDOCP_H_