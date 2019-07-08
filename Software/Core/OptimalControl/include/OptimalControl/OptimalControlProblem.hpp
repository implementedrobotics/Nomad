
/*
 * OptimalControlProblem.hpp
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

#include <iostream>
#include <Eigen/Dense>

#ifndef NOMAD_CORE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H_
#define NOMAD_CORE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H_

namespace OptimalControl
{

class OptimalControlProblem
{

public:
    // Base Class Optimal Control Problem
    // N = Prediction Steps
    // T = Horizon Length
    // num_states = Number of States of OCP
    // num_inputs = Number of Inputs of OCP
    OptimalControlProblem(const int N, const double T, const int num_states, const int num_inputs, const int max_terations = 1000);

    // Solve
    virtual void Solve() = 0;

    // Get System State Vector
    Eigen::MatrixXd X() const { return X_; }

    //Get Current Input Solution
    Eigen::MatrixXd U() const { return U_; }

    // Set Weight Matrices
    void SetWeights(const Eigen::VectorXd &Q, const Eigen::VectorXd &R)
    {
        // TODO: Verify Vector Size Matches correct state and inputs
        Q_ = Q.matrix().asDiagonal().toDenseMatrix();
        R_ = R.matrix().asDiagonal().toDenseMatrix();
    }

    // Set Problem Initial Condition
    void SetInitialCondition(Eigen::VectorXd &x_0) { x_0_ = x_0; }

    // Set Reference Trajectory
    void SetReference(Eigen::VectorXd &X_ref) { X_ref_ = X_ref; }

protected:
    Eigen::VectorXd x_0_;   // Current State/Initial Condition
    Eigen::MatrixXd X_ref_; // Reference Trajectory

    Eigen::MatrixXd X_; // System State
    Eigen::MatrixXd U_; // Optimal Input Solution

    Eigen::MatrixXd Q_; // State Weights
    Eigen::MatrixXd R_; // Input Weights

    int num_states_; // Number of System States
    int num_inputs_; // Number of System Inputs

    int N_; // Number of Prediction Steps

    double T_s_; // Sample Time
    double T_;   // Horizon Length

    int max_iterations_; // Max Iterations for QP
};
} // namespace OptimalControl

namespace OptimalControl
{
namespace LinearOptimalControl
{
class LinearOptimalControlProblem : public OptimalControlProblem
{
public:
    // Base Class Linear Optimal Control Problem
    // N = Prediction Steps
    // T = Horizon Length
    // num_states = Number of States of OCP
    // num_inputs = Number of Inputs of OCP
    LinearOptimalControlProblem(const int N, const double T, const int num_states, const int num_inputs, const int max_iterations = 1000);

    // Set Weight Matrices
    void SetModelMatrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
    {
        A_ = A;
        B_ = B;
    }

    // Solve
    virtual void Solve();

protected:
    Eigen::MatrixXd A_; // System State Transition Matrix
    Eigen::MatrixXd B_; // Input Matrix

};
} // namespace LinearOptimalControl
} // namespace OptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H_
