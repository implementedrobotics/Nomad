
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
    OptimalControlProblem(const int &N, const double &T, const int &num_states, const int &num_inputs);

    // Solve
    virtual void Solve() = 0;

    // Get System State Vector
    Eigen::MatrixXd X() const { return _X; }

    //Get Current Input Solution
    Eigen::MatrixXd U() const { return _U; }

    // Set Weight Matrices
    void SetWeights(const Eigen::VectorXd &Q, const Eigen::VectorXd &R)
    {
        // TODO: Verify Vector Size Matches correct state and inputs
        _Q = Q.matrix().asDiagonal();
        _R = R.matrix().asDiagonal(); 
    }

    // Set Problem Initial Condition
    void SetInitialCondition(Eigen::VectorXd &x_0) { _x_0 = x_0; }

    // Set Reference Trajectory
    void SetReference(Eigen::VectorXd &X_ref) { _X_ref = X_ref; }

protected:
    Eigen::VectorXd _x_0;   // Current State/Initial Condition
    Eigen::MatrixXd _X_ref; // Reference Trajectory

    Eigen::MatrixXd _X; // System State
    Eigen::MatrixXd _U; // Optimal Input Solution

    Eigen::MatrixXd _Q; // State Weights
    Eigen::MatrixXd _R; // Input Weights

    int num_states; // Number of System States
    int num_inputs; // Number of System Inputs

    int N; // Number of Prediction Steps

    double T_s; // Sample Time
    double T;   // Horizon Length
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
    LinearOptimalControlProblem(const int &N, const double &T, const int &num_states, const int &num_inputs);

    // Set Weight Matrices
    void SetModelMatrices(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
    {
        _A = A;
        _B = B;
    }

    // Solve
    virtual void Solve();

protected:
    Eigen::MatrixXd _A; // System State Transition Matrix
    Eigen::MatrixXd _B; // Input Matrix
};
} // namespace LinearOptimalControl
} // namespace OptimalControl

#endif // NOMAD_CORE_OPTIMALCONTROL_OPTIMALCONTROLPROBLEM_H_