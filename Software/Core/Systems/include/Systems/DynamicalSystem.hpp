
/*
 * DynamicalSystem.hpp
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


#ifndef NOMAD_CORE_SYSTEMS_DYNAMICALSYSTEM_H_
#define NOMAD_CORE_SYSTEMS_DYNAMICALSYSTEM_H_

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

class DynamicalSystem
{

public:
    DynamicalSystem(const int& num_states, const double& Ts = 1e-1);

    // Step System
    virtual void Step(const Eigen::VectorXd& u) = 0;

    // Update
    virtual void Update() = 0;

    // Get Current State
    Eigen::VectorXd GetState() const { return _x; }

    // Set Current State
    void SetState(const Eigen::VectorXd &x) { _x = x; }



protected:

    Eigen::VectorXd _x;  // State Vector

    int num_states; // Number of System States

    double T_s; // Sample Time
    double T; // Current Time
};


class NonLinearDynamicalSystem : public DynamicalSystem
{
    
protected:

    virtual void f(const Eigen::MatrixXd& x, const Eigen::MatrixXd& u) = 0;

};

class LinearDynamicalSystem : public DynamicalSystem
{

public:

    LinearDynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s = 1e-1);

    // Model Matrices
    Eigen::MatrixXd A() const { return _A; };
    Eigen::MatrixXd B() const { return _B; };

    // Discrete Time Model Matrices
    Eigen::MatrixXd A_d() const { return _A_d; };
    Eigen::MatrixXd B_d() const { return _B_d; };

protected:

    int num_inputs;     // Number of inputs

    Eigen::MatrixXd _A;  // State Transition Mat rix
    Eigen::MatrixXd _B;  // Input Matrix

    Eigen::MatrixXd _A_d;  // Discrete Time State Transition Mat rix
    Eigen::MatrixXd _B_d;  // Discrete Time Input Matrix

};

class LinearTimeVaryingDynamicalSystem : public LinearDynamicalSystem
{

public:

    LinearTimeVaryingDynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s = 1e-1);

    // Get the Time-Varying Matrices from Model
    virtual void GetModelMatrices(const int& N, Eigen::MatrixXd& A, Eigen::MatrixXd& B) = 0;


    // Model Matrices (Time Varying)
    std::vector<Eigen::MatrixXd> A_TV() const { return _A_tv; };
    std::vector<Eigen::MatrixXd> B_TV() const { return _B_tv; };

    // Discrete Time Model Matrices (Time Varying)
    std::vector<Eigen::MatrixXd> A_d_TV() const { return _A_d_tv; };
    std::vector<Eigen::MatrixXd> B_d_TV() const { return _B_d_tv; };

protected:

    // Time Varying Matrices
    std::vector<Eigen::MatrixXd> _A_tv;
    std::vector<Eigen::MatrixXd> _B_tv;

    std::vector<Eigen::MatrixXd> _A_d_tv;
    std::vector<Eigen::MatrixXd> _B_d_tv;
};

#endif // NOMAD_CORE_SYSTEMS_DYNAMICALSYSTEM_H_