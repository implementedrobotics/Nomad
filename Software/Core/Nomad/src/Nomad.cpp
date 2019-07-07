/*
 * Nomad.cpp
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

#include <Nomad/Nomad.hpp>
#include <plotty/matplotlibcpp.hpp>
#include <OptimalControl/OptimalControlProblem.hpp>
#include <OptimalControl/LinearCondensedOCP.hpp>
#include <iostream>
#include <chrono> 


RigidBlock1D::RigidBlock1D(const double &mass,
                           const Eigen::Vector3d &box_shape,
                           const double &T_s /* = 1e-1*/) : LinearDynamicalSystem(2, 1, T_s),
                                                            mass(mass)
{
    //std::cout << "Num States: " << num_states << std::endl;
    length = box_shape[0];
    width = box_shape[1];
    height = box_shape[2];

    // Setup initial states
    SetState(Eigen::VectorXd::Zero(num_states));

    // Setup System Matrices
    _A << 0, 1,
          0, 0;

    // Setup Input Matrix
    _B << 0,
        1.0 / mass;

    //std::cout << "A: " << std::endl
    //          << _A << std::endl;
    //std::cout << "B: " << std::endl
    //          << _B << std::endl;

    // Cache Discrete Time Variant
    ControlsLibrary::ContinuousToDiscrete(_A, _B, T_s, _A_d, _B_d);
}

void RigidBlock1D::Step(const Eigen::VectorXd &u)
{
    _x = _A_d * _x + _B_d * u;
}

void RigidBlock1D::Update()
{
    
}

using namespace OptimalControl::LinearOptimalControl;
using namespace ControlsLibrary;

int main()
{

    EigenHelpers::BlockMatrixXd bm_test = EigenHelpers::BlockMatrixXd(24, 24, 13, 13);
    Eigen::MatrixXd block_val = Eigen::MatrixXd::Ones(13,13);

    // Get starting timepoint 
    auto start = std::chrono::high_resolution_clock::now(); 
    bm_test.FillDiagonal(block_val, 0);
    // Get ending timepoint 
    auto stop = std::chrono::high_resolution_clock::now(); 


    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 

    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl; 

    //bm_test(1,1, block_val);
    std::cout << bm_test << std::endl;

    return 0;
    LinearCondensedOCP ocp = LinearCondensedOCP(10, 1.0, 4,4,false);

    Eigen::VectorXd Q(2);
    Q[0] = 1.0;
    Q[1] = 2.0;

    Eigen::VectorXd R(1);
    R[0] = 0.01;
    ocp.SetWeights(Q, R);
    RigidBlock1D block = RigidBlock1D(1.0, Eigen::Vector3d(1.0, 0.5, 0.25));

    Eigen::VectorXd initial_state(2);
    initial_state[0] = 0.0;
    initial_state[1] = 0.0;

    int num_steps = 100;
    block.SetState(initial_state);
    Eigen::VectorXd input(1);
    Eigen::MatrixXd plot_me(2, num_steps);
    input[0] = 10.5;
    for (int i = 0; i < num_steps; i++)
    {
        block.Step(input);
        plot_me.col(i) = block.GetState();
    }
    

    plotty::labelPlot("pos", plot_me.row(0));
    plotty::labelPlot("vel", plot_me.row(1));
    plotty::legend();
    plotty::show();
}
