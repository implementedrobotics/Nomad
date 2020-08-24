
/*
 * ControlsLibrary.cpp
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

#include <OptimalControl/ControlsLibrary.hpp>

void ControlsLibrary::ContinuousToDiscrete(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, double Ts, Eigen::MatrixXd &A_d, Eigen::MatrixXd &B_d)
{
    int num_states = A.cols(); // Get number of states
    int num_inputs = B.cols(); // Get number of inputs

    // Create placeholder for operations on matrices to convert to discrete time
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(num_states + num_inputs, num_states + num_inputs);
    M.block(0, 0, num_states, num_states) = A * Ts;
    M.block(0, num_states, num_states, num_inputs) = B * Ts;

    // Compute Matrix Exponential
    Eigen::MatrixXd M_d = M.exp();

    // Extract the discrete time dynamics matrices
    A_d = M_d.block(0, 0, num_states, num_states);
    B_d = M_d.block(0, num_states, num_states, num_inputs);
}
