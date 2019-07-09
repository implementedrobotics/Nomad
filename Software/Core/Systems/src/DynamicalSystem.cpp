/*
 * DynamicalSystem.cpp
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


#include <Systems/DynamicalSystem.hpp>

DynamicalSystem::DynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s /* = 1e-1*/) : T_s_(T_s), t_(0), num_states_(num_states), num_inputs_(num_inputs)
{
    x_ = Eigen::VectorXd(num_states);
}


LinearDynamicalSystem::LinearDynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s /* = 1e-1*/) : DynamicalSystem(num_states, T_s)
{
    A_ = Eigen::MatrixXd(num_states, num_states);
    A_d_ = Eigen::MatrixXd(num_states, num_states);

    B_ = Eigen::MatrixXd(num_states, num_inputs);
    B_d_ = Eigen::MatrixXd(num_states, num_inputs);

}


LinearTimeVaryingDynamicalSystem::LinearTimeVaryingDynamicalSystem(const int& num_states, const int& num_inputs, const double& T_s /* = 1e-1*/) : LinearDynamicalSystem(num_states, num_inputs, T_s)
{
    x_ = Eigen::VectorXd(num_states);
    
}

