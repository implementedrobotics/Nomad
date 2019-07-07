
/*
 * OptimalControlProblem.cpp
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

namespace OptimalControl
{

OptimalControlProblem::OptimalControlProblem(const int &N, const double &T, const int &num_states, const int &num_inputs) : T(T), N(N), num_states(num_states), num_inputs(num_inputs)
{
    _x_0 = Eigen::VectorXd(num_states);
    _X_ref = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, N);

    _X = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, N);
    _U = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs, N-1);

    _Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_states);
    _R = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_inputs, num_inputs);

    T_s = T / N;
}
} // namespace OptimalControl



namespace OptimalControl
{
namespace LinearOptimalControl
{
LinearOptimalControlProblem::LinearOptimalControlProblem(const int &N, const double &T, const int &num_states, const int &num_inputs) : OptimalControlProblem(N, T, num_states, num_inputs)
{
    _A = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_states);
    _B = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(num_states, num_inputs);
}
void LinearOptimalControlProblem::Solve()
{

}
} // namespace LinearOptimalControl
} // namespace OptimalControl