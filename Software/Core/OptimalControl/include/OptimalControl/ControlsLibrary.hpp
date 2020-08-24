/*
 * ControlsLibrary.hpp
 *
 *  Created on: July 5, 2019
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

#ifndef NOMAD_CORE_OPTIMALCONTROL_CONTROLSLIBRARY_H
#define NOMAD_CORE_OPTIMALCONTROL_CONTROLSLIBRARY_H

#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace ControlsLibrary
{

    /**
     * Converts the linear continuous time dynamical systems model to disrete time
     * using the Zero Order Hold method
     *
     * @param[in] A Continuous time system(state) transition matrix
     * @param[in] B Continuous time input matrix
     * @param[in] Ts Sampling time
     * @param[out] A_d Discr#include <qpOASES.hpp>ete time system(state) transition matrix
     * @param[out] B_d Discrete time input matrix
     * @param[in] data the data to analyze
     */
    void ContinuousToDiscrete(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, double Ts, Eigen::MatrixXd &A_d, Eigen::MatrixXd &B_d);

} // namespace ControlsLibrary

#endif // NOMAD_CORE_OPTIMALCONTROL_CONTROLSLIBRARY_H