/*
 * RigidBody.cpp
 *
 *  Created on: July 18, 2019
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

// Primary Include
#include <Systems/RigidBody.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>

// Project Include Files
#include <OptimalControl/ControlsLibrary.hpp>

RigidBlock1D::RigidBlock1D() : LinearDynamicalSystem(2, 1, 1e-1),
                               mass_(1)
{
    length_ = 1;
    width_ = 1;
    height_ = 1;
}
RigidBlock1D::RigidBlock1D(const double &mass,
                           const Eigen::Vector3d &box_shape,
                           const double &T_s /* = 1e-1*/) : LinearDynamicalSystem(2, 1, T_s),
                                                            mass_(mass)
{
    //std::cout << "Num States: " << num_states << std::endl;
    length_ = box_shape[0];
    width_ = box_shape[1];
    height_ = box_shape[2];

    // Setup initial states
    SetState(Eigen::VectorXd::Zero(num_states_));

    // Setup System Matrices
    A_ << 0, 1,
        0, 0;

    // Setup Input Matrix
    B_ << 0,
        1.0 / mass_;

    // Cache Discrete Time Variant
    ControlsLibrary::ContinuousToDiscrete(A_, B_, T_s_, A_d_, B_d_);
}

void RigidBlock1D::Step(const Eigen::VectorXd &u)
{
    x_ = A_d_ * x_ + B_d_ * u;
}

void RigidBlock1D::Step(double u)
{
    x_ = A_d_ * x_ + B_d_ * u;
}

void RigidBlock1D::Update()
{
}