/*
 * CubicPolynomialTrajectory.cpp
 *
 *  Created on: June 21, 2020
 *      Author: Quincy Jones
 *
 * Copyright (c) <2020> <Quincy Jones - quincy@implementedrobotics.com/>
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

// C System Files

// C++ System Files

// Third Party Includes

// Project Include Files
#include <Common/Math/CubicPolynomialTrajectory.hpp>

namespace Common
{
    CubicPolynomialTrajectory::CubicPolynomialTrajectory(double q_f, double t_f)
        : q_0_(0.0), q_f_(q_f), v_0_(0.0), v_f_(0.0), t_0_(0.0), t_f_(t_f)
    {
        // Compute Coefficients
        ComputeCoeffs();
    }
    CubicPolynomialTrajectory::CubicPolynomialTrajectory(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f)
        : q_0_(q_0), q_f_(q_f), v_0_(v_0), v_f_(v_f), t_0_(t_0), t_f_(t_f)
    {
        // Compute Coefficients
        ComputeCoeffs();
    }

    CubicPolynomialTrajectory::CubicPolynomialTrajectory() // Empty Trajectory
        : q_0_(0.0), q_f_(0.0), v_0_(0.0), v_f_(0.0), t_0_(0.0), t_f_(0.0)
    {
        // Compute Coefficients
        ComputeCoeffs();
    }

    void CubicPolynomialTrajectory::Generate(double q_f, double t_f)
    {
        q_0_ = 0.0;
        q_f_ = q_f;
        v_0_ = 0.0;
        v_f_ = 0.0;
        t_0_ = 0.0;
        t_f_ = t_f;

        ComputeCoeffs();
    }

    void CubicPolynomialTrajectory::Generate(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f)
    {
        q_0_ = q_0;
        q_f_ = q_f;
        v_0_ = v_0;
        v_f_ = v_f;
        t_0_ = t_0;
        t_f_ = t_f;

        ComputeCoeffs();
    }

    // TODO: Check for valid t between 0<->t_f
    double CubicPolynomialTrajectory::Position(double t)
    {
        // Trim Position
        // TODO: Function for this trimming/mapping
        double t_eval = std::min(t, t_f_);
        t_eval = std::max(t_eval, t_0_);
        return a_(0) + a_(1) * t_eval + a_(2) * t_eval * t_eval + a_(3) * t_eval * t_eval * t_eval;
    }
    double CubicPolynomialTrajectory::Velocity(double t)
    {
        // Trim Position
        // TODO: Function for this trimming/mapping
        double t_eval = std::min(t, t_f_);
        t_eval = std::max(t_eval, t_0_);
        return a_(1) + 2 * a_(2) * t_eval + 3 * a_(3) * t_eval * t_eval;
    }
    double CubicPolynomialTrajectory::Acceleration(double t)
    {
        // Trim Position
        // TODO: Function for this trimming/mapping
        double t_eval = std::min(t, t_f_);
        t_eval = std::max(t_eval, t_0_);
        return 2 * a_(2) + 6 * a_(3) * t_eval;
    }

    void CubicPolynomialTrajectory::ComputeCoeffs()
    {

        Eigen::Matrix4d C; // Constraints
        C << 1, t_0_, t_0_ * t_0_, t_0_ * t_0_ * t_0_,
            0, 1, 2 * t_0_, 3 * t_0_ * t_0_,
            1, t_f_, t_f_ * t_f_, t_f_ * t_f_ * t_f_,
            0, 1, 2 * t_f_, 3 * t_f_ * t_f_;

        Eigen::Vector4d b;
        b << q_0_, v_0_, q_f_, v_f_;

        // Solve for Coefficients
        a_ = C.lu().solve(b);
    }
} // namespace Common