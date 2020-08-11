
/*
 * CubicPolynomialTrajectory.h
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

#ifndef NOMAD_CUBICPOLYNOMIALTRAJECTORY_H_
#define NOMAD_CUBICPOLYNOMIALTRAJECTORY_H_

// C System Files

// C++ System Files

// Third Party Includes
#include <Eigen/Dense>

// Project Include Files
namespace Common
{
    class CubicPolynomialTrajectory
    {

    public:
        CubicPolynomialTrajectory(double q_f, double t_f);
        CubicPolynomialTrajectory(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f);
        CubicPolynomialTrajectory(); // Empty Trajectory

        void Generate(double q_f, double t_f);
        void Generate(double q_0, double q_f, double v_0, double v_f, double t_0, double t_f);

        // TODO: Check for valid t between 0<->t_f
        double Position(double t);
        double Velocity(double t);
        double Acceleration(double t);

    protected:
        void ComputeCoeffs();

        Eigen::Vector4d a_; // Coefficients

        double q_0_;
        double v_0_;
        double t_0_;

        double q_f_;
        double v_f_;
        double t_f_;
    };
} // namespace Common

#endif // NOMAD_CUBICPOLYNOMIALTRAJECTORY_H_