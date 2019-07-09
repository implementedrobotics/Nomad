/*
 * Nomad.hpp
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

#ifndef NOMAD_CORE_NOMAD_NOMAD_H
#define NOMAD_CORE_NOMAD_NOMAD_H

#include <OptimalControl/ControlsLibrary.hpp>
#include <Systems/DynamicalSystem.hpp>
#include <Eigen/Dense>

class RigidBlock1D : public LinearDynamicalSystem
{
    public:

        // Constructor
        RigidBlock1D(const double& mass, const Eigen::Vector3d& box_shape, const double& T_s = 1e-1);
 
        // Step System
        void Step(const Eigen::VectorXd& u);
        void Step(double u);

        // Update
        void Update();

        // Mass
        double GetMass() const {return mass_;}
    

    protected:
        double mass_;
        double width_;
        double height_;
        double length_;

};

#endif // NOMAD_CORE_NOMAD_NOMAD_H
