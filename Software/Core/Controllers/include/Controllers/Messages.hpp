/*
 * Messages.hpp
 *
 *  Created on: July 17, 2019
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

#ifndef NOMAD_CORE_CONTROLLERS_MESSAGES_H_
#define NOMAD_CORE_CONTROLLERS_MESSAGES_H_

// C System Files
#include <stdint.h>

// C++ System Files

// Third Party Includes

// Project Include Files

namespace Messages
{
namespace Generic
{
//template<const int size>
struct VectorArray
{
    // Vector Data Array
    double *data;
};
} // namespace Generic
namespace Controllers
{
namespace Estimators
{

struct CoMState
{
    // State
    double x[13];
};

} // namespace Estimators

namespace Locomotion
{

struct TrajectorySetpoint
{
    // Desired Forward Velocity
    double x_dot;

    // Desired Lateral Velocity
    double y_dot;

    // Desired Yaw(Turn) Rate
    double yaw_dot;

    // Desired CoM Height above ground
    double z_com;
};

//template <unsigned int num_states, unsigned int N>
struct ReferenceTrajectory
{
    // Desired Reference
    double X_ref[13 * 10]; // TODO: Make this variable and working with ZMQ
};
} // namespace Locomotion
} // namespace Controllers
} // namespace Messages

#endif // NOMAD_CORE_CONTROLLERS_MESSAGES_H_