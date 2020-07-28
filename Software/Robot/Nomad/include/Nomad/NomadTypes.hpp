/*
 * NomadTypes.hpp
 *
 *  Created on: July 28, 2020
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

#ifndef ROBOT_NOMAD_TYPES_H_
#define ROBOT_NOMAD_TYPES_H_

// C System Files

// C++ System Files

// Third Party Includes

// Project Includes

namespace Robot
{
    namespace Nomad
    {
        namespace Types
        {
            struct joint_control_cmd_t
            {
                double torque_ff[12]; // Torque Feedforward
                double q[12];         // Joint Position
                double q_d[12];       // Joint Velocity
                double k_p_joint[12]; // P Gains (Joint)
                double k_d_joint[12]; // D Gains (Joint)
            };

            struct joint_state_t
            {
                double q_hat[12];      // Joint Position Estimate
                double q_dot_hat[12];  // Joint Velocity Estimate
                double torque_hat[12]; // Torque Estimate
            };

            struct imu_state_t
            {
                double orientation[4]; // Orientation Quaternion
                double accel[3];       // Linear Acceleration
                double omega[3];       // Angular Velocity
            };
        } // namespace Types
    }     // namespace Nomad
} // namespace Robot

#endif // ROBOT_NOMAD_TYPES_H_