/*
 * NomadRobot.hpp
 *
 *  Created on: June 29, 2020
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

#ifndef ROBOT_NOMADROBOT_H_
#define ROBOT_NOMADROBOT_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <dart/dynamics/Skeleton.hpp>

// Project Includes

namespace Robot::Nomad
{
    // Indexing for leg order in array
    enum LegIdx
    {
        FRONT_LEFT = 0,
        FRONT_RIGHT = 1,
        REAR_LEFT = 2,
        REAR_RIGHT = 3,
        NUM_LEGS = 4
    };

    // Indexing for leg order in array
    enum FootIdx
    {
        FOOT_FL_X = (FRONT_LEFT * 3) + 0,
        FOOT_FL_Y = (FRONT_LEFT * 3) + 1,
        FOOT_FL_Z = (FRONT_LEFT * 3) + 2,

        FOOT_FR_X = (FRONT_RIGHT * 3) + 0,
        FOOT_FR_Y = (FRONT_RIGHT * 3) + 1,
        FOOT_FR_Z = (FRONT_RIGHT * 3) + 2,

        FOOT_RL_X = (REAR_LEFT * 3) + 0,
        FOOT_RL_Y = (REAR_LEFT * 3) + 1,
        FOOT_RL_Z = (REAR_LEFT * 3) + 2,

        FOOT_RR_X = (REAR_RIGHT * 3) + 0,
        FOOT_RR_Y = (REAR_RIGHT * 3) + 1,
        FOOT_RR_Z = (REAR_RIGHT * 3) + 2
    };

    // Indexing for state vector offsets
    enum DOFIdx
    {
        BODY_PHI = 0,   // Body Roll
        BODY_THETA = 1, // Body Pitch
        BODY_PSI = 2,   // Body Yaw
        BODY_X = 3,     // Body X Position
        BODY_Y = 4,     // Body Y Position
        BODY_Z = 5,     // Body Z Position
        HAA_FL = 6,     // Front Left Leg Hip Ab/Ad Joint State
        HFE_FL = 7,     // Front Left Leg Hip Flexion/Extension Joint State
        KFE_FL = 8,     // Front Left Leg Knee Flexion/Extension Joint State
        HAA_FR = 9,     // Front Right Leg Hip Ab/Ad Joint State
        HFE_FR = 10,    // Front Right Leg Hip Flexion/Extension Joint State
        KFE_FR = 11,    // Front Right Leg Knee Flexion/Extension Joint State
        HAA_RL = 12,    // Rear Left Leg Hip Ab/Ad Joint State
        HFE_RL = 13,    // Rear Left Leg Hip Flexion/Extension Joint State
        KFE_RL = 14,    // Rear LeftLeg Knee Flexion/Extension Joint State
        HAA_RR = 15,    // Rear Right Leg Hip Ab/Ad Joint State
        HFE_RR = 16,    // Rear Right Leg Hip Flexion/Extension Joint State
        KFE_RR = 17,    // Rear Right Leg Knee Flexion/Extension Joint State
        NUM_TOTAL_DOFS = 18
    };

    class NomadRobot
    {

    public:
        // Base Class Robot Nomad Node
        NomadRobot();

        // Load Dart Skeleton from URDF
        static dart::dynamics::SkeletonPtr Load(const std::string &urdf);

    protected:
    };
} // namespace Robot::Nomad

#endif // ROBOT_NOMADROBOT_H_
