/*
 * NomadRobot.cpp
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

// Primary Include
#include <Nomad/NomadRobot.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>

// Third-Party Includes
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/BodyNode.hpp>

// Project Includes

namespace Robot
{
    namespace Nomad
    {

        NomadRobot::NomadRobot()
        {
        }

        dart::dynamics::SkeletonPtr NomadRobot::Load(const std::string &urdf)
        {
            dart::utils::DartLoader loader;
            dart::dynamics::SkeletonPtr robot = loader.parseSkeleton(urdf);

            // Rename the floating base dofs
            robot->getDof(0)->setName("omega_x");
            robot->getDof(1)->setName("omega_y");
            robot->getDof(2)->setName("omega_z");
            robot->getDof(3)->setName("base_x");
            robot->getDof(4)->setName("base_y");
            robot->getDof(5)->setName("base_z");

            // Set position limits enforcement
            robot->getJoint("j_kfe_FL")->setPositionLimitEnforced(true);
            robot->getJoint("j_kfe_FR")->setPositionLimitEnforced(true);
            robot->getJoint("j_kfe_RL")->setPositionLimitEnforced(true);
            robot->getJoint("j_kfe_RR")->setPositionLimitEnforced(true);

            int i = 0;
            for (auto dof : robot->getDofs())
            {
                std::cout << "DOF: " << i++ << " : " << dof->getName() << std::endl;
            }

            std::cout << "Mass: " << robot->getMass() << std::endl;

            // Set Intial Pose
            robot->getDof("base_z")->setPosition(0.2);
            robot->getDof("j_hfe_FL")->setPosition(-M_PI_2);
            robot->getDof("j_hfe_FR")->setPosition(M_PI_2);
            robot->getDof("j_hfe_RL")->setPosition(-M_PI_2);
            robot->getDof("j_hfe_RR")->setPosition(M_PI_2);

            robot->getDof("j_kfe_FL")->setPositionLimits(-2.2, 0.0);
            robot->getDof("j_kfe_FR")->setPositionLimits(0.0, 2.2);
            robot->getDof("j_kfe_RL")->setPositionLimits(-2.2, 0.0);
            robot->getDof("j_kfe_RR")->setPositionLimits(0.0, 2.2);

            //robot_->getDof("j_kfe_RL")->setPosition(-2.2);
            //robot_->getDof("j_kfe_FL")->setPosition(-2.2);
            //robot_->getDof("j_kfe_RR")->setPosition(2.2);
            //robot_->getDof("j_kfe_FR")->setPosition(2.2);
            return robot;
        }
    } // namespace Nomad
} // namespace Robot
