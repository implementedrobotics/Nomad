/*
 * NomadRobot.h
 *
 *  Created on: June 23, 2020
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

#ifndef NOMAD_ROBOT_H_
#define NOMAD_ROBOT_H_

// C System Files

// C++ System Files

// Third Party Includes
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

// Project Include Files

// Nomad Robot Class
class NomadRobot
{

public:
  // Base Class Nomad Robot
  NomadRobot(const dart::simulation::WorldPtr world);
  void LoadFromURDF(const std::string &urdf);

  void ProcessInputs();
  void Run(double dt);
  void SendOutputs();

  void UpdateState();

  // GetState() 18 dof
  void SetInitialPose();
  void Reset();

  Eigen::Quaterniond GetBodyOrientation() const;
  Eigen::Vector3d GetAngularAcceleration() const;
  Eigen::Vector3d GetLinearAcceleration() const;

  const dart::dynamics::SkeletonPtr& Skeleton(){return robot_;}

protected:

  dart::dynamics::SkeletonPtr robot_;
  dart::simulation::WorldPtr world_;

  // State Vector
  Eigen::VectorXd X_;

  // Inputs Vector
  Eigen::VectorXd U_;

};

#endif // NOMAD_ROBOT_H_