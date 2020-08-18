/*
 * NomadDynamics.hpp
 *
 *  Created on: Jule 2, 2020
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

#ifndef ROBOT_NOMAD_NOMADDYNAMICS_H_
#define ROBOT_NOMAD_NOMADDYNAMICS_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>

#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

// Project Includes
#include <Systems/SystemBlock.hpp>
#include <Nomad/NomadRobot.hpp>
#include <Nomad/MessageTypes/com_state_t.hpp>
#include <Nomad/MessageTypes/joint_state_t.hpp>
#include <Nomad/MessageTypes/full_state_t.hpp>


namespace Robot::Nomad::Dynamics
{
    constexpr int kNumTotalDofs = 18;
    constexpr int kNumActuatedDofs = 12;
    constexpr int kNumFloatingDofs = 6;
    constexpr int kNumContacts = 4;

    class NomadDynamics : public Core::Systems::SystemBlock
    {

    public:
        enum OutputPort
        {
            FULL_STATE = 0, // Full Dynamics State Output
            NUM_OUTPUTS = 1
        };

        enum InputPort
        {
            BODY_STATE_HAT = 0, // Estimated Body State from Fused State Estimate
            JOINT_STATE = 1,    // Joint State (q, q_dot)
            NUM_INPUTS = 1
        };

        // Base Class Nomad Dynamics Task Node
        // name = Task Name
        // stack_size = Task Thread Stack Size
        // rt_priority = Task Thread Priority
        // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
        // rt_core_id = CPU Core to pin the task.  -1 for no affinity
        NomadDynamics(const double T_s = -1);

        // Set Dart Robot Skeleton
        void SetRobotSkeleton(dart::dynamics::SkeletonPtr robot);

    protected:

        // Update function for stateful outputs
        void UpdateStateOutputs();

        // Update function for stateless outputs
        void UpdateStatelessOutputs();

        // Update fucntion for next state from inputs
        void UpdateState();

        // (Input) Actuated Joint State Estimate
        joint_state_t joint_state_;

        // (Input) CoM State
        com_state_t com_state_;

        // (Output) Full Robot State
        full_state_t full_state_;

        // Dart Helpers
        dart::dynamics::SkeletonPtr robot_;

        // Body Node for Floating Base Body
        dart::dynamics::BodyNodePtr base_body_;

        // Body Nodes for Hip Base Bodies
        dart::dynamics::BodyNodePtr hip_base_body_[NUM_LEGS];

        // Body Nodes for Foot EE Bodies
        dart::dynamics::BodyNodePtr foot_body_[NUM_LEGS];

        // Degree of Freedom Ptrs for Free DOFs
        dart::dynamics::DegreeOfFreedomPtr DOF_[NUM_TOTAL_DOFS]; // See DOFIdx Enum

        // Full Stacked Leg Jacobian
        Eigen::MatrixXd J_legs_;

        // Selector Matrices
        Eigen::MatrixXd S_j_; // Actuated Joint Selection Matrix

        Eigen::MatrixXd S_f_; // Floating Base Selection Matrix
    };
} // namespace Robot::Nomad::Dynamics

#endif // ROBOT_NOMAD_NOMADDYNAMICS_H_
