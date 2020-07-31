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
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/int32_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Nomad/MessageTypes/com_state_t.hpp>
#include <Nomad/MessageTypes/joint_state_t.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Controllers/LegController.hpp>
#include <Nomad/NomadRobot.hpp>

namespace Robot
{
    namespace Nomad
    {
        namespace Dynamics

        {
            constexpr int kNumTotalDofs = 18;
            constexpr int kNumActuatedDofs = 12;
            constexpr int kNumFloatingDofs = 6;
            constexpr int kNumContacts = 4;

            // TODO: Move to a standard hear for all custom tops
            struct nomad_full_state_t
            {
                double q[kNumTotalDofs];        // Robot Position State
                double q_dot[kNumTotalDofs];    // Robot Velocity State
                double foot_pos[kNumActuatedDofs]; // Foot Position State
                double foot_vel[kNumActuatedDofs]; // Foot Velocity State

                double M[kNumTotalDofs * kNumTotalDofs];   // Mass Inertia Matrix
                double g[kNumTotalDofs];        // Gravity Terms Vector
                double b[kNumTotalDofs];        // Coriolis Terms Vector
                double J_c[3 * kNumContacts * kNumTotalDofs]; // Contact Jacobian
            };

            class NomadDynamics : public Realtime::RealTimeTaskNode
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
                    JOINT_STATE = 1,  // Joint State (q, q_dot)
                    NUM_INPUTS = 1
                };

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

                // Base Class Nomad Dynamics Task Node
                // name = Task Name
                // stack_size = Task Thread Stack Size
                // rt_priority = Task Thread Priority
                // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
                // rt_core_id = CPU Core to pin the task.  -1 for no affinity
                NomadDynamics(const std::string &name = "Nomad_Dynamics_Handler",
                              const long rt_period = 10000,
                              const unsigned int rt_priority = Realtime::Priority::MEDIUM,
                              const int rt_core_id = -1,
                              const unsigned int stack_size = PTHREAD_STACK_MIN);

                // Set Dart Robot Skeleton
                void SetRobotSkeleton(dart::dynamics::SkeletonPtr robot);

            protected:
                // Overriden Run Function
                virtual void Run();

                // Pre-Run Setup Routine.  Setup any one time initialization here.
                virtual void Setup();

                // (Input) Joint State
                //generic_msg_t leg_state_msg_;
                //::Controllers::Locomotion::leg_controller_cmd_t leg_state_;
                
                // (Input) Actuated Joint State Estimate
                joint_state_t joint_state_;

                // (Input) CoM State
                com_state_t com_state_;

                // (Output) Full Robot State
                generic_msg_t nomad_full_state_msg_;
                nomad_full_state_t full_state_;

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
        } // namespace Dynamics
    }     // namespace Nomad
} // namespace Robot

#endif // ROBOT_NOMAD_NOMADDYNAMICS_H_
