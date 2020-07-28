/*
 * FusedLegKinematicsStateEstimator.hpp
 *
 *  Created on: July 13, 2019
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

#ifndef ROBOT_NOMAD_ESTIMATORS_FUSEDLEGKINEMATICSSTATEESTIMATOR_H_
#define ROBOT_NOMAD_ESTIMATORS_FUSEDLEGKINEMATICSSTATEESTIMATOR_H_

// C System Files
#include <stdint.h>

// C++ System Files
#include <iostream>
#include <string>

// Third Party Includes
#include <Eigen/Dense>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Communications/Messages/double_vec_t.hpp>
#include <Communications/Messages/int32_vec_t.hpp>
#include <Communications/Messages/generic_msg_t.hpp>
#include <Nomad/NomadTypes.hpp>

namespace Robot
{
    namespace Nomad
    {
        namespace Estimators
        {
            class FusedLegKinematicsStateEstimator : public Realtime::RealTimeTaskNode
            {

            public:
                enum OutputPort
                {
                    BODY_STATE_HAT = 0, // State Estimate
                    NUM_OUTPUTS = 1
                };

                enum InputPort
                {
                    IMU_DATA = 0,     // IMU Sensor Input
                    JOINT_STATE = 1,  // Joint State for Leg Kinematics Input
                    VISUAL_ODOM = 2,  // Visual Odometry Sensor Input
                    CHEATER_POSE = 3, // Ground Truth Pose Data
                    NUM_INPUTS = 4
                };

                // TODO: Move to a State class
                enum Idx
                {
                    PHI = 0,    // Body Orientation (Roll)
                    THETA = 1,  // Body Orientation (Pitch)
                    PSI = 2,    // Body Orientation (Yaw)
                    X = 3,      // Body Position (X)
                    Y = 4,      // Body Position (Y)
                    Z = 5,      // Body Position (Z)
                    W_X = 6,    // Angular Vel (Roll)
                    W_Y = 7,    // Angular Vel (Pitch)
                    W_Z = 8,    // Angular Vel (Yaw)
                    X_DOT = 9,  // Body Velocity (X)
                    Y_DOT = 10, // Body Velocity (Y)
                    Z_DOT = 11, // Body Velocity (Z)
                    //GRAVITY = 12 // Augmented Gravity
                };

                // Base Class State Estimator Task Node
                // name = Task Name
                // stack_size = Task Thread Stack Size
                // rt_priority = Task Thread Priority
                // rt_period = Task Execution Period (microseconds), default = 10000uS/100hz
                // rt_core_id = CPU Core to pin the task.  -1 for no affinity
                FusedLegKinematicsStateEstimator(const std::string &name = "Leg_Kin_State_Estimator_Task",
                                                 const long rt_period = 10000,
                                                 const unsigned int rt_priority = Realtime::Priority::MEDIUM,
                                                 const int rt_core_id = -1,
                                                 const unsigned int stack_size = PTHREAD_STACK_MIN);

            protected:
                // Overriden Run Function
                virtual void Run();

                // Pre-Run Setup Routine.  Setup any one time initialization here.
                virtual void Setup();

                // Number of states
                unsigned int num_states_;

                // (Input) Actuated Joint State Estimate
                // double_vec_t joint_state_in_;
                generic_msg_t joint_state_in_;

                // (Input) IMU Estimate
                //double_vec_t imu_data_in_;
                generic_msg_t imu_data_in_;

                // (Input) Ground Truth Pose Estimate
                //double_vec_t imu_data_in_;

                // (Output) State Estimate
                double_vec_t com_state_out_;

                // (Output) Contact Estimate
                int32_vec_t contact_state_out_;
            };
        } // namespace Estimators
    }     // namespace Nomad
} // namespace Robot

#endif // ROBOT_NOMAD_ESTIMATORS_FUSEDLEGKINEMATICSSTATEESTIMATOR_H_
