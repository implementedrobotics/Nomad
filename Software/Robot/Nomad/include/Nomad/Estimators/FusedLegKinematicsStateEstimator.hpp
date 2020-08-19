/*
 * FusedLegKinematicsStateEstimator.hpp
 *
 *  Created on: July 28, 2020
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
#include <Systems/SystemBlock.hpp>
#include <Nomad/MessageTypes/imu_data_t.hpp>
#include <Nomad/MessageTypes/com_state_t.hpp>
#include <Nomad/MessageTypes/joint_state_t.hpp>
#include <Nomad/MessageTypes/full_state_t.hpp>

namespace Robot::Nomad::Estimators
{
    class FusedLegKinematicsStateEstimator : public Core::Systems::SystemBlock
    {

    public:
        enum OutputPort
        {
            BODY_STATE_HAT = 0,    // State Estimate
            BODY_STATE_ACTUAL = 1, // Ground Truth Body State
            NUM_OUTPUTS = 2
        };

        enum InputPort
        {
            IMU_DATA = 0,    // IMU Sensor Input
            JOINT_STATE = 1, // Joint State for Leg Kinematics Input
            VISUAL_ODOM = 2, // Visual Odometry Sensor Input
            COM_STATE = 3,   // Ground Truth Pose Data
            FOOT_STATE = 4,  // Foot State
            NUM_INPUTS = 5
        };

        // Base Class State Estimator Task Node
        // T_s = System sampling time
        FusedLegKinematicsStateEstimator(const double T_s = -1);

    protected:

        // Update function for stateful outputs
        void UpdateStateOutputs();

        // Update function for stateless outputs
        void UpdateStatelessOutputs();

        // Update fucntion for next state from inputs
        void UpdateState();
        

        // (Input) Actuated Joint State Estimate
        joint_state_t joint_state_;

        // (Input) IMU State
        imu_data_t imu_data_;

        // (Input) COM State Input (Absolute Externally Measured(Vive Tracker) or Sim)
        com_state_t com_state_in_;

        // (Input) Foot Position State
        // TODO: Add a smaller new message for just the foot position/velocity
        full_state_t full_state_in_;

        // (Output) COM State Output
        com_state_t com_state_out_;

        // (Output) COM State Estimate Output
        com_state_t com_state_hat_;

        // (Output) Contact Estimate
        //int32_vec_t contact_state_out_;
    };
} // namespace Robot::Nomad::Estimators

#endif // ROBOT_NOMAD_ESTIMATORS_FUSEDLEGKINEMATICSSTATEESTIMATOR_H_
