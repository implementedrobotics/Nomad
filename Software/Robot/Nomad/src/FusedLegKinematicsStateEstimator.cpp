/*
 * FusedLegKinematicsStateEstimator.cpp
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

// Primary Include
#include <Nomad/Estimators/FusedLegKinematicsStateEstimator.hpp>

// C System Includes

// C++ System Includes
#include <iostream>
#include <string>
#include <sstream>
#include <memory>

// Third-Party Includes
#include <zcm/zcm-cpp.hpp>

// Project Includes
#include <Realtime/RealTimeTask.hpp>
#include <Common/Time.hpp>

namespace Robot::Nomad::Estimators
{
    // TODO: Static Variable in "Physics" Class somewhere
    double kGravity = 9.81;

    FusedLegKinematicsStateEstimator::FusedLegKinematicsStateEstimator(const double T_s)  : SystemBlock("Leg_Kin_State_Estimator_Task", T_s)
    {
        // State Estimate Input Port
        input_port_map_[InputPort::IMU_DATA] = Communications::Port<imu_data_t>::CreateInput("IMU_STATE");
        input_port_map_[InputPort::JOINT_STATE] = Communications::Port<joint_state_t>::CreateInput("JOINT_STATE");
        input_port_map_[InputPort::COM_STATE] = Communications::Port<com_state_t>::CreateInput("POSE_STATE");
        input_port_map_[InputPort::FOOT_STATE] = Communications::Port<full_state_t>::CreateInput("FOOT_STATE");

        // State Estimate Output Port
        output_port_map_[OutputPort::BODY_STATE_HAT] = Communications::Port<com_state_t>::CreateOutput("BODY_STATE_HAT");
        //output_port_map_[OutputPort::BODY_STATE_ACTUAL] = Communications::Port::CreateOutput("BODY_STATE_ACTUAL");
    }

    // Update function for stateful outputs
    void FusedLegKinematicsStateEstimator::UpdateStateOutputs()
    {
        // Receive Data
    }

    // Update function for stateless outputs
    void FusedLegKinematicsStateEstimator::UpdateStatelessOutputs()
    {
        // Read Inputs
        GetInputPort(InputPort::COM_STATE)->Receive(com_state_in_);

        // This is the "perfect" state estimate.  For now we pretend that we actually
        // are estimating
        com_state_out_.orientation[0] = com_state_in_.orientation[0]; // X
        com_state_out_.orientation[1] = com_state_in_.orientation[1]; // Y
        com_state_out_.orientation[2] = com_state_in_.orientation[2]; // Z
        com_state_out_.orientation[3] = com_state_in_.orientation[3]; // W

        com_state_out_.pos_world[0] = com_state_in_.pos_world[0];
        com_state_out_.pos_world[1] = com_state_in_.pos_world[1];
        com_state_out_.pos_world[2] = com_state_in_.pos_world[2];

        com_state_out_.omega_world[0] = com_state_in_.omega_world[0];
        com_state_out_.omega_world[1] = com_state_in_.omega_world[1];
        com_state_out_.omega_world[2] = com_state_in_.omega_world[2];

        com_state_out_.vel_world[0] = com_state_in_.vel_world[0];
        com_state_out_.vel_world[1] = com_state_in_.vel_world[1];
        com_state_out_.vel_world[2] = com_state_in_.vel_world[2];

        // Compute State "Estimate".  For now this is just a direct copy from the sim, i.e. "perfect"
        // TODO: Could add noise here to make it a bit more interesting
        com_state_hat_.orientation[0] = com_state_in_.orientation[0]; // X
        com_state_hat_.orientation[1] = com_state_in_.orientation[1]; // Y
        com_state_hat_.orientation[2] = com_state_in_.orientation[2]; // Z
        com_state_hat_.orientation[3] = com_state_in_.orientation[3]; // W

        com_state_hat_.pos_world[0] = com_state_in_.pos_world[0];
        com_state_hat_.pos_world[1] = com_state_in_.pos_world[1];
        com_state_hat_.pos_world[2] = com_state_in_.pos_world[2];

        com_state_hat_.omega_world[0] = com_state_in_.omega_world[0];
        com_state_hat_.omega_world[1] = com_state_in_.omega_world[1];
        com_state_hat_.omega_world[2] = com_state_in_.omega_world[2];

        com_state_hat_.vel_world[0] = com_state_in_.vel_world[0];
        com_state_hat_.vel_world[1] = com_state_in_.vel_world[1];
        com_state_hat_.vel_world[2] = com_state_in_.vel_world[2];

        // Send out the Body State
        GetOutputPort(OutputPort::BODY_STATE_HAT)->Send(com_state_hat_);
    }

    // Update function for next state from inputs
    void FusedLegKinematicsStateEstimator::UpdateState()
    {

    }


} // namespace Robot::Nomad::Estimators
